#!/usr/bin/python3

# BSD 3-Clause License

# Copyright (c) 2022, DFKI Niedersachsen
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import List, Tuple, Sequence
import yaml
import time
import numpy
import sys

import rospy
import rospkg
import rosgraph

from copy import deepcopy
from pose_selector.srv import ClassQuery, ClassQueryRequest, GetPoses
from symbolic_fact_generation.common.collision_checking import oriented_collision_check_with_obj_size
from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.lib import split_object_class_from_id

from rospy_message_converter import message_converter
from tf.transformations import quaternion_matrix


def wait_for_services(names, total_s, step_s=5.0, wait=None, log=print):
    '''Wait for every service in names, retrying in step_s slices for up to total_s in all (a slow pose selector
    after a restart used to kill the fact publisher after one 10 s wait, #61). Raises rospy.ROSException naming the
    missing service when the time is up.'''
    wait = wait or rospy.wait_for_service
    deadline = time.monotonic() + max(0.0, float(total_s))
    for name in names:
        while True:
            remaining = deadline - time.monotonic()
            slice_s = max(0.1, min(step_s, remaining))
            started = time.monotonic()
            try:
                wait(name, timeout=slice_s)
                break
            except rospy.ROSInterruptException:
                # rospy is shutting down: rospy.wait_for_service returns at once from now on, so retrying would log
                # "still waiting" in a tight loop (tens of thousands of lines per second froze the tables demo GUI)
                raise
            except rospy.ROSException:
                if deadline - time.monotonic() <= 0.0:
                    raise rospy.ROSException(f"service {name} not available after {total_s:.0f} s")
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException(f"rospy shut down while waiting for service {name}")
                log(f"still waiting for {name} ({deadline - time.monotonic():.0f} s left)")
                # a wait that gave up before its slice elapsed must not turn this loop into a busy loop
                time.sleep(max(0.0, min(1.0, slice_s) - (time.monotonic() - started)))


class OnGenerator(GeneratorInterface):

    def __init__(
        self,
        fact_name: str = "on",
        objects_of_interest: List[str] = [],
        container_objects: List[str] = [],
        query_srv_str: str = "/pick_pose_selector_node/pose_selector_class_query",
        planning_scene_param: str = "/mobipick/pick_object_node/planning_scene_boxes",
        get_all_srv_str: str = None,
        max_support_height: float = 1.5,
    ) -> None:
        try:
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            # the pose selector can take a while after a (re)start of the bringup: wait up to ~pose_selector_wait_s
            wait_for_services([query_srv_str] + ([get_all_srv_str] if get_all_srv_str else []),
                              rospy.get_param("~pose_selector_wait_s", 60.0))
            self._pose_selector_query_srv = rospy.ServiceProxy(query_srv_str, ClassQuery)
            self._pose_selector_get_all_srv = None
            if get_all_srv_str:
                self._pose_selector_get_all_srv = rospy.ServiceProxy(get_all_srv_str, GetPoses)

            self._objects_of_interest = []
            for object_of_interest in objects_of_interest:
                obj_of_interest_class, _ = split_object_class_from_id(object_of_interest)
                if obj_of_interest_class not in self._objects_of_interest:
                    self._objects_of_interest.append(obj_of_interest_class)

            self._container_objects = container_objects
            self._fact_name = fact_name

            package_path = rospkg.RosPack().get_path("symbolic_fact_generation")

            self._planning_scene_object_poses = []

            # if planning scene config file on parameter server
            if rospy.has_param(planning_scene_param):
                print(f"Using {planning_scene_param} parameter.")
                planning_scene_boxes = rospy.get_param(planning_scene_param)
                id_counter = {}
                for box in planning_scene_boxes:
                    class_id, instance_id = split_object_class_from_id(box["scene_name"])
                    if instance_id is None:
                        # create id for different class types starting from 1
                        id_counter[class_id] = id_counter.get(class_id, 1)
                        instance_id = id_counter[class_id]
                        id_counter[class_id] += 1
                    pose = {
                        "class_id": class_id,
                        "instance_id": instance_id,
                        "pose": {
                            "position": {
                                "x": box["box_position_x"],
                                "y": box["box_position_y"],
                                "z": box["box_position_z"],
                            },
                            "orientation": {
                                "x": box["box_orientation_x"],
                                "y": box["box_orientation_y"],
                                "z": box["box_orientation_z"],
                                "w": box["box_orientation_w"],
                            },
                        },
                        "size": {"x": box["box_x_dimension"], "y": box["box_y_dimension"], "z": box["box_z_dimension"]},
                        "min": {
                            "x": -(box["box_x_dimension"] / 2.0),
                            "y": -(box["box_y_dimension"] / 2.0),
                            "z": -(box["box_z_dimension"] / 2.0),
                        },
                        "max": {
                            "x": (box["box_x_dimension"] / 2.0),
                            "y": (box["box_y_dimension"] / 2.0),
                            "z": (box["box_z_dimension"] / 2.0),
                        },
                    }
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message("object_pose_msgs/ObjectPose", pose)
                    )
            else:
                # use default config file
                print("Using symbolic_fact_generation/config/tables_poses.yaml")
                table_poses_yaml = package_path + "/config/table_poses.yaml"
                yamlfile = open(table_poses_yaml, "r")
                yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)

                for pose in yaml_content["poses"]:
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message("object_pose_msgs/ObjectPose", pose)
                    )

            # only surfaces things can stand on: the planning scene also holds the lab ceiling ("roof") and walls,
            # and an object with a wrong pose near them must not become on(klt_1, roof_1)
            self._planning_scene_object_poses = [
                surface
                for surface in self._planning_scene_object_poses
                if is_support_surface(surface, max_support_height)
            ]

        except FileNotFoundError:
            print(
                "[WARNING] No planning scene parameter is set and table_poses.yaml file is not found! Only objects on other objects facts can be generated!"
            )
        except rospy.ROSInitException:
            print("ROS master was shutdown!")
            sys.exit(1)
        except rospy.ROSException as e:
            # raise instead of sys.exit: the fact publisher logs it and keeps publishing every other fact (#61)
            raise RuntimeError(f"on facts disabled, pose selector not available: {e}") from e

        # Object-on-object "on" facts, off by default and read as *private* node
        # parameters: only the fact_publisher's own generator enables them, so the
        # planner's generator (tables_demo_planning/tables_demo_api.py) keeps the
        # table-only behaviour even when a dataset run sets the parameters.
        self._stacking_facts = bool(rospy.get_param("~stacking_facts", False))
        self._stacking_margin_m = float(rospy.get_param("~stacking_margin_m", 0.05))
        self._stacking_min_height_m = float(rospy.get_param("~stacking_min_height_m", 0.03))

    def generate_facts(self):
        # Open-set mode queries the complete pose database, while omitting the
        # get-all service preserves the original configured-class behaviour.
        if self._pose_selector_get_all_srv is not None:
            obj_poses = list(self._pose_selector_get_all_srv().poses.objects)
        else:
            obj_poses = []
            for obj in self._objects_of_interest:
                query_result = self._pose_selector_query_srv(ClassQueryRequest(class_id=obj))
                obj_poses.extend(query_result.poses)

        on_facts = []

        # create new list with container objects
        container_objects = [
            container_obj for container_obj in obj_poses if container_obj.class_id in self._container_objects
        ]

        # iterate over all container objects to create in facts
        for container_obj in container_objects:
            for obj in obj_poses:
                container_obj_name = container_obj.class_id + "_" + str(container_obj.instance_id)
                obj_name = obj.class_id + "_" + str(obj.instance_id)
                new_fact = None
                # no need to check with itself
                if container_obj_name != obj_name:
                    if check_in_condition(obj, container_obj):
                        new_fact = Fact(name="in", values=[obj_name, container_obj_name])

                # add new fact to list if not already there
                if new_fact is not None and new_fact not in on_facts:
                    on_facts.append(new_fact)

        # iterate over all poses
        for surface_obj in self._planning_scene_object_poses:
            for obj in obj_poses:
                surface_obj_name = surface_obj.class_id + "_" + str(surface_obj.instance_id)
                obj_name = obj.class_id + "_" + str(obj.instance_id)
                new_fact = None
                # no need to check with itself
                if surface_obj_name != obj_name:
                    # dont check objects which are in a container
                    if obj_name not in [
                        in_container.values[0]
                        for in_container in on_facts
                        if in_container.name == "in" and in_container.values[0] == obj_name
                    ]:
                        if check_on_condition(obj, surface_obj):
                            new_fact = Fact(name=self._fact_name, values=[obj_name, surface_obj_name])

                # add new fact to list if not already there
                if new_fact is not None and new_fact not in on_facts:
                    on_facts.append(new_fact)

        if self._stacking_facts:
            planning_scene_names = {
                surface.class_id + "_" + str(surface.instance_id)
                for surface in self._planning_scene_object_poses
            }
            in_container_names = {
                fact.values[0] for fact in on_facts if fact.name == "in"
            }
            on_facts.extend(
                stacking_on_facts(
                    obj_poses,
                    exclude_surface_names=planning_scene_names,
                    in_container_names=in_container_names,
                    margin=self._stacking_margin_m,
                    min_height_m=self._stacking_min_height_m,
                    fact_name=self._fact_name,
                )
            )

        return on_facts


def is_support_surface(surface_obj, max_support_height=1.5) -> bool:
    """True when the top of the (upright) box surface_obj is at most max_support_height above the map floor"""
    return surface_obj.pose.position.z + surface_obj.size.z / 2.0 <= max_support_height


def check_in_condition(obj, container_obj) -> bool:
    if oriented_collision_check_with_obj_size(container_obj.pose, container_obj.size, obj.pose, obj.size):
        # calculate euclidean distance to check if obj is in container_obj
        dist = numpy.linalg.norm(
            (
                obj.pose.position.x - container_obj.pose.position.x,
                obj.pose.position.y - container_obj.pose.position.y,
                obj.pose.position.z - container_obj.pose.position.z,
            )
        )
        radius = max(
            container_obj.max.x,
            container_obj.max.y,
            container_obj.max.z,
            container_obj.size.x / 2.0,
            container_obj.size.y / 2.0,
            container_obj.size.z / 2.0,
        )
        # remove 10% of radius for objects colliding with the outside wall
        # still detected as IN for rectangular container objects like klt if close to it
        radius = radius - radius * 0.1
        if dist < radius:
            return True
    return False


def check_on_condition(obj, surface_obj, z_threshold=0.1) -> bool:
    """
    Checks whether `obj` intersects with a box on top of `surface_obj`. That
    collision box has the same x and y dimensions as `surface_obj` and a z
    dimension of `z_threshold`.

    :param z_threshold: distance in meters above the table to count as "on" the table
    """
    # Assumption: surface_obj.pose.orientation is "upright"
    surface_pose = deepcopy(surface_obj.pose)
    surface_size = deepcopy(surface_obj.size)

    surface_pose.position.z = surface_pose.position.z + surface_size.z / 2 + z_threshold / 2
    surface_size.z = z_threshold

    return oriented_collision_check_with_obj_size(surface_pose, surface_size, obj.pose, obj.size)


def object_bounds(obj) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """``(min, max)`` corners of the axis-aligned box of an object in the reference frame.

    The bounds of the message describe the box *in the object frame*, around the
    pose origin (the ground-truth feeder adds the ``center_offset`` of the class
    there), so they are rotated with the pose and the pose position goes on top.
    Rotating matters: the ground-truth edges of a dataset are labelled from the
    same rotated boxes (``gt_bounds`` in the recorded run), and an unrotated
    multimeter or drill reaches 20 cm higher than it really does. A client that
    sends no bounds at all - the pose_selector stores every message as it
    arrives - falls back to the box of ``size`` around the origin, the
    convention ``check_in_condition`` uses.
    """

    position = (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
    bounds = ((obj.min.x, obj.min.y, obj.min.z), (obj.max.x, obj.max.y, obj.max.z))
    if not all(high > low for low, high in zip(*bounds)):
        half = (obj.size.x / 2.0, obj.size.y / 2.0, obj.size.z / 2.0)
        bounds = (tuple(-h for h in half), half)

    rotation = quaternion_matrix(
        (
            obj.pose.orientation.x,
            obj.pose.orientation.y,
            obj.pose.orientation.z,
            obj.pose.orientation.w,
        )
    )
    low, high = bounds
    centre = [sum(rotation[row][col] * (low[col] + high[col]) / 2.0 for col in range(3)) for row in range(3)]
    half_extent = [sum(abs(rotation[row][col]) * (high[col] - low[col]) / 2.0 for col in range(3)) for row in range(3)]

    return (
        tuple(p + c - h for p, c, h in zip(position, centre, half_extent)),
        tuple(p + c + h for p, c, h in zip(position, centre, half_extent)),
    )


def check_resting_on(obj, surface_obj, margin: float = 0.05) -> bool:
    """Checks whether ``obj`` rests on top of ``surface_obj``.

    The definition of the geometric ``on`` edge label, copied from ``resting_on``
    in ssg_tools ``relations.py`` so that a fact and a label can never disagree -
    a fact overrides the label in ``build_relationships``: both boxes grow by
    ``margin`` and have to touch without separating on any axis, ``obj`` has to
    be above ``surface_obj`` within ``margin``, and a pair that is above each
    other (two boxes thinner than the margin at the same level) is no evidence
    of resting and is dropped.
    """

    (obj_min, obj_max), (surface_min, surface_max) = object_bounds(obj), object_bounds(surface_obj)
    obj_low = [low - margin for low in obj_min]
    obj_high = [high + margin for high in obj_max]
    surface_low = [low - margin for low in surface_min]
    surface_high = [high + margin for high in surface_max]

    touching = max(
        max(o_low - s_high for o_low, s_high in zip(obj_low, surface_high)),
        max(s_low - o_high for s_low, o_high in zip(surface_low, obj_high)),
    ) <= 0.0
    above = obj_min[2] >= surface_max[2] - margin
    above_reverse = surface_min[2] >= obj_max[2] - margin

    return touching and above and not above_reverse


def stacking_on_facts(
    obj_poses: Sequence,
    exclude_surface_names: Sequence[str] = (),
    in_container_names: Sequence[str] = (),
    margin: float = 0.05,
    min_height_m: float = 0.03,
    fact_name: str = "on",
) -> List[Fact]:
    """``on`` facts for every pair of objects that rests on another object.

    The pose database holds every object of the world, so a relay lying on a
    multimeter is a pair like any other and gets the same predicate as
    ``on(obj, table)``: the dataset labels the two identically, and a fact
    overrides the geometric label (see ``build_relationships`` in ssg_tools
    ``disc_run_adapter.py``). Surfaces that already have a fact of their own
    (the planning-scene tables) and objects that are ``in`` a container keep the
    fact they have, and an object too flat to stand on is never a surface.
    """

    facts = []
    for surface_obj in obj_poses:
        if surface_obj.size.z < min_height_m:
            continue
        surface_name = surface_obj.class_id + "_" + str(surface_obj.instance_id)
        if surface_name in exclude_surface_names:
            continue
        for obj in obj_poses:
            obj_name = obj.class_id + "_" + str(obj.instance_id)
            if obj_name == surface_name or obj_name in in_container_names:
                continue
            if check_resting_on(obj, surface_obj, margin):
                new_fact = Fact(name=fact_name, values=[obj_name, surface_name])
                if new_fact not in facts:
                    facts.append(new_fact)

    return facts
