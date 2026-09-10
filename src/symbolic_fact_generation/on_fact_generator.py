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

from typing import List
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


class OnGenerator(GeneratorInterface):

    def __init__(
        self,
        fact_name: str = "on",
        objects_of_interest: List[str] = [],
        container_objects: List[str] = [],
        query_srv_str: str = "/pick_pose_selector_node/pose_selector_class_query",
        planning_scene_param: str = "/mobipick/pick_object_node/planning_scene_boxes",
        get_all_srv_str: str = None,
    ) -> None:
        try:
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            rospy.wait_for_service(query_srv_str, timeout=10.0)
            self._pose_selector_query_srv = rospy.ServiceProxy(query_srv_str, ClassQuery)
            self._pose_selector_get_all_srv = None
            if get_all_srv_str:
                rospy.wait_for_service(get_all_srv_str, timeout=10.0)
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

        except FileNotFoundError:
            print(
                "[WARNING] No planning scene parameter is set and table_poses.yaml file is not found! Only objects on other objects facts can be generated!"
            )
        except rospy.ROSInitException:
            print("ROS master was shutdown!")
            sys.exit(1)
        except rospy.ROSException:
            print(f"Timeout while waiting for pose_selector service: {query_srv_str}!")
            sys.exit(1)

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

        return on_facts


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
