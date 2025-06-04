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
import sys

import rospy
import rospkg
import rosgraph

from pose_selector.srv import ClassQuery, ClassQueryRequest
from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.lib import split_object_class_from_id

from rospy_message_converter import message_converter
from object_pose_msgs.msg import ObjectPose

from semap_lite import SceneGraph, Entity
from semap_lite.intersections import spatial_intersection_multi, classify_intersection, IntersectionType
from semap_lite.viewer import SceneViewer
import trimesh


class SemapGenerator(GeneratorInterface):

    def __init__(self, objects_of_interest: List[str] = [],
                 container_objects: List[str] = [],
                 query_srv_str: str = '/pick_pose_selector_node/pose_selector_class_query',
                 planning_scene_param: str = '/mobipick/pick_object_node/planning_scene_boxes',
                 object_margin: float = 0.01) -> None:
        try:
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            rospy.wait_for_service(query_srv_str, timeout=10.0)
            self._pose_selector_query_srv = rospy.ServiceProxy(query_srv_str, ClassQuery)

            self._objects_of_interest = []
            for object_of_interest in objects_of_interest:
                obj_of_interest_class, _ = split_object_class_from_id(object_of_interest)
                if obj_of_interest_class not in self._objects_of_interest:
                    self._objects_of_interest.append(obj_of_interest_class)

            self._container_objects = container_objects

            package_path = rospkg.RosPack().get_path('symbolic_fact_generation')

            self._planning_scene_object_poses = []

            # if planning scene config file on parameter server
            if rospy.has_param(planning_scene_param):
                print(f"Using {planning_scene_param} parameter.")
                planning_scene_boxes = rospy.get_param(planning_scene_param)
                id_counter = {}
                for box in planning_scene_boxes:
                    class_id, instance_id = split_object_class_from_id(box['scene_name'])
                    if instance_id is None:
                        # create id for different class types starting from 1
                        id_counter[class_id] = id_counter.get(class_id, 1)
                        instance_id = id_counter[class_id]
                        id_counter[class_id] += 1
                    pose = {'class_id': class_id,
                            'instance_id': instance_id,
                            'pose':
                                {'position':
                                    {
                                        'x': box['box_position_x'],
                                        'y': box['box_position_y'],
                                        'z': box['box_position_z']
                                    },
                                 'orientation':
                                    {
                                        'x': box['box_orientation_x'],
                                        'y': box['box_orientation_y'],
                                        'z': box['box_orientation_z'],
                                        'w': box['box_orientation_w']
                                    }
                                 },
                            'size':
                                {
                                    'x': box['box_x_dimension'],
                                    'y': box['box_y_dimension'],
                                    'z': box['box_z_dimension']
                                },
                            'min':
                                {
                                    'x': -(box['box_x_dimension'] / 2.0),
                                    'y': -(box['box_y_dimension'] / 2.0),
                                    'z': -(box['box_z_dimension'] / 2.0)
                                },
                            'max':
                                {
                                    'x': (box['box_x_dimension'] / 2.0),
                                    'y': (box['box_y_dimension'] / 2.0),
                                    'z': (box['box_z_dimension'] / 2.0)
                                }
                            }
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))
            else:
                # use default config file
                print("Using symbolic_fact_generation/config/tables_poses.yaml")
                table_poses_yaml = package_path + "/config/table_poses.yaml"
                yamlfile = open(table_poses_yaml, 'r')
                yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)

                for pose in yaml_content["poses"]:
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))
                    
            # Create semap scene graph
            self._obj_margin = object_margin
            self._scene_graph = SceneGraph(base_frame='map', default_margin=self._obj_margin)

            print(self._planning_scene_object_poses)

            mesh_path = rospkg.RosPack().get_path('pbr_objects') + "/meshes/"
            self._obj_meshes = {}
            for obj in self._objects_of_interest:
                self._obj_meshes[obj] = trimesh.load_mesh(mesh_path + obj + "/" + obj + "_simplified.stl", force='mesh')

        except FileNotFoundError:
            print("[WARNING] No planning scene parameter is set and table_poses.yaml file is not found! Only objects on other objects facts can be generated!")
        except rospy.ROSInitException:
            print("ROS master was shutdown!")
            sys.exit(1)
        except rospy.ROSException:
            print(f"Timeout while waiting for pose_selector service: {query_srv_str}!")
            sys.exit(1)

    
    def create_entity(self, obj_pose: ObjectPose) -> Entity:
        """
        Creates a Semap entity from an ObjectPose message.

        Args:
            obj_pose (ObjectPose): ObjectPose message

        Returns:
            Entity: Semap entity
        """
        pose_angles = trimesh.transformations.euler_from_quaternion([obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z], axes='sxyz')
        tf_matrix = trimesh.transformations.compose_matrix(angles=pose_angles, translate=(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z))
        box = trimesh.creation.box(extents=(obj_pose.size.x, obj_pose.size.y, obj_pose.size.z), transform=tf_matrix)
        box.update_faces(box.unique_faces())
        trimesh.repair.fix_winding(box)

        return Entity(box, margin=self._obj_margin, copy=False)
    
    def copy_and_transform_mesh(self, obj: ObjectPose) -> trimesh.Trimesh:
        """
        Retrieve and copy a mesh and transform it to the pose of the given object.

        Args:
            obj (ObjectPose): ObjectPose message

        Returns:
            trimesh.Trimesh: Transformed mesh
        """
        obj_mesh = self._obj_meshes[obj.class_id].copy()
        pose_angles = trimesh.transformations.euler_from_quaternion([obj.pose.orientation.w, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z], axes='sxyz')
        tf_matrix = trimesh.transformations.compose_matrix(angles=pose_angles, translate=(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z))
        obj_mesh.apply_transform(tf_matrix)

        return obj_mesh
    
    def add_planning_scene_to_scene_graph(self) -> None:
        """
        Iterates over all ObjectPose messages in the planning scene and adds them to the scene graph.
        """
        for pose in self._planning_scene_object_poses:
            self._scene_graph.add_node(pose.class_id + "_" + str(pose.instance_id), entity=self.create_entity(pose))

    def generate_semap_facts(self, scene_graph: SceneGraph) -> List[Fact]:
        """
        Generates semantic facts from the given scene graph.

        Iterates over all edges in the scene graph and generates a fact for each edge.
        The type of the edge is used to determine the type of the fact.
        If the edge is a wall, the fact is skipped.
        The fact is added to the list of facts if it is not already there.
        The list of facts is returned.

        Args:
            scene_graph (SceneGraph): Scene graph

        Returns:
            List[Fact]: List of semantic facts
        """
        semap_facts = []
        for k, v, t in scene_graph.edges.data('type'):
            new_fact = None

            if "wall" in k or "wall" in v:
                continue
            if t == IntersectionType.Crosses:
                new_fact = Fact(name="crosses", values=[k, v])
            elif t == IntersectionType.Within:
                new_fact = Fact(name="within", values=[k, v])
            elif t == IntersectionType.PartialWithin:
                new_fact = Fact(name="partial_within", values=[k, v])
            elif t == IntersectionType.Contains:
                new_fact = Fact(name="contains", values=[k, v])
            elif t == IntersectionType.PartialContains:
                new_fact = Fact(name="partial_contains", values=[k, v])
            elif t == IntersectionType.Touches:
                new_fact = Fact(name="touches", values=[k, v])

            # add new fact to list if not already there
            if new_fact is not None and new_fact not in semap_facts:
                semap_facts.append(new_fact)
        return semap_facts

    def generate_facts(self) -> List[Fact]:
        # query pose_selector for all object classes
        """
        Generates on and in facts from the given scene graph.

        Queries the pose selector for all object classes,
        adds all objects to the scene graph,
        classifies relations between objects,
        generates semantic facts from the classified relations,
        and creates on and in facts from the semantic facts.

        Returns:
            List[Fact]: List of all generated facts
        """
       
        obj_poses = []

        for obj in self._objects_of_interest:
            query_result = self._pose_selector_query_srv(ClassQueryRequest(class_id=obj))
            obj_poses.extend(query_result.poses)
            
        # clear scene graph
        self._scene_graph.clear()
        # add tables and walls
        self.add_planning_scene_to_scene_graph()

        # add all objects stored in the pose selector
        for obj in obj_poses:
            obj_entity = self.copy_and_transform_mesh(obj)
            obj_name = obj.class_id + "_" + str(obj.instance_id)
            
            self._scene_graph.add_node(obj_name, entity=obj_entity)

        # classify relations
        mat = spatial_intersection_multi(self._scene_graph)

        names = classify_intersection(mat)
        semap_facts = self.generate_semap_facts(names)

        # visualize scene
        viewer = SceneViewer()
        viewer.add_scene_graph(self._scene_graph)
        viewer.show()

        facts = []

        # create on and in facts from semap facts
        for fact in semap_facts:
            new_fact = None
            if fact.name in ["crosses", "partial_within", "within"]:
                for container in self._container_objects:
                    if container in fact.values[0]:
                        new_fact = Fact(name="in", values=[fact.values[1], fact.values[0]])
                    elif container in fact.values[1]:
                        new_fact = Fact(name="in", values=[fact.values[0], fact.values[1]])
            elif fact.name == "touches":
                if "table" in fact.values[1]:
                    new_fact = Fact(name="on", values=[fact.values[0], fact.values[1]])
                elif "table" in fact.values[0]:
                    new_fact = Fact(name="on", values=[fact.values[1], fact.values[0]])
            if new_fact is not None and new_fact not in facts:
                facts.append(new_fact)

        return facts

    def generate_facts_using_obj_bounding_box(self) -> List[Fact]:
        # query pose_selector for all object classes
        """
        Generates all facts from the given scene graph using object bounding boxes of the objects.

        Queries the pose selector for all object classes,
        adds all objects to the scene graph,
        classifies relations between objects using their bounding boxes,
        generates semantic facts from the classified relations,
        visualizes the scene, and
        returns the list of generated facts.

        Returns:
            List[Fact]: List of all generated facts
        """
        obj_poses = []

        for obj in self._objects_of_interest:
            query_result = self._pose_selector_query_srv(ClassQueryRequest(class_id=obj))
            obj_poses.extend(query_result.poses)
            
        # clear scene graph
        self._scene_graph.clear()
        # add tables and walls
        self.add_planning_scene_to_scene_graph()

        # add all objects stored in the pose selector
        for obj in obj_poses:
            obj_entity = self.create_entity(obj)
            obj_name = obj.class_id + "_" + str(obj.instance_id)
            
            self._scene_graph.add_node(obj_name, entity=obj_entity)

        # classify relations
        mat = spatial_intersection_multi(self._scene_graph)

        names = classify_intersection(mat)
        semap_facts = self.generate_semap_facts(names)

        # visualize scene
        viewer = SceneViewer()
        viewer.add_scene_graph(self._scene_graph)
        viewer.show()

        return semap_facts