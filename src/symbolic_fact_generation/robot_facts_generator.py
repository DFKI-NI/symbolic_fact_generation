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

from typing import Dict, List, Sequence, Tuple

import rospy
import yaml
import tf
import math
import numpy
import os
import rosparam
import re

from sensor_msgs.msg import JointState

from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.fact import Fact


class HasArmPostureGenerator(GeneratorInterface):

    def __init__(self, fact_name: str = "robot_has_arm_posture",
                 namespace: str = "/mobipick/",
                 joint_states_topic: str = "/mobipick/joint_states",
                 arm_posture_param: str = "",
                 arm_tolerance: float = 0.01,
                 undefined_pose_name: str = "undefined"):
        rospy.Subscriber(joint_states_topic, JointState, self.jointStatesCB)

        self._namespace = namespace

        self._joint_states_msg = None
        self._joint_states_msg_received = False
        self._fact_name = fact_name
        self._current_arm_posture = undefined_pose_name
        self._arm_tolerance = arm_tolerance
        self._undefined_pose_name = undefined_pose_name

        try:
            # if arm poses param or file specified, use it
            if arm_posture_param:
                # check if param
                arm_posture_file_name = rospy.get_param(arm_posture_param, default=None)
                if arm_posture_file_name:
                    with open(arm_posture_file_name, "r") as arm_postures:
                        self._arm_poses = yaml.safe_load(arm_postures)
                # else extract from file directly
                else:
                    with open(arm_posture_param, "r") as arm_postures:
                        self._arm_poses = yaml.safe_load(arm_postures)
            else:
                # get arm pose from semantic robot description parameter
                self._arm_poses = self.extract_arm_poses_from_robot_description()

        except FileNotFoundError as e:
            self._arm_poses = {}
            print(f"Arm posture config file not found: {e}")

    def parse(self, pattern: str, string: str) -> str:
        """Return first occurrence of pattern in string, or raise AssertionError if pattern cannot be found."""
        match_result = re.search(pattern, string)
        assert match_result is not None, f"Error: Cannot parse '{string}' from '{pattern}'!"
        return match_result.group(1)

    def extract_arm_poses_from_robot_description(self, param_name: str = "robot_description_semantic") -> Dict[str, Dict[str, float]]:
        """Get joint values from semantic robot description parameter used for arm poses."""
        params = rosparam.list_params(self._namespace)

        if self._namespace + param_name in params:
            param = rosparam.get_param(self._namespace + param_name)
        else:
            # Otherwise search for param which ends with "_semantic", according to planning_context.launch.
            for param in params:
                if param.endswith("_semantic"):
                    break
            else:
                return {}

        # Collect all joint values from group states associated with group "arm".
        group_tokens: List[Tuple[str, str]] = re.findall(r'<group_state\s+([\'\"\w\s=]+)>(.*?)</group_state>',
                                                         param, re.DOTALL)
        return {
            self.parse(r'name=[\'\"](\w+)[\'\"]', token): {
                self.parse(r'name=[\'\"](.+?)[\'\"]', line): float(self.parse(r'value=[\'\"](.+?)[\'\"]', line))
                for line in content.split("\n") if line.strip().startswith("<joint ")
            }
            for token, content in group_tokens if "group='arm'" in token or 'group="arm"' in token
        }

    def generate_facts(self) -> List[Fact]:
        arm_posture_facts = []

        if self._joint_states_msg_received and self._arm_poses:
            self._joint_states_msg_received = False
            arm_posture = self._undefined_pose_name

            for posture in self._arm_poses:
                match = True
                for i, joint_position in enumerate(self._joint_states_msg.position):
                    if self._joint_states_msg.name[i] in list(self._arm_poses[posture]):
                        if abs(joint_position - self._arm_poses[posture][self._joint_states_msg.name[i]]) < self._arm_tolerance:
                            continue
                        else:
                            match = False
                            break
                if match:
                    arm_posture = posture
                    break

            arm_posture_facts.append(Fact(name=self._fact_name, values=[arm_posture]))
            self._current_arm_posture = arm_posture
        else:
            arm_posture_facts.append(Fact(name=self._fact_name, values=[self._current_arm_posture]))

        return arm_posture_facts

    def jointStatesCB(self, msg):
        self._joint_states_msg = msg
        self._joint_states_msg_received = True


class RobotAtGenerator(GeneratorInterface):

    def __init__(self, fact_name: str = "robot_at",
                 global_frame: str = "/map",
                 robot_frame: str = "/mobipick/base_link",
                 waypoint_param: str = "waypoints_yaml",
                 waypoint_namespace: str = "/mobipick/tables_demo_planning",
                 at_threshold: float = 0.2,
                 rot_threshold: float = 0.015,
                 undefined_pose_name: str = "undefined"):
        self._robot_at = undefined_pose_name

        self._fact_name = fact_name
        self._global_frame = global_frame
        self._robot_frame = robot_frame
        self._at_threshold = at_threshold
        self._rot_treshold = rot_threshold
        self._undefined_pose_name = undefined_pose_name

        self._tf_listener = tf.TransformListener()

        # check if param
        waypoints_file_name = rospy.get_param(waypoint_param, default=None) if waypoint_param else None
        if waypoints_file_name and os.path.isfile(waypoints_file_name):
            self._waypoints = self.load_waypoints(waypoints_file_name)
        # else extract from file directly if possible
        elif os.path.isfile(waypoint_param):
            self._waypoints = self.load_waypoints(waypoint_param)
        # else load from rosparam server
        else:
            self._waypoints = self.load_rosparams(waypoint_namespace)

    def generate_facts(self) -> List[Fact]:
        robot_at_facts = []
        if self._waypoints is not None:
            robot_at = self._undefined_pose_name

            try:
                self._tf_listener.waitForTransform(self._global_frame, self._robot_frame, rospy.Time(), rospy.Duration(5.0))
                now = rospy.Time.now()
                self._tf_listener.waitForTransform(self._global_frame, self._robot_frame, now, rospy.Duration(5.0))
                trans, rot = self._tf_listener.lookupTransform(self._global_frame, self._robot_frame, now)
            # tf.Exception is the base class: waitForTransform raises it directly on a late transform
            except tf.Exception as e:
                rospy.logwarn(f"Failed to get robot pose: {e}")
                robot_at_facts.append(Fact(name=self._fact_name, values=[self._robot_at]))
                return robot_at_facts
            for waypoint in self._waypoints:
                if self.distance_to_waypoint(waypoint, trans) < self._at_threshold:
                    if self.has_correct_heading(waypoint, rot):
                        robot_at = waypoint
            robot_at_facts.append(Fact(name=self._fact_name, values=[robot_at]))
            self._robot_at = robot_at
        else:
            robot_at_facts.append(Fact(name=self._fact_name, values=[self._robot_at]))

        return robot_at_facts

    def load_waypoints(self, filepath: str) -> Dict[str, List[float]]:
        """Load poses from config file."""
        poses: Dict[str, List[float]] = {}
        try:
            with open(filepath, 'r') as yaml_file:
                yaml_contents: Dict[str, Sequence[Sequence[float]]] = yaml.safe_load(yaml_file)["poses"]
                poses = {
                    key: [val for tokens in param for val in tokens] for key, param in yaml_contents.items()
                    if isinstance(param, Sequence)
                    and all(isinstance(tokens, Sequence) and all(isinstance(f, float) for f in tokens) for tokens in param)
                    and tuple(map(len, param)) == (3, 4)
                }
            if not poses:
                raise RuntimeError(f"No waypoints found in config file: {filepath}")
            return poses
        except FileNotFoundError as e:
            print(f"Robot waypoints config file not found: {e}")
            return poses

    def load_rosparams(self, namespace: str) -> Dict[str, List[float]]:
        """Load poses from rosparam server."""
        poses: Dict[str, List[float]] = {}
        for param_name in rosparam.list_params(namespace):
            param = rosparam.get_param(param_name)
            if (
                isinstance(param, Sequence)
                and all(isinstance(tokens, Sequence) and all(isinstance(f, float) for f in tokens) for tokens in param)
                and tuple(map(len, param)) == (3, 4)
            ):
                poses[param_name.rsplit('/', maxsplit=1)[-1]] = [val for tokens in param for val in tokens]
        if not poses:
            raise RuntimeError("No waypoints found on rosparam server.")
        return poses

    def distance_to_waypoint(self, waypoint, robot_position):

        x_1 = self._waypoints[waypoint][0]
        y_1 = self._waypoints[waypoint][1]
        x_2 = robot_position[0]
        y_2 = robot_position[1]

        d = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
        return d

    def has_correct_heading(self, waypoint, robot_rotation):
        # calculate difference between two rotations as quaternion
        d = numpy.dot(self._waypoints[waypoint][3:], robot_rotation)

        return abs(abs(d) - 1.0) < self._rot_treshold
