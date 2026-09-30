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

import sys
import rospy
import yaml

from typing import List, Tuple

from symbolic_fact_generation.msg import Facts as FactsMsg
from symbolic_fact_generation.msg import Fact as FactMsg
from symbolic_fact_generation.msg import ChangedFacts as ChangedFactsMsg
from symbolic_fact_generation.common.fact import Fact


def convert_facts_to_msg(facts: List[Fact]) -> FactsMsg:
    """Convert fact class object to Facts msg."""
    return FactsMsg([FactMsg(name=f.name, values=f.values) for f in facts])

def convert_changed_facts_to_msg(update_types: List, facts: List[Fact]) -> ChangedFactsMsg:
    """Convert fact class object and update_type array to ChangedFacts msg."""
    return ChangedFactsMsg(update_types, [FactMsg(name=f.name, values=f.values) for f in facts])



def split_object_class_from_id(obj: str) -> Tuple[str, int]:
    """
    Assume class name seperated by underscore from id.

    e.g. input:  relay_1
         output: relay, 1
    """
    if "_" not in obj:
        return obj, None

    try:
        obj_class, obj_id = obj.rsplit("_", 1)
        obj_id = int(obj_id)
        return obj_class, obj_id
    except Exception:
        return obj, None


def retarget_robot_names(value, robot_namespace="mobipick", map_frame="map"):
    """Point fact generator params written for robot 1 at another robot (#226, several Mobipicks in one sim).

    In strings (also inside lists): /mobipick/... and mobipick/... get the robot's namespace, /map its map frame and
    the global pose selectors (/pick_pose_selector_node/..., /place_pose_selector_node/...) move under its namespace.
    Robot 1 (mobipick, map) is returned unchanged.
    """
    ns = str(robot_namespace or "mobipick").strip("/")
    frame = str(map_frame or "map").strip("/")
    if ns == "mobipick" and frame == "map":
        return value
    if isinstance(value, list):
        return [retarget_robot_names(v, ns, frame) for v in value]
    if not isinstance(value, str):
        return value
    if value in ("/map", "map"):
        return ("/" if value.startswith("/") else "") + frame
    if value == "/mobipick" or value.startswith("/mobipick/"):
        return "/" + ns + value[len("/mobipick"):]
    if value.startswith("mobipick/"):
        return ns + value[len("mobipick"):]
    if ns != "mobipick" and value.startswith(("/pick_pose_selector_node/", "/place_pose_selector_node/")):
        return "/" + ns + value
    return value


def read_yaml_file(path):
    """Open and safely load a yaml file."""
    try:
        yamlfile = open(path, "r")
        return yaml.safe_load(yamlfile)
    except FileNotFoundError as file_exc:
        rospy.logerr(f"YAML File not found:\n {file_exc}")
        sys.exit(1)
    except yaml.YAMLError as yaml_exc:
        rospy.logerr(f"Error while loading YAML file:\n {yaml_exc}")
        sys.exit(1)
