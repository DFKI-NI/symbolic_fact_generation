#!/usr/bin/python3

# BSD 3-Clause License

# Copyright (c) 2026, DFKI Niedersachsen
# All rights reserved.

"""Symbolic facts derived from the gripper joint state."""

import math
from typing import List

import rospy
from sensor_msgs.msg import JointState

from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.generator_interface import GeneratorInterface


class GripperHasObjectGenerator(GeneratorInterface):
    """Generate a predicate when an attempted closure is blocked by an object.

    The Robotiq driver reports the actuated finger joint at its configured
    closed position when the fingers close without encountering an object.  A
    joint position between the configured open and closed tolerances therefore
    indicates that something probably stopped the fingers.  Consumers should
    evaluate this fact after commanding the gripper to close.

    ``effort_threshold`` optionally makes a non-closed position insufficient
    on its own: the absolute joint effort must also reach the threshold.  A
    value of zero disables effort confirmation, which is useful on hardware
    whose JointState publisher does not provide measured effort.
    """

    def __init__(self, fact_name: str = "gripper_has_object",
                 joint_states_topic: str = "/mobipick/joint_states",
                 joint_name: str = "mobipick/gripper_finger_joint",
                 closed_joint_position: float = 0.755,
                 closed_tolerance: float = 0.01,
                 effort_threshold: float = 0.0,
                 open_joint_position: float = 0.0,
                 open_tolerance: float = 0.06):
        if closed_tolerance < 0.0:
            raise ValueError("closed_tolerance must be non-negative")
        if effort_threshold < 0.0:
            raise ValueError("effort_threshold must be non-negative")
        if open_tolerance < 0.0:
            raise ValueError("open_tolerance must be non-negative")

        self._fact_name = fact_name
        self._joint_name = joint_name
        self._closed_joint_position = closed_joint_position
        self._closed_tolerance = closed_tolerance
        self._effort_threshold = effort_threshold
        self._open_joint_position = open_joint_position
        self._open_tolerance = open_tolerance
        self._has_object = False

        self._joint_states_subscriber = rospy.Subscriber(
            joint_states_topic, JointState, self.joint_states_cb
        )

    def generate_facts(self) -> List[Fact]:
        """Return the zero-argument predicate while an object is detected."""
        if self._has_object:
            return [Fact(name=self._fact_name, values=[])]
        return []

    def joint_states_cb(self, msg: JointState) -> None:
        """Update the detection state from this generator's configured joint."""
        try:
            joint_index = msg.name.index(self._joint_name)
            position = msg.position[joint_index]
        except (ValueError, IndexError):
            return

        if not math.isfinite(position):
            return

        not_completely_closed = (
            abs(position - self._closed_joint_position) > self._closed_tolerance
        )
        not_completely_open = (
            abs(position - self._open_joint_position) > self._open_tolerance
        )
        effort_confirms_contact = True

        if self._effort_threshold > 0.0:
            try:
                effort = msg.effort[joint_index]
            except IndexError:
                effort_confirms_contact = False
            else:
                effort_confirms_contact = (
                    math.isfinite(effort)
                    and abs(effort) >= self._effort_threshold
                )

        self._has_object = (
            not_completely_open
            and not_completely_closed
            and effort_confirms_contact
        )
