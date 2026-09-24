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

    ``effort_hold_time`` keeps the effort confirmation alive for that many
    seconds after the last sample that reached the threshold.  Gazebo reports
    an applied joint force of exactly zero on every physics step in which the
    joint exceeds its URDF velocity limit, so a finger bouncing on a grasped
    object interleaves valid effort samples with zeros and the fact would
    flicker without this peak hold.  The hold is measured on the JointState
    header stamp (ROS time when the stamp is unset).

    ``hardware_closed_joint_position`` (disabled when negative) handles real
    hardware whose JointState publisher reports zero effort and whose fingers
    close further than the simulated ones.  While the joint has never reported
    a nonzero effort, the effort confirmation is skipped and this position,
    with ``hardware_closed_tolerance``, counts as closed as well.  Gazebo
    reports nonzero effort almost immediately and keeps the regular checks.

    ``gripper_status_topic`` (disabled when empty) names the Robotiq driver's
    register input topic.  While it delivers messages the gripper's own object
    detection decides: gOBJ 1 or 2 (stopped on an object) means an object, 3
    (reached the requested position) means none, and 0 (moving) keeps the
    previous value.  The joint position cannot tell a partially opened empty
    gripper from a held object without effort, which real hardware lacks.
    Without status messages for ``gripper_status_timeout`` seconds, the joint
    state checks apply again (the simulation has no such topic).
    """

    def __init__(self, fact_name: str = "gripper_has_object",
                 joint_states_topic: str = "/mobipick/joint_states",
                 joint_name: str = "mobipick/gripper_finger_joint",
                 closed_joint_position: float = 0.755,
                 closed_tolerance: float = 0.01,
                 effort_threshold: float = 0.0,
                 open_joint_position: float = 0.0,
                 open_tolerance: float = 0.06,
                 effort_hold_time: float = 0.0,
                 hardware_closed_joint_position: float = -1.0,
                 hardware_closed_tolerance: float = 0.03,
                 gripper_status_topic: str = "",
                 gripper_status_timeout: float = 2.0):
        if closed_tolerance < 0.0:
            raise ValueError("closed_tolerance must be non-negative")
        if effort_threshold < 0.0:
            raise ValueError("effort_threshold must be non-negative")
        if open_tolerance < 0.0:
            raise ValueError("open_tolerance must be non-negative")
        if effort_hold_time < 0.0:
            raise ValueError("effort_hold_time must be non-negative")
        if hardware_closed_tolerance < 0.0:
            raise ValueError("hardware_closed_tolerance must be non-negative")

        self._fact_name = fact_name
        self._joint_name = joint_name
        self._closed_joint_position = closed_joint_position
        self._closed_tolerance = closed_tolerance
        self._effort_threshold = effort_threshold
        self._open_joint_position = open_joint_position
        self._open_tolerance = open_tolerance
        self._effort_hold_time = rospy.Duration(effort_hold_time)
        self._last_effort_contact_time = None
        self._hardware_closed_joint_position = hardware_closed_joint_position
        self._hardware_closed_tolerance = hardware_closed_tolerance
        self._effort_reported = False
        self._has_object = False
        self._gripper_status_timeout = rospy.Duration(gripper_status_timeout)
        self._gripper_status_time = None
        self._gripper_status_has_object = False
        self._gripper_status_subscriber = None
        if gripper_status_topic:
            try:
                # Only needed on the real robot; the simulation may lack the package.
                from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
            except ImportError as e:
                rospy.logwarn(f"gripper_has_object: ignoring {gripper_status_topic}: {e}")
            else:
                self._gripper_status_subscriber = rospy.Subscriber(
                    gripper_status_topic, Robotiq2FGripper_robot_input, self.gripper_status_cb
                )

        self._joint_states_subscriber = rospy.Subscriber(
            joint_states_topic, JointState, self.joint_states_cb
        )

    def generate_facts(self) -> List[Fact]:
        """Return the zero-argument predicate while an object is detected."""
        if self._gripper_status_is_fresh():
            has_object = self._gripper_status_has_object
        else:
            has_object = self._has_object
        if has_object:
            return [Fact(name=self._fact_name, values=[])]
        return []

    def gripper_status_cb(self, msg) -> None:
        """Track the Robotiq driver's object detection (gOBJ)."""
        self._gripper_status_time = rospy.get_rostime()
        if msg.gOBJ in (1, 2):
            self._gripper_status_has_object = True
        elif msg.gOBJ == 3:
            self._gripper_status_has_object = False

    def _gripper_status_is_fresh(self) -> bool:
        if self._gripper_status_time is None:
            return False
        age = rospy.get_rostime() - self._gripper_status_time
        return rospy.Duration(0) <= age <= self._gripper_status_timeout

    def joint_states_cb(self, msg: JointState) -> None:
        """Update the detection state from this generator's configured joint."""
        try:
            joint_index = msg.name.index(self._joint_name)
            position = msg.position[joint_index]
        except (ValueError, IndexError):
            return

        if not math.isfinite(position):
            return

        try:
            effort_sample = msg.effort[joint_index]
        except IndexError:
            effort_sample = None
        if effort_sample is not None and math.isfinite(effort_sample) and effort_sample != 0.0:
            self._effort_reported = True
        hardware = self._hardware_closed_joint_position >= 0.0 and not self._effort_reported

        not_completely_closed = (
            abs(position - self._closed_joint_position) > self._closed_tolerance
        )
        if hardware:
            # Keep the simulated closed position too: Gazebo counts as hardware
            # until its first nonzero effort sample and must not report an
            # object when it closes on air before that.
            not_completely_closed = not_completely_closed and (
                abs(position - self._hardware_closed_joint_position)
                > self._hardware_closed_tolerance
            )
        not_completely_open = (
            abs(position - self._open_joint_position) > self._open_tolerance
        )
        effort_confirms_contact = True

        if self._effort_threshold > 0.0 and not hardware:
            try:
                effort = msg.effort[joint_index]
            except IndexError:
                effort_confirms_contact = False
            else:
                effort_confirms_contact = (
                    math.isfinite(effort)
                    and abs(effort) >= self._effort_threshold
                )
            effort_confirms_contact = self._hold_effort_contact(
                msg, effort_confirms_contact
            )

        self._has_object = (
            not_completely_open
            and not_completely_closed
            and effort_confirms_contact
        )

    def _hold_effort_contact(self, msg: JointState, sample_confirms: bool) -> bool:
        """Extend a confirming effort sample over the configured hold time."""
        if self._effort_hold_time == rospy.Duration(0):
            return sample_confirms

        now = msg.header.stamp
        if now == rospy.Time(0):
            try:
                now = rospy.get_rostime()
            except rospy.ROSException:
                return sample_confirms

        if sample_confirms:
            self._last_effort_contact_time = now
            return True
        if self._last_effort_contact_time is None:
            return False
        elapsed = now - self._last_effort_contact_time
        if elapsed < rospy.Duration(0):
            # Time jumped backwards (e.g. a simulation reset): forget the hold.
            self._last_effort_contact_time = None
            return False
        return elapsed <= self._effort_hold_time
