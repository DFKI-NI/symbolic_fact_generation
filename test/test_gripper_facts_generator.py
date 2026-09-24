#!/usr/bin/env python3

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import rospy
from sensor_msgs.msg import JointState

from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.gripper_facts_generator import GripperHasObjectGenerator


class TestGripperHasObjectGenerator(unittest.TestCase):
    JOINT_NAME = "mobipick/gripper_finger_joint"

    def setUp(self):
        subscriber_patcher = patch(
            "symbolic_fact_generation.gripper_facts_generator.rospy.Subscriber"
        )
        self.addCleanup(subscriber_patcher.stop)
        self.subscriber = subscriber_patcher.start()

    def make_generator(self, effort_threshold=0.0, effort_hold_time=0.0, **kwargs):
        return GripperHasObjectGenerator(
            effort_threshold=effort_threshold, effort_hold_time=effort_hold_time, **kwargs
        )

    def make_hardware_aware_generator(self):
        # The configuration used for both Gazebo and the real Robotiq, whose
        # driver reports zero effort and closes on air at about 0.675 rad.
        return self.make_generator(
            effort_threshold=0.1,
            effort_hold_time=0.5,
            hardware_closed_joint_position=0.675,
            hardware_closed_tolerance=0.03,
        )

    def joint_state(self, position, effort=None, stamp=None):
        msg = JointState()
        if stamp is not None:
            msg.header.stamp = rospy.Time(stamp)
        msg.name = [self.JOINT_NAME]
        msg.position = [position]
        msg.effort = [] if effort is None else [effort]
        return msg

    HAS_OBJECT = [Fact(name="gripper_has_object", values=[])]

    def test_subscribes_to_configured_joint_state_topic(self):
        generator = self.make_generator()

        self.subscriber.assert_called_once_with(
            "/mobipick/joint_states", JointState, generator.joint_states_cb
        )

    def test_does_not_publish_fact_before_a_joint_state(self):
        self.assertEqual(self.make_generator().generate_facts(), [])

    def test_does_not_publish_fact_when_gripper_is_completely_open(self):
        generator = self.make_generator()

        # The simulated controller's 0.140 m open command maps to about
        # 0.049 rad because its calibrated physical open gap is 0.149 m.
        generator.joint_states_cb(self.joint_state(0.049))

        self.assertEqual(generator.generate_facts(), [])

    def test_publishes_fact_when_gripper_stops_before_fully_closed(self):
        generator = self.make_generator()

        generator.joint_states_cb(self.joint_state(0.5))

        self.assertEqual(
            generator.generate_facts(),
            [Fact(name="gripper_has_object", values=[])],
        )

    def test_removes_fact_when_gripper_is_completely_closed(self):
        generator = self.make_generator()
        generator.joint_states_cb(self.joint_state(0.5))

        generator.joint_states_cb(self.joint_state(0.750))

        self.assertEqual(generator.generate_facts(), [])

    def test_can_require_effort_to_confirm_contact(self):
        generator = self.make_generator(effort_threshold=1.0)

        generator.joint_states_cb(self.joint_state(0.5, effort=0.9))
        self.assertEqual(generator.generate_facts(), [])

        generator.joint_states_cb(self.joint_state(0.5, effort=-1.0))
        self.assertEqual(
            generator.generate_facts(),
            [Fact(name="gripper_has_object", values=[])],
        )

    def test_missing_effort_does_not_confirm_contact(self):
        generator = self.make_generator(effort_threshold=1.0)

        generator.joint_states_cb(self.joint_state(0.5))

        self.assertEqual(generator.generate_facts(), [])

    def test_partially_open_resting_gripper_is_not_an_object(self):
        generator = self.make_generator(effort_threshold=0.1)

        generator.joint_states_cb(self.joint_state(0.112, effort=-0.008))

        self.assertEqual(generator.generate_facts(), [])

    def test_effort_hold_bridges_zero_effort_samples(self):
        # Gazebo reports zero applied force on steps where the bouncing
        # finger exceeds its velocity limit, interleaved with real samples.
        generator = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)

        generator.joint_states_cb(self.joint_state(0.52, effort=1.786, stamp=10.0))
        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

        generator.joint_states_cb(self.joint_state(0.52, effort=0.0, stamp=10.02))
        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

        generator.joint_states_cb(self.joint_state(0.52, effort=0.0, stamp=10.5))
        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

    def test_effort_hold_expires_without_new_contact(self):
        generator = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)
        generator.joint_states_cb(self.joint_state(0.52, effort=1.786, stamp=10.0))

        generator.joint_states_cb(self.joint_state(0.52, effort=0.0, stamp=10.51))

        self.assertEqual(generator.generate_facts(), [])

    def test_effort_hold_does_not_start_without_contact(self):
        generator = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)

        generator.joint_states_cb(self.joint_state(0.52, effort=0.0, stamp=10.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_effort_hold_does_not_override_position_checks(self):
        generator = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)
        generator.joint_states_cb(self.joint_state(0.52, effort=1.786, stamp=10.0))

        generator.joint_states_cb(self.joint_state(0.750, effort=0.0, stamp=10.1))

        self.assertEqual(generator.generate_facts(), [])

    def test_effort_hold_resets_when_time_jumps_backwards(self):
        generator = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)
        generator.joint_states_cb(self.joint_state(0.52, effort=1.786, stamp=10.0))

        generator.joint_states_cb(self.joint_state(0.52, effort=0.0, stamp=1.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_rejects_negative_effort_hold_time(self):
        with self.assertRaises(ValueError):
            self.make_generator(effort_hold_time=-0.1)

    def test_ignores_joint_states_without_configured_joint(self):
        generator = self.make_generator()
        generator.joint_states_cb(self.joint_state(0.5))
        msg = JointState(name=["some_other_joint"], position=[0.755])

        generator.joint_states_cb(msg)

        self.assertEqual(
            generator.generate_facts(),
            [Fact(name="gripper_has_object", values=[])],
        )

    def test_hardware_without_effort_publishes_fact_for_a_held_object(self):
        generator = self.make_hardware_aware_generator()

        generator.joint_states_cb(self.joint_state(0.521, effort=0.0))

        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

    def test_hardware_gripper_closed_on_air_is_not_an_object(self):
        generator = self.make_hardware_aware_generator()

        generator.joint_states_cb(self.joint_state(0.675, effort=0.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_hardware_gripper_open_is_not_an_object(self):
        generator = self.make_hardware_aware_generator()

        generator.joint_states_cb(self.joint_state(0.003, effort=0.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_reported_effort_keeps_the_simulation_checks(self):
        generator = self.make_hardware_aware_generator()
        generator.joint_states_cb(self.joint_state(0.112, effort=-0.008, stamp=10.0))

        # Closed on air in Gazebo: near the hardware closed position, but the
        # simulated closed position and effort confirmation now apply.
        generator.joint_states_cb(self.joint_state(0.675, effort=0.0, stamp=11.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_simulation_closing_on_air_before_any_effort_is_not_an_object(self):
        generator = self.make_hardware_aware_generator()

        generator.joint_states_cb(self.joint_state(0.755, effort=0.0))

        self.assertEqual(generator.generate_facts(), [])

    def test_simulation_grasp_sequence_matches_the_simulation_only_config(self):
        # Resting, closing (with the zero effort samples Gazebo emits above the
        # finger velocity limit), holding, then opening again.
        sequence = [
            (0.049, -0.008, 10.0),
            (0.30, 0.0, 10.1),
            (0.52, 1.786, 10.2),
            (0.52, 0.0, 10.25),
            (0.52, 1.5, 10.6),
            (0.52, 0.0, 11.2),
            (0.049, -0.008, 12.0),
        ]
        simulation = self.make_generator(effort_threshold=0.1, effort_hold_time=0.5)
        hardware_aware = self.make_hardware_aware_generator()

        for position, effort, stamp in sequence:
            msg = self.joint_state(position, effort=effort, stamp=stamp)
            simulation.joint_states_cb(msg)
            hardware_aware.joint_states_cb(msg)
            self.assertEqual(
                hardware_aware.generate_facts(), simulation.generate_facts(), stamp
            )

    def gripper_status(self, gOBJ):
        return SimpleNamespace(gOBJ=gOBJ)

    def make_status_generator(self):
        with patch.dict(sys.modules, {
            "robotiq_2f_gripper_control": MagicMock(),
            "robotiq_2f_gripper_control.msg": MagicMock(),
        }):
            return self.make_generator(gripper_status_topic="/mobipick/gripper_hw/input")

    @patch("symbolic_fact_generation.gripper_facts_generator.rospy.get_rostime")
    def test_gripper_status_overrides_a_partially_open_empty_gripper(self, now):
        now.return_value = rospy.Time(10.0)
        generator = self.make_status_generator()
        # The joint position alone looks like a held object after a release.
        generator.joint_states_cb(self.joint_state(0.094))

        generator.gripper_status_cb(self.gripper_status(3))

        self.assertEqual(generator.generate_facts(), [])

    @patch("symbolic_fact_generation.gripper_facts_generator.rospy.get_rostime")
    def test_gripper_status_reports_a_held_object(self, now):
        now.return_value = rospy.Time(10.0)
        generator = self.make_status_generator()

        generator.gripper_status_cb(self.gripper_status(2))
        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

        # Moving (gOBJ 0) keeps the last detection until the gripper stops.
        generator.gripper_status_cb(self.gripper_status(0))
        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

    @patch("symbolic_fact_generation.gripper_facts_generator.rospy.get_rostime")
    def test_stale_gripper_status_falls_back_to_joint_states(self, now):
        now.return_value = rospy.Time(10.0)
        generator = self.make_status_generator()
        generator.gripper_status_cb(self.gripper_status(3))
        generator.joint_states_cb(self.joint_state(0.5))

        now.return_value = rospy.Time(12.5)

        self.assertEqual(generator.generate_facts(), self.HAS_OBJECT)

    def test_rejects_negative_hardware_closed_tolerance(self):
        with self.assertRaises(ValueError):
            self.make_generator(hardware_closed_tolerance=-0.1)


if __name__ == "__main__":
    unittest.main()
