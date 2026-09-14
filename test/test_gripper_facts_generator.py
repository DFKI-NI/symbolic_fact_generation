#!/usr/bin/env python3

import unittest
from unittest.mock import patch

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

    def make_generator(self, effort_threshold=0.0):
        return GripperHasObjectGenerator(effort_threshold=effort_threshold)

    def joint_state(self, position, effort=None):
        msg = JointState()
        msg.name = [self.JOINT_NAME]
        msg.position = [position]
        msg.effort = [] if effort is None else [effort]
        return msg

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

    def test_ignores_joint_states_without_configured_joint(self):
        generator = self.make_generator()
        generator.joint_states_cb(self.joint_state(0.5))
        msg = JointState(name=["some_other_joint"], position=[0.755])

        generator.joint_states_cb(msg)

        self.assertEqual(
            generator.generate_facts(),
            [Fact(name="gripper_has_object", values=[])],
        )


if __name__ == "__main__":
    unittest.main()
