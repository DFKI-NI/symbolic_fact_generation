#!/usr/bin/env python3
"""RobotAtGenerator uses the latest robot transform and tolerates TF lag (#196). No ROS master needed:
python3 -m unittest test.test_robot_at_generator (in the Mobipick image, rospy and tf importable)."""
import unittest

import rospy
import tf

from symbolic_fact_generation.robot_facts_generator import RobotAtGenerator


class FakeListener:
    """tf.TransformListener stand-in: one transform at a fixed stamp, or none"""

    def __init__(self, stamp, trans, rot):
        self.stamp, self.trans, self.rot = stamp, trans, rot
        self.waited = False

    def getLatestCommonTime(self, global_frame, robot_frame):
        if self.stamp is None:
            raise tf.Exception('no transform yet')
        return self.stamp

    def lookupTransform(self, global_frame, robot_frame, time):
        assert time == self.stamp, 'must look up the latest transform, not "now"'
        return self.trans, self.rot

    def waitForTransform(self, *args):
        self.waited = True   # the old code blocked here for up to 5 s per tick


def make_generator(listener, now_s, waypoint=(19.5, 14.7, 0.0, 0.0, 0.0, 0.0, 1.0)):
    generator = RobotAtGenerator.__new__(RobotAtGenerator)   # no tf.TransformListener, no rosparam
    generator._robot_at = 'unknown_pose'
    generator._fact_name, generator._global_frame, generator._robot_frame = 'robot_at', '/map', '/mobipick/base_link'
    generator._at_threshold, generator._rot_treshold, generator._undefined_pose_name = 0.1, 0.004, 'unknown_pose'
    generator._max_tf_age_s = 5.0
    generator._tf_listener = listener
    generator._waypoints = {'base_table_2_pose': list(waypoint)}
    generator._now = lambda: rospy.Time(now_s)
    return generator


def robot_at(generator):
    facts = generator.generate_facts()
    return facts[0].values[0]


class TestRobotAtTfAge(unittest.TestCase):
    def test_uses_the_latest_transform_despite_a_small_lag(self):
        # 2026-09-28 18:47:02: a transform 93 ms behind "now" made the old lookup fail and keep a stale fact
        listener = FakeListener(rospy.Time(1000.0), (19.52, 14.68, 0.0), (0.0, 0.0, 0.0, 1.0))
        generator = make_generator(listener, 1000.093)
        self.assertEqual('base_table_2_pose', robot_at(generator))
        self.assertFalse(listener.waited)

    def test_a_transform_older_than_the_limit_keeps_the_last_fact(self):
        listener = FakeListener(rospy.Time(1000.0), (19.52, 14.68, 0.0), (0.0, 0.0, 0.0, 1.0))
        generator = make_generator(listener, 1004.0)
        self.assertEqual('base_table_2_pose', robot_at(generator))     # 4 s old: still used
        generator._now = lambda: rospy.Time(1066.0)                     # 66 s old (the demo's TF stall)
        self.assertEqual('base_table_2_pose', robot_at(generator))     # last value kept, the robot did not move
        generator._robot_at = 'unknown_pose'
        self.assertEqual('unknown_pose', robot_at(generator))          # and nothing is invented from stale data

    def test_no_transform_keeps_the_last_fact(self):
        generator = make_generator(FakeListener(None, None, None), 1000.0)
        self.assertEqual('unknown_pose', robot_at(generator))

    def test_off_the_waypoint_is_unknown(self):
        listener = FakeListener(rospy.Time(1000.0), (19.65, 14.68, 0.0), (0.0, 0.0, 0.0, 1.0))   # 15 cm along the table
        self.assertEqual('unknown_pose', robot_at(make_generator(listener, 1000.0)))


if __name__ == '__main__':
    unittest.main()
