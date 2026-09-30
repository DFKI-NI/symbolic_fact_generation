#!/usr/bin/env python3
"""#226: fact generator params retargeted to another robot; robot 1 unchanged."""
import os
import unittest

import yaml

from symbolic_fact_generation.common.lib import retarget_robot_names

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'facts_config.yaml')


class TestRetarget(unittest.TestCase):
    def test_robot_1_unchanged(self):
        params = ['/map', '/mobipick/base_link', 'mobipick/gripper_finger_joint', '/pick_pose_selector_node/x', 0.1]
        self.assertIs(retarget_robot_names(params), params)

    def test_robot_2(self):
        got = retarget_robot_names(['/map', '/mobipick/base_link', '/mobipick/', 'mobipick/gripper_finger_joint',
                                    '/pick_pose_selector_node/pose_selector_get_all', ['klt'], 'unknown', '', 0.1],
                                   'mobipick2', 'mobipick2/map')
        self.assertEqual(got, ['/mobipick2/map', '/mobipick2/base_link', '/mobipick2/', 'mobipick2/gripper_finger_joint',
                               '/mobipick2/pick_pose_selector_node/pose_selector_get_all', ['klt'], 'unknown', '', 0.1])

    def test_config_has_no_robot_1_name_left(self):
        cfg = yaml.safe_load(open(CONFIG))
        for fact in cfg['facts']:
            (name, spec), = fact.items()
            flat = str(retarget_robot_names(spec['params'], 'mobipick3', 'mobipick3/map'))
            self.assertNotIn("'/mobipick/", flat, name)
            self.assertNotIn("'mobipick/", flat, name)
            self.assertNotIn("'/map'", flat, name)
            self.assertNotIn("'/pick_pose_selector_node", flat, name)


if __name__ == '__main__':
    unittest.main()
