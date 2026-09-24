#!/usr/bin/env python3

import os
import unittest
from importlib.machinery import SourceFileLoader
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import rospy

from symbolic_fact_generation.common.fact import Fact

NODE = os.path.join(os.path.dirname(__file__), '..', 'nodes', 'fact_publisher')
fact_publisher = SourceFileLoader('fact_publisher_node', NODE).load_module()


class FailingOnce:
    def __init__(self, facts):
        self.facts = facts
        self.fail = False

    def generate_facts(self):
        if self.fail:
            raise rospy.ServiceException('unable to connect to service: [Errno 111] Connection refused')
        return list(self.facts)


class TestFactPublisher(unittest.TestCase):
    def make_publisher(self, generators):
        publisher = fact_publisher.FactPublisher.__new__(fact_publisher.FactPublisher)
        publisher._fact_classes = generators
        publisher.current_facts = []
        publisher._last_generated = {}
        publisher._facts_pub = MagicMock()
        publisher._changed_facts_pub = MagicMock()
        return publisher

    def test_failing_generator_keeps_its_facts_and_the_others_update(self):
        on = FailingOnce([Fact('on', ['tennis_ball_1', 'table_2'])])
        gripper = SimpleNamespace(generate_facts=lambda: [])
        publisher = self.make_publisher([('on', on), ('gripper_has_object', gripper)])
        publisher.update_and_publish_facts()

        on.fail = True
        gripper.generate_facts = lambda: [Fact('gripper_has_object', [])]
        with patch.object(rospy, 'logwarn_throttle') as warn:
            publisher.update_and_publish_facts()

        warn.assert_called_once()
        self.assertIn(Fact('on', ['tennis_ball_1', 'table_2']), publisher.current_facts)
        self.assertIn(Fact('gripper_has_object', []), publisher.current_facts)

        on.fail = False
        on.facts = []
        publisher.update_and_publish_facts()
        self.assertEqual(publisher.current_facts, [Fact('gripper_has_object', [])])


if __name__ == '__main__':
    unittest.main()
