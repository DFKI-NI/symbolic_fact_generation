import unittest

from symbolic_fact_generation.on_fact_generator import check_on_condition, check_in_condition, is_support_surface
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion


class TestOnGenerator(unittest.TestCase):
    # Mockup class resembling the message returned by the pose_selector query
    class ObjectPose():
        def __init__(self, class_id, instance_id, pose, size, min=None, max=None):
            self.class_id = class_id
            self.instance_id = instance_id
            self.pose = pose
            self.size = size
            if min is None:
                self.min = Vector3(-(size.x / 2.0), -(size.y / 2.0), -(size.z / 2.0))
            else:
                self.min = min
            if max is None:
                self.max = Vector3(size.x / 2.0, size.y / 2.0, size.z / 2.0)
            else:
                self.max = max

    def test_ceiling_is_no_support_surface(self):
        # 2026-09-27: cic_tables_planning_scene.yaml has the lab ceiling as box 'roof'; klt_1 and relay_1 with wrong
        # poses at ~2.4 m became on(klt_1, roof_1) and the planner tried to drive to the ceiling
        roof = self.ObjectPose('roof', 1, Pose(position=Point(20.5, 14.96, 2.335), orientation=Quaternion(0, 0, 0, 1)),
                               Vector3(5.4, 2.9, 0.1))
        table = self.ObjectPose('table', 1, Pose(position=Point(21.265, 13.84, 0.3575),
                                                 orientation=Quaternion(0, 0, 0, 1)), Vector3(1.6, 0.8, 0.715))
        self.assertFalse(is_support_surface(roof))
        self.assertTrue(is_support_surface(table))
        self.assertFalse(is_support_surface(table, max_support_height=0.5))

    def test_klt_on_table(self):
        klt_pose = Pose(position=Point(21.123, 13.955, 0.802),
                        orientation=Quaternion(0.0, 0.032, -0.84, 0.538))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        table_pose = Pose(position=Point(21.265, 13.84, 0.3575),
                          orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        table_size = Vector3(1.6, 0.8, 0.715)
        table = self.ObjectPose('table', 1, table_pose, table_size)

        self.assertTrue(check_on_condition(klt, table), msg="klt_1 is on table_1")

    def test_klt_below_table(self):
        klt_pose = Pose(position=Point(21.127, 13.955, 0.1),
                        orientation=Quaternion(0.0, 0.03, -0.84, 0.538))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        table_pose = Pose(position=Point(21.265, 13.84, 0.3575),
                          orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        table_size = Vector3(1.6, 0.8, 0.715)
        table = self.ObjectPose('table', 1, table_pose, table_size)

        self.assertFalse(check_on_condition(klt, table), msg="klt_1 is not on table_1")

    def test_multiple_objects_on_multiple_tables(self):
        klt_pose = Pose(position=Point(21.127, 13.95, 0.798),
                        orientation=Quaternion(0.0, 0.019, -0.834, 0.551))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        relay_pose = Pose(position=Point(21.689, 13.887, 0.735),
                        orientation=Quaternion(-0.015, 0.003, 0.944, 0.327))
        relay_size = Vector3(0.0575, 0.0451, 0.104)
        relay = self.ObjectPose('relay', 1, relay_pose, relay_size)

        multimeter_1_pose = Pose(position=Point(19.43, 13.93, 0.73),
                        orientation=Quaternion(0.0, 0.0, 0.6, 0.8))
        multimeter_1_size = Vector3(0.18, 0.087, 0.042)
        multimeter_1 = self.ObjectPose('multimeter', 1, multimeter_1_pose, multimeter_1_size)

        multimeter_2_pose = Pose(position=Point(19.43, 13.93, 0.78),   # artificially shifted 5 cm upwards
                        orientation=Quaternion(0.0, 0.0, 0.6, 0.8))
        multimeter_2_size = multimeter_1_size
        multimeter_2 = self.ObjectPose('multimeter', 1, multimeter_2_pose, multimeter_2_size)

        power_drill_with_grip_pose = Pose(position=Point(19.78, 13.9, 0.836),
                        orientation=Quaternion(-0.4, -0.586, 0.587, 0.39))
        power_drill_with_grip_size = Vector3(0.18, 0.22, 0.08)
        power_drill_with_grip = self.ObjectPose('power_drill_with_grip', 1, power_drill_with_grip_pose, power_drill_with_grip_size)

        table_1_pose = Pose(position=Point(21.265, 13.84, 0.3575),
                          orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        table_size = Vector3(1.6, 0.8, 0.715)
        table_1 = self.ObjectPose('table', 1, table_1_pose, table_size)

        table_2_pose = Pose(position=Point(19.565, 13.76, 0.3575),
                          orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        table_2 = self.ObjectPose('table', 2, table_2_pose, table_size)

        # check actual objects on table 1 = True
        self.assertTrue(check_on_condition(klt, table_1), msg="klt_1 is on table_1")
        self.assertTrue(check_on_condition(relay, table_1), msg="relay_1 is on table_1")
        # check same objects for table 2 = False
        self.assertFalse(check_on_condition(klt, table_2), msg="klt_1 is not on table_2")
        self.assertFalse(check_on_condition(relay, table_2), msg="relay_1 is not on table_2")
        # check actual objects on table 2 = True
        self.assertTrue(check_on_condition(multimeter_1, table_2), msg="multimeter_1 is on table_2")
        self.assertTrue(check_on_condition(multimeter_2, table_2), msg="multimeter_2 is on table_2")
        self.assertTrue(check_on_condition(power_drill_with_grip, table_2), msg="power_drill_with_grip_1 is on table_2")
        # check same objects on table 1 = False
        self.assertFalse(check_on_condition(multimeter_1, table_1), msg="multimeter_1 is not on table_1")
        self.assertFalse(check_on_condition(power_drill_with_grip, table_1), msg="power_drill_with_grip_1 is not on table_1")

    def test_multimeter_in_klt(self):
        klt_pose = Pose(position=Point(21.123, 13.955, 0.802),
                        orientation=Quaternion(0.0, 0.032, -0.84, 0.538))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        multimeter_pose = Pose(position=Point(21.116, 13.933, 0.750),
                               orientation=Quaternion(-0.042, 0.013, 0.39, 0.919))
        multimeter_size = Vector3(0.179, 0.087, 0.042)
        multimeter = self.ObjectPose('multimeter', 1, multimeter_pose, multimeter_size)

        self.assertTrue(check_in_condition(multimeter, klt), msg="multimeter_1 is inside klt_1")


PKG = 'symbolic_fact_generation'
NAME = 'test_symbolic_fact_generation_on_generator'

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestOnGenerator)
