import unittest

from symbolic_fact_generation.on_fact_generator import (
    check_on_condition,
    check_in_condition,
    is_support_surface,
    check_resting_on,
    object_bounds,
    stacking_on_facts,
)
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

    def resting_relay(self, x=19.43, y=13.93, z=0.803):
        # a relay whose bottom is level with the top of multimeter_1 below
        return self.ObjectPose('relay', 1, Pose(position=Point(x, y, z),
                                                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                               Vector3(0.0575, 0.0451, 0.104))

    def multimeter(self, x=19.43, y=13.93, z=0.73):
        return self.ObjectPose('multimeter', 1, Pose(position=Point(x, y, z),
                                                      orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                               Vector3(0.18, 0.087, 0.042))

    def test_relay_resting_on_multimeter(self):
        self.assertTrue(check_resting_on(self.resting_relay(), self.multimeter()),
                        msg="relay_1 rests on multimeter_1")

    def test_multimeter_not_resting_on_relay(self):
        # the test is asymmetric: nothing rests on what it rests on
        self.assertFalse(check_resting_on(self.multimeter(), self.resting_relay()),
                         msg="multimeter_1 does not rest on relay_1")

    def test_relay_beside_multimeter(self):
        self.assertFalse(check_resting_on(self.resting_relay(x=19.75), self.multimeter()),
                         msg="a relay next to the multimeter is not on it")

    def test_relay_below_multimeter(self):
        relay = self.resting_relay(z=0.6)
        self.assertFalse(check_resting_on(relay, self.multimeter()),
                         msg="an object under the multimeter is not on it")

    def test_bounds_prefer_message_bounds(self):
        # the ground-truth feeder stores the bounds of the box in the object
        # frame around the pose origin, so a class whose origin sits at its base
        # (center_offset) keeps the box above the origin
        obj = self.ObjectPose('coaster', 1, Pose(position=Point(19.43, 13.93, 0.751),
                                                 orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                              Vector3(0.04, 0.04, 0.044),
                              min=Vector3(-0.02, -0.02, 0.0), max=Vector3(0.02, 0.02, 0.044))
        self.assertEqual(object_bounds(obj), ((19.41, 13.91, 0.751), (19.45, 13.95, 0.795)))

    def test_bounds_fall_back_to_size(self):
        # a client that sends no bounds at all (the pose_selector stores every
        # message as it arrives) leaves them at zero
        obj = self.ObjectPose('sugar box', 1, Pose(position=Point(5.0, 6.0, 0.8),
                                                   orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                              Vector3(0.2, 0.1, 0.12), min=Vector3(), max=Vector3())
        low, high = object_bounds(obj)
        self.assertEqual([round(value, 6) for value in low], [4.9, 5.95, 0.74])
        self.assertEqual([round(value, 6) for value in high], [5.1, 6.05, 0.86])

    def test_bounds_rotate_with_the_pose(self):
        # a box turned 90 degrees around z is as deep as it is wide
        obj = self.ObjectPose('sugar box', 1, Pose(position=Point(5.0, 6.0, 0.8),
                                                   orientation=Quaternion(0.0, 0.0, 0.7071067811865476, 0.7071067811865476)),
                              Vector3(0.2, 0.1, 0.12))
        low, high = object_bounds(obj)
        self.assertEqual([round(value, 6) for value in low], [4.95, 5.9, 0.74])
        self.assertEqual([round(value, 6) for value in high], [5.05, 6.1, 0.86])

    def test_bounds_of_a_recorded_object(self):
        # multimeter_1 of run_20260928_151445, turned 103 degrees: the recorded
        # ground truth box of the dataset has to come out of the message
        obj = self.ObjectPose('multimeter', 1, Pose(position=Point(18.49, 15.52, 0.739),
                                                    orientation=Quaternion(0.0, 0.0, -0.7824, 0.6228)),
                              Vector3(0.1795, 0.0875, 0.0421))
        low, high = object_bounds(obj)
        # the recorded box of that run; the dataset keeps the box of the model
        # in Gazebo, which is a millimetre off the size of sim_object_extents
        for value, expected in zip(low, (18.427248, 15.422728, 0.718)):
            self.assertAlmostEqual(value, expected, delta=0.001)
        for value, expected in zip(high, (18.552752, 15.617272, 0.76006)):
            self.assertAlmostEqual(value, expected, delta=0.001)

    def test_resting_drops_a_mutual_pair(self):
        # two thin boxes at the same level are above each other within the
        # margin, which is no evidence of resting
        sheet_1 = self.ObjectPose('sheet', 1, Pose(position=Point(19.43, 13.93, 0.73),
                                                   orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                                  Vector3(0.2, 0.2, 0.004))
        sheet_2 = self.ObjectPose('sheet', 2, Pose(position=Point(19.43, 13.93, 0.732),
                                                   orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                                  Vector3(0.2, 0.2, 0.004))
        self.assertFalse(check_resting_on(sheet_1, sheet_2))
        self.assertFalse(check_resting_on(sheet_2, sheet_1))

    def recorded(self, class_id, instance_id, x, y, z, q, size):
        """An object as the ground-truth feeder sends it, from a recorded run."""

        return self.ObjectPose(class_id, instance_id, Pose(position=Point(x, y, z),
                                                           orientation=Quaternion(*q)),
                               Vector3(*size))

    def relay_on_soup(self):
        # the last snapshot of run_20260928_151445: the relay was placed on the
        # soup, 0.0049 m above its box
        relay = self.recorded('relay', 1, 19.5933, 13.8886, 0.8722,
                              (0.0164, 0.0051, 0.0816, 0.9965), (0.0575, 0.0451, 0.1044))
        soup = self.recorded('soup', 1, 19.5822, 13.8944, 0.7707,
                             (-0.6973, 0.0156, 0.0178, 0.7164), (0.0677, 0.1019, 0.0677))
        return relay, soup

    def test_relay_of_a_recorded_run_on_soup(self):
        relay, soup = self.relay_on_soup()
        self.assertTrue(check_resting_on(relay, soup), msg="relay_1 rests on soup_1")
        self.assertFalse(check_resting_on(soup, relay), msg="soup_1 does not rest on relay_1")
        self.assertEqual([str(fact) for fact in stacking_on_facts([relay, soup])],
                         ["on(relay_1, soup_1)"])

    def test_small_footprint_overlap_counts(self):
        # mustard_1 of run_20260928_151445 next to multimeter_1: the boxes
        # overlap by 0.011 m in y, which the grown boxes of resting_on still
        # touch, so the ground-truth edge label says on as well
        mustard = self.recorded('mustard', 1, 18.5347, 15.6552, 0.8154,
                                (-0.6854, -0.1750, 0.1750, 0.6848), (0.0960, 0.1913, 0.0582))
        multimeter = self.recorded('multimeter', 1, 18.49, 15.52, 0.739,
                                   (0.0, 0.0, -0.7824, 0.6228), (0.1795, 0.0875, 0.0421))
        self.assertTrue(check_resting_on(mustard, multimeter),
                        msg="the grown boxes touch, as they do for the label")

    def test_stacking_on_facts_of_a_recorded_run(self):
        # every object of the last snapshot of run_20260928_151445 that is not
        # a table and not in the klt: the pairs are the ones the geometric edge
        # labels of that run carry, so the facts agree with them
        objects = [
            self.recorded('power_drill_with_grip', 1, 20.9698, 14.0618, 0.8268,
                          (-0.5118, -0.4961, 0.4891, 0.5027), (0.1800, 0.2205, 0.0812)),
            self.recorded('multimeter', 1, 18.49, 15.52, 0.739,
                          (0.0, 0.0, -0.7824, 0.6228), (0.1795, 0.0875, 0.0421)),
            self.recorded('screwdriver', 1, 20.9106, 14.0056, 0.7352,
                          (0.0, 0.0, -0.6193, 0.7852), (0.2450, 0.0344, 0.0344)),
            self.recorded('mustard', 1, 18.5347, 15.6552, 0.8154,
                          (-0.6854, -0.1750, 0.1750, 0.6848), (0.0960, 0.1913, 0.0582)),
            self.recorded('hot_glue_gun', 1, 18.4650, 15.4112, 0.8070,
                          (-0.5473, 0.4404, -0.4529, 0.5491), (0.1644, 0.1779, 0.0524)),
            *self.relay_on_soup(),
        ]
        self.assertEqual(sorted(str(fact) for fact in stacking_on_facts(objects)), [
            "on(hot_glue_gun_1, multimeter_1)",
            "on(mustard_1, multimeter_1)",
            "on(power_drill_with_grip_1, screwdriver_1)",
            "on(relay_1, soup_1)",
        ])

    def test_rule_matches_the_label_rule(self):
        # ssg_tools is not in this workspace, so resting_on is repeated here:
        # a fact overrides the geometric label, so the two must not drift apart
        def resting_on(lower, upper, other_lower, other_upper, margin):
            lower, upper, other_lower, other_upper = (list(box) for box in
                                                      (lower, upper, other_lower, other_upper))
            grown_lower = [value - margin for value in lower]
            grown_upper = [value + margin for value in upper]
            grown_other_lower = [value - margin for value in other_lower]
            grown_other_upper = [value + margin for value in other_upper]
            separation = max(
                max(grown_lower[i] - grown_other_upper[i] for i in range(3)),
                max(grown_other_lower[i] - grown_upper[i] for i in range(3)),
            )
            touching = separation <= 0.0
            above = lower[2] >= other_upper[2] - margin
            above_reverse = other_lower[2] >= upper[2] - margin
            return touching and above and not above_reverse

        for x in (0.0, 0.02, 0.05, 0.2, 0.9):
            for z in (0.0, 0.02, 0.05, 0.1):
                for turn in ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)):
                    surface = self.ObjectPose('soup', 1, Pose(position=Point(19.43, 13.93, 0.77),
                                                               orientation=Quaternion(*turn)),
                                              Vector3(0.0677, 0.1019, 0.0677))
                    obj = self.ObjectPose('relay', 1, Pose(position=Point(19.43 + x, 13.93, 0.77 + z),
                                                           orientation=Quaternion(*turn)),
                                          Vector3(0.0575, 0.0451, 0.1044))
                    (obj_low, obj_high) = object_bounds(obj)
                    (surface_low, surface_high) = object_bounds(surface)
                    self.assertEqual(
                        check_resting_on(obj, surface),
                        resting_on(obj_low, obj_high, surface_low, surface_high, 0.05),
                        msg=f"x={x} z={z} turn={turn}",
                    )

    def test_stacking_on_facts_relay_on_multimeter(self):
        table = self.ObjectPose('table', 1, Pose(position=Point(19.43, 13.93, 0.3575),
                                                  orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                                Vector3(1.6, 0.8, 0.715))
        facts = stacking_on_facts([self.resting_relay(), self.multimeter(), table],
                                  exclude_surface_names={'table_1'})
        self.assertEqual([str(fact) for fact in facts], ["on(relay_1, multimeter_1)"])

    def test_stacking_on_facts_skips_container_contents(self):
        # klt_1 raised so that multimeter_1 does rest on it, and the in fact wins
        klt = self.ObjectPose('klt', 1, Pose(position=Point(19.43, 13.93, 0.6355),
                                             orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                              Vector3(0.297, 0.197, 0.147))
        self.assertTrue(check_resting_on(self.multimeter(), klt),
                        msg="fixture: multimeter_1 rests on klt_1")
        facts = stacking_on_facts([self.multimeter(), klt], in_container_names={'multimeter_1'})
        self.assertEqual(facts, [])

    def test_stacking_on_facts_skips_flat_surfaces(self):
        # a coaster the relay really does rest on, but too flat to be a surface
        coaster = self.ObjectPose('coaster', 1, Pose(position=Point(19.43, 13.93, 0.7485),
                                                     orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                                  Vector3(0.12, 0.12, 0.005))
        self.assertTrue(check_resting_on(self.resting_relay(), coaster),
                        msg="fixture: the relay does rest on the coaster")
        facts = stacking_on_facts([self.resting_relay(), coaster])
        self.assertEqual(facts, [])

    def test_stacking_on_facts_no_self_or_duplicate(self):
        multimeter = self.multimeter()
        facts = stacking_on_facts([multimeter])
        self.assertEqual(facts, [])


PKG = 'symbolic_fact_generation'
NAME = 'test_symbolic_fact_generation_on_generator'

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestOnGenerator)
