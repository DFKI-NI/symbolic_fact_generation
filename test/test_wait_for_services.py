'''#61: the on generator waits for the pose selector services with retries instead of dying after one 10 s wait.'''
import unittest

import rospy

from symbolic_fact_generation.on_fact_generator import wait_for_services


class FakeWait:
    def __init__(self, ready_after):
        self.ready_after = dict(ready_after)   # service -> calls that fail before it answers
        self.calls = []

    def __call__(self, name, timeout):
        self.calls.append((name, timeout))
        if self.ready_after.get(name, 0) > 0:
            self.ready_after[name] -= 1
            raise rospy.ROSException('timeout')


class TestWaitForServices(unittest.TestCase):
    def test_retries_until_the_service_appears(self):
        wait = FakeWait({'/query': 3})
        wait_for_services(['/query', '/get_all'], total_s=60.0, step_s=0.01, wait=wait, log=lambda _: None)
        self.assertEqual([n for n, _ in wait.calls], ['/query'] * 4 + ['/get_all'])

    def test_gives_up_after_the_total_naming_the_service(self):
        wait = FakeWait({'/query': 10 ** 6})
        with self.assertRaises(rospy.ROSException) as ctx:
            wait_for_services(['/query'], total_s=0.05, step_s=0.01, wait=wait, log=lambda _: None)
        self.assertIn('/query', str(ctx.exception))

    def test_slices_never_exceed_the_step(self):
        wait = FakeWait({'/query': 2})
        wait_for_services(['/query'], total_s=60.0, step_s=5.0, wait=wait, log=lambda _: None)
        self.assertTrue(all(0.1 <= t <= 5.0 for _, t in wait.calls))


if __name__ == '__main__':
    unittest.main()
