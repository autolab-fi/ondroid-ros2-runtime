import unittest
from ondroid_ros2.line_follow import LineFollowProgress

class TimedFollowingTest(unittest.TestCase):
    def test_requires_duration_and_measured_movement(self):
        p=LineFollowProgress(55)
        for i in range(551):p.observe(i/10,.4+.014*i/10,-.185,.04)
        self.assertFalse(p.ready(54))
        self.assertTrue(p.ready(55))
        self.assertGreater(p.distance,.7)
    def test_stationary_camera_jitter_does_not_pass(self):
        p=LineFollowProgress(55)
        for i in range(560):p.observe(i/10,.4+(.003 if i%2 else -.003),-.185,.04)
        self.assertFalse(p.ready(56))
        self.assertEqual(p.distance,0)
    def test_short_movement_then_wait_does_not_pass(self):
        p=LineFollowProgress(55)
        for i in range(560):p.observe(i/10,.4+min(i/10,10)*.05,-.185,.04 if i<100 else 0)
        self.assertGreater(p.distance,.45)
        self.assertFalse(p.ready(56))
    def test_duration_stays_inside_active_limit(self):
        for v in (-1,0,56,float('nan'),float('inf')):
            with self.assertRaises(ValueError):LineFollowProgress(v)
if __name__=='__main__':unittest.main()
