"""Practice observations must stay distinguishable from stale/saved results."""
from collections import deque
import threading
import unittest
from unittest.mock import patch

from ondroid_ros2.workspaces import Session


class TelemetryTest(unittest.TestCase):
    def session(self):
        s = Session.__new__(Session)
        s.lock = threading.RLock()
        s.id, s.cid, s.lesson = 'test', 20, 'ros_sandbox'
        s.state, s.error = 'ready', ''
        s.pose, s.received_at, s.observed_at, s.command_received_at = None, None, None, None
        s.sequence, s.created = 0, 100
        s.topics, s.terminals, s.trail = {}, {}, deque(maxlen=700)
        return s

    @patch('ondroid_ros2.workspaces.time.monotonic', return_value=100)
    def test_stationary_heartbeat_then_stale_and_ended(self, clock):
        s = self.session()
        s.observe(dict(type='pose', x=.4, y=-.185, yaw=0))
        s.observe(dict(type='pose', x=.4, y=-.185, yaw=0))
        self.assertEqual(s.view()['telemetry']['sequence'], 2)
        self.assertTrue(s.view()['telemetry']['fresh'])
        self.assertEqual(s.view()['topics']['robot/pose']['value'], s.pose)
        clock.return_value = 103
        self.assertFalse(s.view()['telemetry']['fresh'])
        self.assertEqual(s.view()['telemetry']['age_ms'], 3000)
        s.observe(dict(type='pose', x=.4, y=-.185, yaw=0))
        s.state = 'ended'
        self.assertFalse(s.view()['telemetry']['fresh'])

    @patch('ondroid_ros2.workspaces.time.monotonic', return_value=100)
    def test_commands_expire_and_reset_does_not_rewind_sequence(self, clock):
        s = self.session()
        s.observe(dict(type='pose', x=.4, y=-.185, yaw=0))
        s.observe(dict(type='command', linear=.03, angular=0))
        self.assertTrue(s.view()['telemetry']['command_active'])
        clock.return_value = 101
        self.assertFalse(s.view()['telemetry']['command_active'])
        self.assertIn('robot/cmd_vel', s.view()['topics'])  # Last message is still useful.
        s.observe(dict(type='reset'))
        self.assertNotIn('robot/cmd_vel', s.view()['topics'])
        self.assertFalse(s.trail)
        self.assertEqual(s.sequence, 1)

    def test_nonfinite_or_unbounded_values_cannot_replace_last_pose(self):
        s = self.session()
        for x in (float('nan'), float('inf'), 101):
            s.observe(dict(type='pose', x=x, y=0, yaw=0))
        self.assertIsNone(s.pose)
        self.assertEqual(s.sequence, 0)


if __name__ == '__main__':
    unittest.main()
