"""Terminal exercise helper, deliberately separate from recorded lab grading."""
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node = rclpy.create_node('practice_check')
    reset = node.create_client(Empty, 'simulation/reset')
    if not reset.wait_for_service(timeout_sec=5):
        raise RuntimeError('Practice simulator is unavailable')
    future = reset.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done():
        raise RuntimeError('Simulator reset timed out')
    state = {'pose': None, 'command': None, 'command_at': 0.}
    def pose(msg):
        state['pose'] = (msg.pose.position.x, msg.pose.position.y)
    def command(msg):
        state.update(command=msg, command_at=time.monotonic())
    node.create_subscription(PoseStamped, 'robot/pose', pose, 10)
    node.create_subscription(Twist, 'robot/cmd_vel', command, 10)
    stop = node.create_publisher(Twist, 'robot/cmd_vel', 10)
    print('Practice check: build and run run.sh; target (0.60, -0.185) m, limit 60 s.', flush=True)
    process = subprocess.Popen(['bash', 'run.sh'], start_new_session=True)
    passed, hold = False, None
    deadline = time.monotonic()+60
    try:
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            p, c = state['pose'], state['command']
            good = (p and c and time.monotonic()-state['command_at'] < .5 and
                    math.hypot(p[0]-.6, p[1]+.185) <= .03 and
                    abs(c.linear.x) < .001 and abs(c.angular.z) < .03)
            hold = (hold or time.monotonic()) if good else None
            if hold and time.monotonic()-hold >= .5:
                passed = True
                break
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        for _ in range(3):
            stop.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()
    print('Practice passed. Check in lab verifies a saved project on the real robot.' if passed else
          'Practice did not pass. Inspect your node and try again.', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
