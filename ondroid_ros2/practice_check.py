"""Terminal exercise helper, deliberately separate from recorded lab grading."""
import math
import json
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty
from std_msgs.msg import UInt16MultiArray
from line_sensors import analyse
from line_follow import LineFollowProgress


def main():
    spec = json.loads(os.getenv('ONDROID_ROS2_EXERCISE','{}'))
    checker = spec.get('checker','reach_target_v1')
    if checker == 'ros_basics_v1':
        from learning_practice import main as foundations
        return foundations(spec)
    target = spec.get('target') or [.6,-.185]
    tolerance = spec.get('tolerance',.03)
    timed = LineFollowProgress(spec['duration_seconds']) if checker == 'line_follow_v1' and spec.get('duration_seconds') else None
    line = {'count':0,'seen':0,'at':0.}
    initial = [None]
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
        state['pose_at'] = time.monotonic()
        if initial[0] is None:initial[0]=state['pose']
    def command(msg):
        state.update(command=msg, command_at=time.monotonic())
    node.create_subscription(PoseStamped, 'robot/pose', pose, 10)
    node.create_subscription(Twist, 'robot/cmd_vel', command, 10)
    def sense(msg):
        reading = analyse(list(msg.data))
        line['count'] += 1
        line['seen'] += int(reading['line_visible'])
        line['at'] = time.monotonic()
    node.create_subscription(UInt16MultiArray, 'robot/line_sensors', sense, 10)
    stop = node.create_publisher(Twist, 'robot/cmd_vel', 10)
    print(f'Practice check: {checker}; build/run run.sh, limit 60 s.', flush=True)
    process = subprocess.Popen(['bash', 'run.sh'], start_new_session=True)
    passed, hold = False, None
    deadline = time.monotonic()+60
    try:
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            p, c = state['pose'], state['command']
            good = (p and c and time.monotonic()-state['command_at'] < .5 and
                    math.hypot(p[0]-target[0], p[1]-target[1]) <= tolerance and
                    abs(c.linear.x) < .001 and abs(c.angular.z) < .03)
            if checker == 'line_sensor_observe_v1':
                good = (p and initial[0] and c and line['count']>=20 and time.monotonic()-line['at']<.5
                    and math.hypot(p[0]-initial[0][0],p[1]-initial[0][1])<.015
                    and abs(c.linear.x)<.001 and abs(c.angular.z)<.03)
            if timed and p and c:
                timed.observe(time.monotonic(), *p, c.linear.x)
                good = (timed.ready(time.monotonic()) and time.monotonic()-state.get('pose_at',0)<.4
                    and time.monotonic()-state['command_at']<.5 and abs(c.linear.x)<.001 and abs(c.angular.z)<.03)
            if checker == 'line_follow_v1':
                good = good and line['count']>=20 and line['seen']/line['count']>=.65
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
