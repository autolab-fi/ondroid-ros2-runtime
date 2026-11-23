"""Unprivileged ROS adapter. Only JSON lines cross the sandbox boundary.

This process is deliberately NOT a grader. All sandbox output is untrusted.
The host owns pose observations, motion limits and the terminal verdict.
"""
import json
import math
import os
import selectors
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import UInt16MultiArray, Float32


def emit(kind, **values):
    print(json.dumps(dict(type=kind, **values), allow_nan=False), flush=True)


class Bridge(Node):
    def __init__(self):
        super().__init__('ondroid_bridge')
        self.pose = self.create_publisher(PoseStamped, 'robot/pose', 10)
        self.create_subscription(Twist, 'robot/cmd_vel', self.command, 10)
        self.line = self.create_publisher(UInt16MultiArray, 'robot/line_sensors', 10)
        self.create_subscription(Float32, 'demo/distance', self.demo_distance, 10)
        self.last_demo = 0.0
        self.last_command = 0.0

    def command(self, msg):
        # Bound traffic even if a learner publishes faster than the course needs.
        now = time.monotonic()
        if now - self.last_command < .09:
            return
        self.last_command = now
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            emit('invalid_command', message='Velocity must be finite')
            return
        emit('command', linear=msg.linear.x, angular=msg.angular.z)

    def demo_distance(self, msg):
        now = time.monotonic()
        if math.isfinite(msg.data) and abs(msg.data) <= 100 and now-self.last_demo >= .09:
            self.last_demo = now
            emit('demo_distance', data=float(msg.data))

    def receive(self, value):
        if value.get('type') == 'line_sensors':
            from line_sensors import analyse
            msg = UInt16MultiArray()
            msg.data = analyse(value['raw'])['raw']
            self.line.publish(msg)
            return
        if value.get('type') != 'pose':
            return
        x, y, yaw = (float(value[k]) for k in ('x', 'y', 'yaw'))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Invalid pose')
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        stamp = float(value['observed_at'])
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pose.publish(msg)


def main():
    rclpy.init()
    bridge = Bridge()
    env = dict(os.environ)
    # Learner code has ordinary rclpy, no platform import or MQTT credentials.
    command = [sys.executable, '-u', '/input/solution.py']
    if Path('/input/project/run.sh').is_file():
        command = ['bash', '-lc', 'mkdir -p /tmp/ros2_ws && cp -r /input/project/. /tmp/ros2_ws/ && cd /tmp/ros2_ws && exec bash run.sh']
    learner = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                               start_new_session=True)
    selector = selectors.DefaultSelector()
    for stream in (sys.stdin.buffer, learner.stdout):
        os.set_blocking(stream.fileno(), False)
        selector.register(stream, selectors.EVENT_READ)
    buffers = {stream: bytearray() for stream in (sys.stdin.buffer, learner.stdout)}
    log_bytes = 0
    emit('ready', ros_distro=os.environ['ROS_DISTRO'])
    deadline = time.monotonic()+(110 if Path('/input/project/run.sh').is_file() else 80)
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(bridge, timeout_sec=.01)
            for key, _ in selector.select(timeout=.01):
                stream = key.fileobj
                data = os.read(stream.fileno(), 4096)
                if not data:
                    if stream is sys.stdin.buffer:
                        return
                    selector.unregister(stream)
                    continue
                buffer = buffers[stream]
                buffer.extend(data)
                if len(buffer) > 32768:
                    del buffer[:-32768]
                while b'\n' in buffer:
                    line, _, remaining = buffer.partition(b'\n')
                    buffer[:] = remaining
                    if stream is sys.stdin.buffer:
                        bridge.receive(json.loads(line))
                    elif log_bytes < 65536:
                        message = line.decode(errors='replace')[:2048]
                        log_bytes += len(message.encode())
                        emit('log', message=message)
            code = learner.poll()
            if code is not None:
                emit('exit', code=code)
                return
    finally:
        if learner.poll() is None:
            os.killpg(learner.pid, signal.SIGKILL)
        learner.wait(timeout=3)
        selector.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
