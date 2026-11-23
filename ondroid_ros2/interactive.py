"""Live practice simulation. This is not the independent physical grader."""
import json
import time

import rclpy
from std_srvs.srv import Empty

from bridge import Bridge, emit
from simulation import Simulation


class Practice(Bridge):
    def __init__(self):
        super().__init__()
        self.sim = Simulation()
        self.create_service(Empty, 'simulation/reset', self.reset)
        self.create_timer(.1, self.tick)

    def command(self, msg):
        from runner import velocity
        try:
            v, w = velocity(dict(linear=msg.linear.x, angular=msg.angular.z))
            self.sim.drive(v, w, None)
        except ValueError:
            self.sim.stop()
            emit('invalid_command', message='Velocity is outside the practice limits')
            return
        super().command(msg)

    def reset(self, request, response):
        self.sim = Simulation()
        emit('reset')
        return response

    def tick(self):
        pose = self.sim.sample()
        self.receive(dict(type='pose', **{k: pose[k] for k in ('x', 'y', 'yaw')}, observed_at=time.time()))
        emit('pose', **{k: pose[k] for k in ('x', 'y', 'yaw')})


def main():
    rclpy.init()
    node = Practice()
    emit('ready')
    try:
        rclpy.spin(node)
    finally:
        node.sim.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
