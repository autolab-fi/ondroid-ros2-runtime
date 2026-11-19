"""Host-owned Docker sandbox lifecycle; no simulator or robot credentials enter it."""
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import uuid


class SandboxCleanupError(RuntimeError):
    pass


class Sandbox:
    def __init__(self, source, image, *, docker='docker'):
        self.name = 'ondroid-ros2-' + uuid.uuid4().hex
        self.root = tempfile.TemporaryDirectory(prefix='ondroid-ros2-', dir=os.environ.get('ROS2_INPUT_ROOT'))
        os.chmod(self.root.name, 0o755)
        path = Path(self.root.name) / 'solution.py'
        path.write_text(source)
        path.chmod(0o444)
        self.events = queue.Queue(maxsize=256)
        self.process = subprocess.Popen([
            docker, 'run', '--rm', '-i', '--name', self.name,
            '--network', 'none', '--read-only', '--cap-drop', 'ALL',
            '--security-opt', 'no-new-privileges', '--user', '65534:65534',
            '--cpus', '1', '--memory', '384m', '--memory-swap', '384m',
            '--pids-limit', '64', '--tmpfs', '/tmp:rw,nosuid,nodev,size=64m',
            '--mount', f'type=bind,source={self.root.name},target=/input,readonly',
            image], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(32769)
                if not line:
                    break
                if len(line) > 32768:
                    self._put(dict(type='error', message='Sandbox output limit exceeded'))
                    break
                try:
                    value = json.loads(line)
                    if isinstance(value, dict):
                        self._put(value)
                except ValueError:
                    self._put(dict(type='error', message='Invalid sandbox protocol'))
                    break
        finally:
            self._put(dict(type='closed'))

    def _put(self, value):
        try:
            self.events.put_nowait(value)
        except queue.Full:
            self.process.terminate()

    def send(self, value):
        self.process.stdin.write(json.dumps(value, allow_nan=False) + '\n')
        self.process.stdin.flush()

    def close(self):
        # docker CLI termination alone does not reliably terminate its container.
        try:
            removed = subprocess.run(['docker', 'rm', '-f', self.name], capture_output=True, timeout=15)
            if removed.returncode:
                probe = subprocess.run(['docker', 'inspect', self.name], capture_output=True, timeout=5)
                if probe.returncode == 0 or b'No such object' not in probe.stderr:
                    raise SandboxCleanupError('Sandbox removal could not be confirmed')
        except subprocess.TimeoutExpired as error:
            raise SandboxCleanupError('Sandbox cleanup timed out') from error
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)
        self.reader.join(timeout=2)
        self.root.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def velocity(value):
    v, w = float(value['linear']), float(value['angular'])
    if not math.isfinite(v) or not math.isfinite(w) or not 0 <= v <= .05 or abs(w) > .8:
        raise ValueError('Use linear.x from 0 to 0.05 m/s and angular.z from -0.8 to 0.8 rad/s')
    return v, w


def run_target(*, source, image, sample, drive, stop, event, cancelled=lambda: False,
               timeout=60, target=(.60, -.185), tolerance=.03, assert_owner=lambda: None):
    """Grade trusted observations, never a pose/result published by the learner.

    sample returns {x,y,yaw,sequence,received_monotonic}; drive receives the same
    sample used to authorize the command. Runtime startup precedes motor arming.
    """
    started = time.monotonic()
    active_since = None
    last_command = None
    last_input = 0
    reached_since = None
    commands = 0
    result = dict(status='execution_error', message='Execution ended without a result')
    try:
        with Sandbox(source, image) as sandbox:
            while True:
                assert_owner()
                now = time.monotonic()
                if cancelled():
                    result = dict(status='cancelled', message='Stopped by the learner')
                    break
                if active_since is None and now-started > 15:
                    raise TimeoutError('No velocity command received within 15 seconds of startup')
                if active_since is not None and now-active_since > timeout:
                    result = dict(status='failed', message='Target was not reached within 60 seconds')
                    break
                pose = sample()
                if pose is None:
                    if active_since is not None:
                        raise RuntimeError('Robot pose is unavailable; stopped')
                    time.sleep(.02)
                    continue
                if now - pose['received_monotonic'] > .4:
                    raise RuntimeError('Robot pose is stale; stopped')
                if now - last_input >= .1:
                    stamp = time.time() - (time.monotonic()-pose['received_monotonic'])
                    sandbox.send(dict(type='pose', x=pose['x'], y=pose['y'], yaw=pose['yaw'], observed_at=stamp))
                    event(dict(type='topic', topic='robot/pose', message_type='geometry_msgs/msg/PoseStamped',
                               value={k: pose[k] for k in ('x','y','yaw')}, observed_at=stamp))
                    last_input = now
                try:
                    message = sandbox.events.get(timeout=.02)
                except queue.Empty:
                    message = None
                if message:
                    kind = message.get('type')
                    if kind == 'command':
                        linear, angular = velocity(message)
                        drive(linear, angular, pose)
                        now = time.monotonic()  # First drive may arm the robot asynchronously.
                        active_since = active_since or now
                        last_command = now
                        commands += 1
                        event(dict(type='topic', topic='robot/cmd_vel', message_type='geometry_msgs/msg/Twist',
                                   value=dict(linear=linear, angular=angular), observed_at=time.time()))
                    elif kind == 'log':
                        event(dict(type='log', message=str(message.get('message', ''))[:2048], observed_at=time.time()))
                    elif kind in ('exit', 'closed', 'error', 'invalid_command'):
                        raise RuntimeError(str(message.get('message') or f"Learner program ended ({message.get('code', kind)})"))
                if last_command is not None and now-last_command > .5:
                    raise RuntimeError('No fresh velocity command for 500 ms; stopped')
                distance = math.hypot(pose['x']-target[0], pose['y']-target[1])
                # A successful target must be held with explicit zero commands.
                if active_since and distance <= tolerance and message and message.get('type') == 'command':
                    if abs(float(message['linear'])) < .001 and abs(float(message['angular'])) < .03:
                        reached_since = reached_since or now
                        if now-reached_since >= .5:
                            result = dict(status='passed', message='Reached the target and stopped', distance_to_target=distance)
                            break
                    else:
                        reached_since = None
                elif distance > tolerance:
                    reached_since = None
    except SandboxCleanupError:
        raise
    except Exception as error:
        result = dict(status='execution_error', message=str(error)[:500])
    finally:
        stop()
    result.update(commands=commands, execution_seconds=round(time.monotonic()-(active_since or started), 3))
    return result
