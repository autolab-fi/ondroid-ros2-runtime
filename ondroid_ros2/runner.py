"""Host-owned Docker sandbox lifecycle; no simulator or robot credentials enter it."""
import json
import math
import os
from pathlib import Path
try:
    from .line_sensors import analyse
    from .line_follow import LineFollowProgress
except ImportError:
    from line_sensors import analyse
    from line_follow import LineFollowProgress
import queue
import subprocess
import tempfile
import threading
import time
import uuid
try:
    from .project import decode, PREFIX
except ImportError:  # Also imported by the standalone in-container practice node.
    from project import decode, PREFIX


class SandboxCleanupError(RuntimeError):
    pass


class Sandbox:
    def __init__(self, source, image, *, docker='docker', learning=None):
        self.name = 'ondroid-ros2-' + uuid.uuid4().hex
        self.root = tempfile.TemporaryDirectory(prefix='ondroid-ros2-', dir=os.environ.get('ROS2_INPUT_ROOT'))
        os.chmod(self.root.name, 0o755)
        files = decode(source)
        if files is None:
            files = {'solution.py': source}
            base = Path(self.root.name)
        else:
            base = Path(self.root.name)/'project'
        for name, content in files.items():
            path = base/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
            path.chmod(0o444)
        for directory in base.rglob('*'):
            if directory.is_dir():
                directory.chmod(0o755)
        self.events = queue.Queue(maxsize=256)
        self.process = subprocess.Popen([
            docker, 'run', '--rm', '-i', '--name', self.name,
            *(['--env','ONDROID_ROS2_LEARNING='+json.dumps(learning)] if learning else []),
            '--network', 'none', '--read-only', '--cap-drop', 'ALL',
            '--security-opt', 'no-new-privileges', '--user', '65534:65534',
            '--cpus', '1', '--memory', '384m', '--memory-swap', '384m',
            '--pids-limit', '96', '--tmpfs', '/tmp:rw,exec,nosuid,nodev,size=192m',
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
               timeout=60, target=(.60, -.185), tolerance=.03, assert_owner=lambda: None, exercise=None):
    """Grade trusted observations, never a pose/result published by the learner.

    sample returns {x,y,yaw,sequence,received_monotonic}; drive receives the same
    sample used to authorize the command. Runtime startup precedes motor arming.
    """
    spec = exercise or {}
    checker = spec.get('checker', 'reach_target_v1')
    if checker == 'ros_basics_v1':
        try:from .learning import run_learning
        except ImportError:from learning import run_learning
        try:return run_learning(source=source,image=image,event=event,cancelled=cancelled,exercise=spec)
        finally:stop()
    if checker not in ('reach_target_v1','line_sensor_observe_v1','line_follow_v1'):
        raise ValueError('Unknown ROS exercise checker')
    target = spec.get('target') or target
    tolerance = spec.get('tolerance', tolerance)
    require_line = bool(spec.get('line_sensor'))
    timed = LineFollowProgress(spec['duration_seconds']) if checker == 'line_follow_v1' and spec.get('duration_seconds') else None
    last_linear = 0.0
    line_sequence, line_samples, line_seen = None, 0, 0
    initial_pose = None
    line_lost_since = None
    started = time.monotonic()
    startup_limit = 45 if source.startswith(PREFIX) else 15
    active_since = None
    last_command = None
    last_input = 0
    last_line_input = 0
    reached_since = None
    commands = 0
    result = dict(status='execution_error', message='Execution ended without a result')
    try:
        if startup_limit == 45:
            event(dict(type='log', message='Preparing saved ROS2 project (startup limit: 45 seconds; active control: 60 seconds)', observed_at=time.time()))
        with Sandbox(source, image) as sandbox:
            while True:
                assert_owner()
                now = time.monotonic()
                if cancelled():
                    result = dict(status='cancelled', message='Stopped by the learner')
                    break
                if active_since is None and now-started > startup_limit:
                    raise TimeoutError(f'No velocity command received within {startup_limit} seconds of startup')
                if active_since is not None and now-active_since > timeout:
                    result = dict(status='failed', message='Line-following requirements were not completed within 60 seconds' if timed else 'Target was not reached within 60 seconds')
                    break
                pose = sample()
                if pose is None:
                    if active_since is not None:
                        raise RuntimeError('Robot pose is unavailable; stopped')
                    time.sleep(.02)
                    continue
                if now - pose['received_monotonic'] > .4:
                    raise RuntimeError('Robot pose is stale; stopped')
                if initial_pose is None:
                    initial_pose = (pose['x'],pose['y'])
                line = pose.get('line')
                if require_line and active_since is not None:
                    if line is None:
                        if now-active_since > 2:
                            raise RuntimeError('No physical line sensor telemetry; stopped')
                    elif now-line['received_monotonic'] > .5:
                        raise RuntimeError('Line sensor telemetry is stale; stopped')
                if line is not None and now-line['received_monotonic'] <= .5 and line.get('sequence') != line_sequence and now-last_line_input >= .1:
                    reading = analyse(line['raw'])
                    last_line_input = now
                    line_sequence = line.get('sequence')
                    line_samples += 1
                    line_seen += int(reading['line_visible'])
                    if checker == 'line_follow_v1' and active_since is not None:
                        line_lost_since = None if reading['line_visible'] else (line_lost_since or now)
                        if line_lost_since is not None and now-line_lost_since > 1:
                            raise RuntimeError('The line was lost; stopped')
                    stamp = time.time()-(time.monotonic()-line['received_monotonic'])
                    sandbox.send(dict(type='line_sensors',raw=reading['raw']))
                    event(dict(type='topic',topic='robot/line_sensors',message_type='std_msgs/msg/UInt16MultiArray',value=dict(data=reading['raw']),observed_at=stamp))
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
                        last_linear = linear
                        if checker == 'line_sensor_observe_v1' and (abs(linear)>.001 or abs(angular)>.03):
                            raise ValueError('This sensor observation demo must keep the robot stopped')
                        drive(linear, angular, pose)
                        now = time.monotonic()  # First drive may arm the robot asynchronously.
                        active_since = active_since or now
                        last_command = now
                        commands += 1
                        event(dict(type='topic', topic='robot/cmd_vel', message_type='geometry_msgs/msg/Twist',
                                   value=dict(linear=linear, angular=angular), observed_at=time.time()))
                    elif kind == 'demo_distance':
                        value = float(message.get('data'))
                        if not math.isfinite(value) or abs(value)>100:
                            raise ValueError('Invalid demo distance')
                        event(dict(type='topic',topic='demo/distance',message_type='std_msgs/msg/Float32',value=dict(data=value),observed_at=time.time()))
                    elif kind == 'inspector':
                        try:
                            from .inspector_schema import valid_snapshot
                        except ImportError:
                            from inspector_schema import valid_snapshot
                        try:
                            snapshot=valid_snapshot(message.get('snapshot'))
                        except (ValueError,TypeError):
                            continue
                        event(dict(type='inspector',snapshot=snapshot))
                    elif kind == 'log':
                        event(dict(type='log', message=str(message.get('message', ''))[:2048], observed_at=time.time()))
                    elif kind in ('exit', 'closed', 'error', 'invalid_command'):
                        raise RuntimeError(str(message.get('message') or f"Learner program ended ({message.get('code', kind)})"))
                if last_command is not None and now-last_command > .5:
                    raise RuntimeError('No fresh velocity command for 500 ms; stopped')
                if checker == 'line_sensor_observe_v1':
                    if math.hypot(pose['x']-initial_pose[0],pose['y']-initial_pose[1]) > .015:
                        raise RuntimeError('Robot moved during stationary sensor observation')
                    if active_since is not None and now-active_since >= 3 and line_samples >= 20:
                        result = dict(status='passed',message='Received line sensors while the robot remained stopped')
                        break
                    continue
                if timed and active_since is not None:
                    timed.observe(now, pose['x'], pose['y'], last_linear)
                distance = math.hypot(pose['x']-target[0], pose['y']-target[1])
                # A successful target must be held with explicit zero commands.
                eligible = timed.ready(now) if timed else distance <= tolerance
                if active_since and eligible and message and message.get('type') == 'command':
                    if abs(float(message['linear'])) < .001 and abs(float(message['angular'])) < .03:
                        reached_since = reached_since or now
                        if now-reached_since >= .5:
                            if checker == 'line_follow_v1' and (line_samples<20 or line_seen/line_samples<.65):
                                raise RuntimeError('Insufficient trusted line observations while following the track')
                            result = dict(status='passed', message=f'Followed the line for {timed.duration:g} seconds and stopped', **timed.metrics()) if timed else dict(status='passed', message='Reached the target and stopped', distance_to_target=distance)
                            break
                    else:
                        reached_since = None
                elif not eligible:
                    reached_since = None
    except SandboxCleanupError:
        raise
    except Exception as error:
        result = dict(status='execution_error', message=str(error)[:500])
    finally:
        stop()
    if timed:
        result.update(timed.metrics())
    result.update(line_samples=line_samples, line_visible_samples=line_seen, commands=commands, execution_seconds=round(time.monotonic()-(active_since or started), 3))
    result['startup_seconds'] = round((active_since or time.monotonic())-started, 3)
    return result
