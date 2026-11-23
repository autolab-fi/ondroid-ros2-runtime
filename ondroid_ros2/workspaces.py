"""Private, bounded development sessions. Only the authenticated backend calls this API.

Learner containers have no network or host mounts. Source-only backups are kept
outside the container; build/install/log and generated data stay in bounded tmpfs.
"""
import base64
from collections import deque
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid

from .project import decode, encode, starter
from .line_sensors import analyse
from ondroid_lab_terminal import Terminal




class Session:
    def __init__(self, manager, uid, cid, source, lesson='reach_target', exercise=None):
        self.id = uuid.uuid4().hex
        self.uid, self.cid, self.manager = uid, cid, manager
        self.lesson = lesson
        self.container = 'ondroid-ros2-dev-'+self.id
        self.created = self.active = self.saved = time.monotonic()
        self.state, self.error = 'starting', ''
        self.pose = None
        self.sequence = 0
        self.observed_at = None
        self.received_at = None
        self.topics = {}
        self.command_received_at = None
        self.trail = deque(maxlen=700)
        self.terminals = {}
        self.lock = threading.RLock()
        # Retain the original pilot backup name; other exercises have separate sources.
        suffix = '' if lesson == 'reach_target' else '-'+lesson
        self.backup = manager.root/f'{uid}-{cid}{suffix}.json'
        files = decode(self.backup.read_text()) if self.backup.exists() else starter(source, launch=(exercise or {}).get('launch'))
        self.process = subprocess.Popen(['docker', 'run', '--rm', '--init', '-i', '--name', self.container,
            '--env','ONDROID_ROS2_EXERCISE='+json.dumps(exercise or {}),
            '--label', 'ondroid.ros2.workspace='+manager.identity, '--network', 'none', '--read-only', '--cap-drop', 'ALL',
            '--security-opt', 'no-new-privileges', '--user', '65534:65534', '--cpus', '1',
            '--memory', '768m', '--memory-swap', '768m', '--pids-limit', '128',
            '--log-driver', 'none', '--tmpfs', '/tmp:rw,nosuid,nodev,size=128m',
            '--tmpfs', '/workspace:rw,exec,nosuid,nodev,size=256m,uid=65534,gid=65534,mode=700',
            '--entrypoint', '/ros_entrypoint.sh', manager.image, 'python3', '/opt/ondroid_ros2/interactive.py'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self.read, daemon=True).start()
        try:
            deadline = time.monotonic()+15
            while not self.pose and self.process.poll() is None and time.monotonic()<deadline:
                time.sleep(.05)
            if not self.pose:
                raise RuntimeError('Development simulator did not start')
            self.command('restore', json.dumps(files))
            self.state = 'ready'
            self.new_terminal()
            self.snapshot()
        except Exception:
            self.close(save=False)
            raise

    def read(self):
        while True:
            line = self.process.stdout.readline(32769)
            if not line:
                return
            if len(line) > 32768:
                self.error = 'Simulator output limit exceeded'
                return
            try:
                msg = json.loads(line)
                self.observe(msg)
            except (ValueError, KeyError, TypeError, AttributeError):
                continue

    def observe(self, msg):
        """Bounded practice telemetry, never evidence for the independent grader."""
        with self.lock:
            kind = msg.get('type')
            if kind == 'reset':
                self.trail.clear()
                self.topics.pop('robot/cmd_vel', None)
                self.command_received_at = None
            elif kind in ('line_sensors', 'demo_distance'):
                value = analyse(msg['raw']) if kind == 'line_sensors' else dict(data=float(msg['data']))
                if kind == 'demo_distance' and not math.isfinite(value['data']):
                    return
                if kind == 'line_sensors':value=dict(data=value['raw'])
                topic = 'robot/line_sensors' if kind == 'line_sensors' else 'demo/distance'
                self.topics[topic] = dict(type='topic',topic=topic,
                    message_type='std_msgs/msg/'+('UInt16MultiArray' if kind == 'line_sensors' else 'Float32'),
                    value=value,observed_at=time.time())
            elif kind in ('pose', 'command'):
                keys = ('x', 'y', 'yaw') if kind == 'pose' else ('linear', 'angular')
                value = {k: float(msg[k]) for k in keys}
                if not all(math.isfinite(v) and abs(v) <= 100 for v in value.values()):
                    return
                stamp = time.time()
                topic = 'robot/pose' if kind == 'pose' else 'robot/cmd_vel'
                self.topics[topic] = dict(type='topic', topic=topic,
                    message_type='geometry_msgs/msg/'+('PoseStamped' if kind == 'pose' else 'Twist'),
                    value=value, observed_at=stamp)
                if kind == 'pose':
                    self.pose = value
                    self.trail.append(value)
                    self.sequence += 1
                    self.received_at = time.monotonic()
                    self.observed_at = stamp
                else:
                    self.command_received_at = time.monotonic()

    def command(self, operation, input=None):
        result = subprocess.run(['docker', 'exec', '-i', self.container, 'python3',
            '/opt/ondroid_ros2/workspace_files.py', operation], input=input, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        if result.returncode:
            raise ValueError('Cannot save project: keep UTF-8 source files in src/ and run.sh, without symlinks (64 files, 192 KiB total)')
        return json.loads(result.stdout) if operation != 'restore' else None

    def snapshot(self):
        with self.lock:
            previous = self.command('snapshot')['source']
            for _ in range(3):
                current = self.command('snapshot')['source']
                if current == previous:
                    # Decode again on the trusted host. Never trust container output.
                    current = encode(decode(current))
                    temporary = self.backup.with_suffix('.tmp')
                    temporary.write_text(current)
                    temporary.chmod(0o600)
                    temporary.replace(self.backup)
                    self.saved = time.monotonic()
                    self.error = ''
                    return dict(source=current, source_sha256=hashlib.sha256(current.encode()).hexdigest())
                previous = current
            raise ValueError('Files are changing. Save your editor and try again')

    def new_terminal(self):
        with self.lock:
            # Closed shells free a slot, but reconnecting a tab never spawns a shell.
            for tid, terminal in list(self.terminals.items()):
                if terminal.process.poll() is not None:
                    terminal.close()
                    del self.terminals[tid]
            if len(self.terminals) >= 4:
                raise ValueError('At most four terminals per session')
            terminal = Terminal(self.container, rcfile='/opt/ondroid_ros2/bashrc')
            self.terminals[terminal.id] = terminal
            self.active = time.monotonic()
            return dict(id=terminal.id)

    def view(self):
        with self.lock:
            age = None if self.received_at is None else max(0, time.monotonic()-self.received_at)
            return dict(id=self.id, course_id=self.cid, lesson_id=self.lesson, state=self.state, message=self.error,
                terminals=list(self.terminals), pose=self.pose, trajectory=list(self.trail),
                topics=dict(self.topics), telemetry=dict(sequence=self.sequence, observed_at=self.observed_at,
                    age_ms=None if age is None else round(age*1000),
                    fresh=self.state == 'ready' and age is not None and age < 2,
                    command_active=self.command_received_at is not None and time.monotonic()-self.command_received_at < .5),
                remaining_seconds=max(0, int(1800-(time.monotonic()-self.created))))

    def close(self, save=True):
        with self.lock:
            if self.state == 'ended':
                return
            if save:
                try:
                    self.snapshot()
                except Exception:
                    self.error = 'Latest source backup failed; the previous saved sources were retained'
            self.state = 'cleaning'
            result = subprocess.run(['docker', 'rm', '-f', self.container], capture_output=True, timeout=15)
            if result.returncode and subprocess.run(['docker', 'inspect', self.container], capture_output=True).returncode == 0:
                raise RuntimeError('Workspace cleanup not confirmed')
            for terminal in self.terminals.values():
                terminal.close()
            self.terminals.clear()
            self.process.wait(timeout=5)
            self.state = 'ended'


class Manager:
    def __init__(self):
        self.root = Path(os.environ['ROS2_INPUT_ROOT'])/'workspaces'
        self.root.mkdir(mode=0o700, exist_ok=True)
        self.root.chmod(0o700)
        self.image = os.environ['ROS2_WORKSPACE_IMAGE']
        self.identity = os.getenv('ROS2_WORKSPACE_MANAGER', '1')
        self.sessions = {}
        self.lock = threading.RLock()
        # Never reconnect a forgotten learner process after a service restart.
        old = subprocess.run(['docker', 'ps', '-aq', '--filter', 'label=ondroid.ros2.workspace='+self.identity], capture_output=True, text=True, check=True)
        for ident in old.stdout.split():
            subprocess.run(['docker', 'rm', '-f', ident], check=True, capture_output=True, timeout=15)
        threading.Thread(target=self.reap, daemon=True).start()

    def reap(self):
        while True:
            time.sleep(10)
            for session in list(self.sessions.values()):
                try:
                    age = time.monotonic()
                    if session.state == 'ended':
                        if age-session.created > 3600:
                            self.sessions.pop(session.id, None)
                    elif age-session.created > 1800 or age-session.active > 600 or session.process.poll() is not None:
                        session.close()
                    elif age-session.saved > 30:
                        session.snapshot()
                except Exception as error:
                    session.error = str(error)[:240]

    def handle(self, parts, body):
        uid, cid = body['user_id'], body['course_id']
        if type(uid) is not int or type(cid) is not int or uid < 0 or cid < 1:
            raise ValueError('Invalid owner')
        if parts == ['sessions']:
            lesson = body.get('lesson_id', 'reach_target')
            if not isinstance(lesson, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', lesson):
                raise ValueError('Invalid lesson')
            with self.lock:
                existing = next((s for s in self.sessions.values() if s.uid == uid and s.cid == cid and s.state != 'ended'), None)
                if existing:
                    if existing.lesson != lesson:
                        raise ValueError('End the current session before starting another exercise. Saved sources are kept separately.')
                    return existing.view()
                if sum(s.state != 'ended' for s in self.sessions.values()) >= int(os.getenv('ROS2_WORKSPACE_LIMIT', '2')):
                    raise ValueError('Development server is full. Try again shortly')
                session = Session(self, uid, cid, body['source'], lesson, body.get('exercise'))
                self.sessions[session.id] = session
                return session.view()
        session = self.sessions.get(parts[1]) if len(parts) >= 2 else None
        if not session or (session.uid, session.cid) != (uid, cid):
            raise LookupError('Session not found')
        if 'lesson_id' in body and body['lesson_id'] != session.lesson:
            raise LookupError('Session belongs to another exercise')
        if len(parts) == 2:
            return session.view()
        with session.lock:
            if session.state != 'ready':
                raise ValueError('Session has ended. Start a new session to restore saved files')
            operation = parts[2]
            if operation == 'end':
                # Explicit End must not silently discard invalid/oversized sources.
                session.snapshot()
                session.close(save=False)
                return session.view()
            if operation == 'snapshot':
                return session.snapshot()
            if operation == 'terminals':
                if len(parts) == 3:
                    return session.new_terminal()
                terminal = session.terminals.get(parts[3])
                if not terminal:
                    raise LookupError('Terminal not found')
                if not re.fullmatch('[a-f0-9]{32}', str(body.get('client', ''))):
                    raise ValueError('Invalid terminal connection')
                if body.get('close') is True:
                    terminal.terminate()
                    del session.terminals[terminal.id]
                    session.active = time.monotonic()
                    return session.view()
                if 'input' in body:
                    session.active = time.monotonic()
                return terminal.io(body)
        raise LookupError('Operation not found')


def serve():
    manager = Manager()
    secret = Path(os.environ['ROS2_SERVICE_SECRET_FILE']).read_text().strip()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            if not hmac.compare_digest(self.headers.get('X-ROS2-Service', ''), secret):
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 256 * 1024:
                    raise ValueError('Invalid body size')
                body = json.loads(self.rfile.read(size))
                value = manager.handle(self.path.strip('/').split('/'), body)
                status = 200
            except LookupError:
                value, status = dict(description='Session or terminal not found'), 404
            except (ValueError, KeyError, TypeError) as error:
                value, status = dict(description=str(error)[:240]), 400
            except Exception:
                value, status = dict(description='Development session unavailable'), 503
            data = json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
    ThreadingHTTPServer(('0.0.0.0', 8081), Handler).serve_forever()
