"""Bounded Docker PTYs shared by interactive laboratory runtimes.

The caller must authorize ownership and serialize session lifecycle operations.
This package controls only a named container; it never mounts student files.
"""
import base64
import fcntl
import os
import pty
import select
import signal
import struct
import subprocess
import termios
import threading
import time
import uuid
from .control import START_SHELL, STOP_SHELL

class Terminal:
    def __init__(self, container, rcfile='/etc/ondroid/bashrc'):
        self.id = uuid.uuid4().hex
        self.container = container
        self.fd, slave = pty.openpty()
        self.process = subprocess.Popen(['docker', 'exec', '-it', '-e', 'TERM=xterm-256color',
            container, 'python3', '-c', START_SHELL, self.id, rcfile],
            stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        os.close(slave)
        os.set_blocking(self.fd, False)
        self.output = bytearray()
        self.end = self.sequence = 0
        self.client = None
        self.last_input = None
        self.lock = threading.Lock()
        threading.Thread(target=self.read, daemon=True).start()
        # Docker configures the local PTY asynchronously. Input sent before its
        # stream is attached can be flushed during terminal setup. Admit input
        # only after the trusted rcfile/prompt has produced its first output.
        deadline = time.monotonic() + 5
        while not self.end and self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(.02)
        if not self.end or self.process.poll() is not None:
            try:
                self.terminate()
            except Exception:
                self.close()
            raise RuntimeError('Terminal shell did not become ready')

    def read(self):
        while self.process.poll() is None:
            try:
                if select.select([self.fd], [], [], .2)[0]:
                    data = os.read(self.fd, 8192)
                    if not data:
                        break
                    with self.lock:
                        self.end += len(data)
                        self.output.extend(data)
                        del self.output[:-65536]
            except (OSError, ValueError):
                break

    def io(self, body):
        with self.lock:
            if 'client' in body:
                if body.get('attach'):
                    self.client = body['client']
                elif self.client != body['client']:
                    raise ValueError('Terminal opened in another window. Reload to reconnect')
            seq = body.get('sequence')
            if 'input' in body and seq == self.sequence:
                data = base64.b64decode(body['input'], validate=True)
                if len(data) > 4096:
                    raise ValueError('Terminal input exceeds 4 KiB')
                # A single <=4 KiB write is atomic on the PTY; acknowledge only it.
                if os.write(self.fd, data) != len(data):
                    raise RuntimeError('Terminal input interrupted; reconnect before continuing')
                self.sequence += 1
                self.last_input = body['input']
            elif 'input' in body and seq != self.sequence-1:
                raise ValueError('Terminal input sequence changed; reconnect')
            elif 'input' in body and body['input'] != self.last_input:
                raise ValueError('Terminal input changed; reconnect')
            if 'cols' in body:
                cols, rows = int(body['cols']), int(body['rows'])
                if not 10 <= cols <= 400 or not 2 <= rows <= 200:
                    raise ValueError('Invalid terminal dimensions')
                fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
                if self.process.poll() is None:
                    os.kill(self.process.pid, signal.SIGWINCH)
            after = int(body.get('after', 0))
            start = self.end-len(self.output)
            reset = not start <= after <= self.end
            data = bytes(self.output[max(0, after-start) if not reset else 0:])
            return dict(id=self.id, output=base64.b64encode(data).decode(), after=self.end,
                        reset=reset, sequence=self.sequence, exited=self.process.poll() is not None)

    def terminate(self):
        # Killing the local docker CLI alone can leave its remote shell/node alive.
        result = subprocess.run(['docker', 'exec', self.container, 'python3', '-c', STOP_SHELL, self.id],
                                capture_output=True, text=True, timeout=5)
        if result.returncode:
            raise RuntimeError('Terminal could not be stopped; try again or end the session')
        self.close()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        os.close(self.fd)

