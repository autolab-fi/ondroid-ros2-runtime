"""Bounded server runner. Native backend owns authorization and durable state."""
import json
import os
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request

from .runner import run_target
from .simulation import Simulation


def request(path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    headers = {'X-ROS2-Service': Path(os.environ['ROS2_SERVICE_SECRET_FILE']).read_text().strip(),
               'Content-Type': 'application/json'}
    req = urllib.request.Request(os.environ['ROS2_BACKEND_URL'].rstrip('/')+'/ros2'+path,
                                 data=data, headers=headers)
    with urllib.request.urlopen(req, timeout=5) as response:
        content = response.read()
        return json.loads(content) if content else None


def execute(job):
    sim = Simulation()
    events = []
    pending = []
    lock = threading.Lock()
    cancelled = threading.Event()
    done = threading.Event()
    disconnected_since = [None]
    def event(value):
        with lock:
            events.append(value)
            del events[:-100]
    def report():
        while not done.wait(.25):
            with lock:
                pending.extend(events)
                events.clear()
            try:
                state = request('/internal/attempts/'+job['id'],
                    dict(execution_id=job['execution_id'], events=pending[:100]))
                del pending[:100]
                disconnected_since[0] = None
                if state['cancel_requested']:
                    cancelled.set()
            except Exception:
                disconnected_since[0] = disconnected_since[0] or time.monotonic()
                if time.monotonic()-disconnected_since[0] > 3:
                    cancelled.set()
    reporter = threading.Thread(target=report, daemon=True)
    reporter.start()
    try:
        result = run_target(source=job['source'], image=job['release']['runtime_image'],
                            sample=sim.sample, drive=sim.drive, stop=sim.stop, event=event,
                            cancelled=cancelled.is_set, target=job['release']['target'],
                            tolerance=job['release']['tolerance'])
    finally:
        done.set()
        reporter.join(timeout=6)
    with lock:
        pending.extend(events)
    for attempt in range(3):
        try:
            request('/internal/attempts/'+job['id'], dict(execution_id=job['execution_id'], events=pending[-100:], result=result))
            print(json.dumps(dict(attempt_id=job['id'], status=result['status'])), flush=True)
            return
        except Exception as error:
            print('ROS2 result delivery failed: '+type(error).__name__, flush=True)
            time.sleep(.5)


def main():
    # One simulator execution per runner bounds CPU and memory. More replicas can
    # claim disjoint durable rows using PostgreSQL SKIP LOCKED.
    while True:
        try:
            job = request('/internal/sim/claim', {})
            if job:
                execute(job)
            else:
                time.sleep(.5)
        except Exception as error:
            print('ROS2 runner unavailable: '+type(error).__name__, flush=True)
            time.sleep(2)


if __name__ == '__main__':
    main()
