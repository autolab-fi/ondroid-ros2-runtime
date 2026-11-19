"""Real ROS2 containers: success, missing commands, unsafe speed, crash, cancellation."""
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ondroid_ros2.runner import run_target
from ondroid_ros2.simulation import Simulation

source = pathlib.Path(sys.argv[1]).read_text()
image = sys.argv[2] if len(sys.argv)>2 else 'ondroid-ros2-runtime:pilot-20261119'
results = []
for name, code, cancel, expected in [
    ('target', source, False, 'passed'),
    ('unsafe_velocity', source.replace('0.04 if', '0.40 if'), False, 'execution_error'),
    ('syntax_error', 'this is invalid python!', False, 'execution_error'),
    ('cancelled', source, True, 'cancelled'),
]:
    sim = Simulation()
    count = [0]
    def event(value):
        count[0] += 1
    start = time.monotonic()
    result = run_target(source=code, image=image, sample=sim.sample, drive=sim.drive,
                        stop=sim.stop, event=event, cancelled=lambda: cancel and time.monotonic()-start>2)
    row = dict(name=name, events=count[0], **result)
    results.append(row)
    print(json.dumps(row), flush=True)
    assert result['status'] == expected, row
    assert sim.linear == sim.angular == 0
pathlib.Path('/tmp/ros2-runtime-acceptance.json').write_text(json.dumps(results, indent=2))
