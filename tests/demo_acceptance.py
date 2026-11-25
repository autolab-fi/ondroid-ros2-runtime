"""Real Jazzy project containers: new demos and the original target exercise."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.project import starter,encode
from ondroid_ros2.runner import run_target
from ondroid_ros2.simulation import Simulation
root=Path(sys.argv[2]);manifest=json.loads((root/'manifest.json').read_text())
for entry in manifest['lessons']:
    if entry['id']=='ros_sandbox' or entry['checker']=='ros_basics_v1':continue
    sim=Simulation();events=[]
    result=run_target(source=encode(starter((root/entry['starter']).read_text(),launch=entry.get('launch'))),image=sys.argv[1],
        sample=sim.sample,drive=sim.drive,stop=sim.stop,event=events.append,exercise=entry)
    topics={e.get('topic') for e in events if e.get('type')=='topic'}
    assert result['status']=='passed',(entry['id'],result,events[-10:])
    assert 'robot/line_sensors' in topics
    if entry['id'] in ('drive_and_topics','line_sensor_follow'):assert 'demo/distance' in topics
    if entry['id']=='line_sensor_monitor':assert any('Right -> left ADC' in e.get('message','') for e in events)
    assert sim.linear==sim.angular==0
    print(json.dumps(dict(activity=entry['id'],topics=sorted(topics),**result)),flush=True)
