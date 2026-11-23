"""Fail closed before moving a monitor and when physical sensor input goes stale."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.runner import run_target
from ondroid_ros2.simulation import Simulation
root=Path(sys.argv[2]);image=sys.argv[1]
for kind in ('stale','moving_monitor'):
 sim=Simulation();active=[None];sent=[]
 def sample():
  p=sim.sample()
  if active[0] and time.monotonic()-active[0]>1:p['line']['received_monotonic']-=10
  return p
 def drive(v,w,p):
  active[0]=active[0] or time.monotonic();sent.append((v,w));sim.drive(v,w,p)
 source=(root/('lessons/1.2/solution.py' if kind=='stale' else 'lessons/1.3/solution.py')).read_text()
 result=run_target(source=source,image=image,sample=sample,drive=drive,stop=sim.stop,event=lambda e:None,
  exercise={'checker':'line_sensor_observe_v1','line_sensor':True})
 assert result['status']=='execution_error',result
 assert sim.linear==sim.angular==0
 assert not any(abs(v)>.001 or abs(w)>.03 for v,w in sent),sent
 assert ('stale' in result['message']) if kind=='stale' else ('stopped' in result['message'])
 print(json.dumps(dict(case=kind,**result)),flush=True)
