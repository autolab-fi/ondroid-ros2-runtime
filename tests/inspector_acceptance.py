"""Real Jazzy/PTY graph discovery, custom messages and removal; no physical device."""
import json,os,sys,tempfile,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.workspaces import Manager
from workspace_acceptance import run_shell

PROBE='''import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
rclpy.init()
n=Node('student_probe')
p=n.create_publisher(Float32,'student/error',10)
def tick():
    p.publish(Float32(data=3.5))
n.create_timer(.1,tick)
print('probe ready',flush=True)
rclpy.spin(n)
'''

if __name__=='__main__':
 os.environ['ROS2_WORKSPACE_IMAGE']=sys.argv[1]
 os.environ['ROS2_WORKSPACE_MANAGER']='inspector-test-'+uuid.uuid4().hex
 root=Path(sys.argv[2]);entry=next(x for x in json.loads((root/'manifest.json').read_text())['lessons'] if x['id']=='line_sensor_monitor')
 with tempfile.TemporaryDirectory() as temp:
  os.environ['ROS2_INPUT_ROOT']=temp;m=Manager();s=None
  try:
   reply=m.handle(['sessions'],dict(user_id=991,course_id=20,lesson_id='line_sensor_monitor',source=(root/entry['starter']).read_text(),exercise=entry))
   s=m.sessions[reply['id']]
   import subprocess
   subprocess.run(['docker','exec','-i',s.container,'python3','-c',"from pathlib import Path;import sys;Path('/workspace/ros2_ws/src/probe.py').write_text(sys.stdin.read())"],input=PROBE,text=True,check=True)
   terminal=next(iter(s.terminals.values()))
   run_shell(s,terminal,'python3 src/probe.py\n','probe ready',timeout=30)
   deadline=time.monotonic()+15
   while time.monotonic()<deadline:
    graph=s.view()['inspector'] or {};rows=graph.get('topics',[])
    hit=next((t for t in rows if t['name']=='/student/error' and t.get('value',{}).get('data')==3.5),None)
    if hit and any(n['name']=='/student_probe' for n in graph['nodes']):break
    time.sleep(.2)
   else:raise AssertionError(('Custom topic missing',s.view()))
   assert hit['hz']>0 and hit['publishers']==1
   assert any(n['name']=='/ondroid_bridge' for n in graph['nodes'])
   print(json.dumps(dict(custom_topic=hit,node_names=[n['name'] for n in graph['nodes']])),flush=True)
   run_shell(s,terminal,'\x03','student@ros2:',timeout=15)
   deadline=time.monotonic()+15
   while time.monotonic()<deadline:
    if not any(n['name']=='/student_probe' for n in (s.view()['inspector'] or {}).get('nodes',[])):break
    time.sleep(.2)
   else:raise AssertionError('Stopped node was not removed')
   print('REMOVAL PASSED',flush=True)
  finally:
   if s:s.close(save=False)
