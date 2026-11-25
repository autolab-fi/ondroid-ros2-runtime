"""Real saved sources + PTY practice for every foundational activity."""
import json,os,sys,tempfile,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.workspaces import Manager
from ondroid_ros2.project import starter,decode
from workspace_acceptance import run_shell
root=Path(sys.argv[2]);image=sys.argv[1]
os.environ['ROS2_WORKSPACE_IMAGE']=image;os.environ['ROS2_WORKSPACE_MANAGER']='foundations-'+uuid.uuid4().hex
entries=[e for e in json.loads((root/'manifest.json').read_text())['lessons'] if e['checker']=='ros_basics_v1']
with tempfile.TemporaryDirectory() as temp:
 os.environ['ROS2_INPUT_ROOT']=temp;m=Manager()
 for entry in entries:
  extra={k:(root/v).read_text() for k,v in entry.get('project_files',{}).items()};exercise=dict(entry,project_files=extra)
  view=m.handle(['sessions'],dict(user_id=991,course_id=20,lesson_id=entry['id'],source=(root/entry['starter']).read_text(),exercise=exercise));s=m.sessions[view['id']]
  try:
   initial=decode(s.snapshot()['source'])
   if entry['learning']['kind']=='answers':assert json.loads(initial['src/answers.json']) and 'TODO' in initial['src/answers.json']
   folder=(root/entry['starter']).parent
   if entry['learning']['kind']=='answers':extra['src/answers.json']=(folder/'solution.json').read_text();code=(root/entry['starter']).read_text()
   elif entry['learning']['kind']=='cli_publisher':extra['run.sh']=(folder/'solution.sh').read_text();code=(root/entry['starter']).read_text()
   else:code=(folder/'solution.py').read_text()
   s.command('restore',json.dumps(starter(code,launch='python',project_files=extra)))
   terminal=next(iter(s.terminals.values()));output=run_shell(s,terminal,'check-in-sim\n','Practice passed.',timeout=45)
   assert output.count('PASS:')==3,output
   deadline=time.monotonic()+8
   while time.monotonic()<deadline:
    nodes=[n['name'] for n in (s.view()['inspector'] or {}).get('nodes',[])]
    if not any(n in nodes for n in ('/practice_check','/temperature_publisher','/doubler')):break
    time.sleep(.2)
   else:raise AssertionError(('Practice left a node running',nodes))
   print(json.dumps(dict(lesson=entry['id'],practice='passed',cleanup='confirmed',source_sha256=s.snapshot()['source_sha256'])),flush=True)
  finally:s.close(save=False)
