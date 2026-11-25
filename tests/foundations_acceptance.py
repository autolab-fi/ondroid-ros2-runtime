"""Actual Jazzy containers: solutions pass; wrong values/constant outputs/logs fail."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.project import starter,encode
from ondroid_ros2.learning import run_learning
root=Path(sys.argv[2]);image=sys.argv[1]
entries=[e for e in json.loads((root/'manifest.json').read_text())['lessons'] if e['checker']=='ros_basics_v1']

def source(entry,solution=True):
 files={k:(root/v).read_text() for k,v in entry.get('project_files',{}).items()}
 path=root/entry['starter']
 if solution:
  folder=path.parent
  if entry['learning']['kind']=='answers':files['src/answers.json']=(folder/'solution.json').read_text()
  elif entry['learning']['kind']=='cli_publisher':files['run.sh']=(folder/'solution.sh').read_text()
  else:path=folder/'solution.py'
 return encode(starter(path.read_text(),launch=entry.get('launch'),project_files=files))

for entry in entries:
 events=[];result=run_learning(source=source(entry),image=image,exercise=entry,event=events.append)
 assert result['status']=='passed',(entry['id'],result,events[-10:])
 assert len([e for e in events if e.get('type')=='log' and e['message'].startswith('PASS:')])==3
 print(json.dumps(dict(lesson=entry['id'],**result)),flush=True)
for id,alter,label in [
 ('write_publisher',lambda s:s.replace('message.data = 20.5','message.data = 0.0'),'wrong_temperature'),
 ('write_subscriber',lambda s:s.replace('message.data * 2','14'),'constant_response'),
 ('write_publisher',lambda s:"print('PASS: all criteria passed')\n",'fake_stdout')]:
 entry=next(e for e in entries if e['id']==id);events=[]
 solution=(root/entry['starter']).with_name('solution.py').read_text();files=starter(alter(solution),launch='python')
 result=run_learning(source=encode(files),image=image,exercise=entry,event=events.append)
 assert result['status']=='failed',(label,result)
 print(json.dumps(dict(negative=label,**result)),flush=True)
