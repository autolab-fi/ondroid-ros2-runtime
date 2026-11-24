"""Real ROS terminals: minute following, practice, and safe reference upgrades."""
import base64,hashlib,json,os,subprocess,sys,tempfile,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.workspaces import Manager
from ondroid_ros2.project import starter,encode,decode
from workspace_acceptance import run_shell
root=Path(sys.argv[2]);image=sys.argv[1]
new=(root/'lessons/1.4/solution.py').read_text()
old=subprocess.check_output(['git','-C',str(root),'show','109a4fa:lessons/1.4/solution.py']).decode()
spec=next(e for e in json.loads((root/'manifest.json').read_text())['lessons'] if e['id']=='line_sensor_follow')
os.environ['ROS2_WORKSPACE_IMAGE']=image
os.environ['ROS2_WORKSPACE_MANAGER']='qualification-'+uuid.uuid4().hex
with tempfile.TemporaryDirectory() as tmp:
 os.environ['ROS2_INPUT_ROOT']=tmp;m=Manager()
 for uid,changed in [(990,False),(991,True)]:
  files=starter(old+('\n# My edits\n' if changed else ''),launch='python')
  (m.root/f'{uid}-20-line_sensor_follow.json').write_text(encode(files))
  view=m.handle(['sessions'],dict(user_id=uid,course_id=20,lesson_id='line_sensor_follow',source=new,exercise=spec));s=m.sessions[view['id']]
  try:
   restored=decode(s.snapshot()['source'])['src/my_robot/my_robot/controller.py']
   assert restored==(files['src/my_robot/my_robot/controller.py'] if changed else new)
   if changed:continue
   terminal=next(iter(s.terminals.values()));started=time.monotonic()
   run_shell(s,terminal,'bash run.sh\n','Following the line for 55 seconds.',timeout=20)
   last=None;until=time.monotonic()+60
   while time.monotonic()<until:
    out=base64.b64decode(terminal.io(dict(after=0))['output']).decode(errors='replace')
    if 'Demo finished after 55 seconds; stopped.' in out:break
    time.sleep(.25)
   else:raise AssertionError('No timed completion log')
   time.sleep(.5);v=s.view();assert abs(v['topics']['robot/cmd_vel']['value']['linear'])<.001
   p=v['pose'];time.sleep(.5);q=s.view()['pose'];assert abs(q['x']-p['x'])+abs(q['y']-p['y'])<.001
   print(json.dumps(dict(manual='passed',elapsed=round(time.monotonic()-started,2),pose=q,upgrade='unmodified reference updated')),flush=True)
   run_shell(s,terminal,'\x03','student@ros2:')
   run_shell(s,terminal,'check-in-sim\n','Practice passed.',timeout=75)
   print(json.dumps(dict(practice='passed',seconds=55)),flush=True)
  finally:s.close()
 print(json.dumps(dict(edited_sources='preserved')),flush=True)
