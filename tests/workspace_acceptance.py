"""Real containers and PTYs: shared files, separate shells, isolation and snapshots."""
import base64
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ondroid_ros2.project import decode
from ondroid_ros2.runner import run_target
from ondroid_ros2.simulation import Simulation
from ondroid_ros2.workspaces import Manager


def run_shell(session, terminal, command, expected, timeout=20):
    reply = terminal.io(dict(after=0))
    terminal.io(dict(after=reply['after'], sequence=reply['sequence'], input=base64.b64encode(command.encode()).decode()))
    after, output = reply['after'], ''
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        value = terminal.io(dict(after=after))
        after = value['after']
        output += base64.b64decode(value['output']).decode(errors='replace')
        if expected in output:
            return output
        time.sleep(.1)
    raise AssertionError('Terminal did not return expected output: '+output[-2500:])


if __name__ == '__main__':
    os.environ['ROS2_WORKSPACE_IMAGE'] = sys.argv[1]
    os.environ['ROS2_WORKSPACE_MANAGER'] = 'qualification-'+uuid.uuid4().hex
    source = Path(sys.argv[2]).read_text()
    with tempfile.TemporaryDirectory() as root:
        os.environ['ROS2_INPUT_ROOT'] = root
        manager = Manager()
        sessions = []
        try:
            a = manager.handle(['sessions'], dict(user_id=991, course_id=20, source=source))
            first = manager.sessions[a['id']]
            sessions.append(first)
            second_id = first.new_terminal()['id']
            t1, t2 = list(first.terminals.values())
            run_shell(first, t2, 'ros2 node list\n', '/ondroid_bridge')
            # Ordinary terminal execution must drive the browser telemetry without
            # check-in-sim, a submission or a physical queue reservation.
            initial = first.view()
            time.sleep(.3)
            assert first.view()['telemetry']['sequence'] > initial['telemetry']['sequence']
            assert first.view()['telemetry']['fresh']
            run_shell(first, t1, 'bash run.sh\n', 'Waiting for robot/pose', timeout=40)
            deadline = time.monotonic()+8
            while first.view()['pose']['x'] < .43 and time.monotonic() < deadline:
                time.sleep(.1)
            assert first.view()['pose']['x'] >= .43, first.view()
            assert first.view()['topics']['robot/cmd_vel']['value']['linear'] > 0
            run_shell(first, t1, '\x03', 'student@ros2:')
            time.sleep(.7)
            stopped = first.view()['pose']['x']
            time.sleep(.3)
            assert abs(first.view()['pose']['x']-stopped) < .001
            run_shell(first, t2, "ros2 service call /simulation/reset std_srvs/srv/Empty '{}'\n", 'response:', timeout=15)
            time.sleep(.3)
            assert abs(first.view()['pose']['x']-.4) < .001
            run_shell(first, t1, "printf shared > src/probe.txt\n", 'student@ros2:')
            run_shell(first, t2, "cat src/probe.txt; printf '\\nSHARED_OK\\n'\n", 'shared\r\nSHARED_OK')
            for _ in range(2):
                first.new_terminal()
            try:
                first.new_terminal()
                raise AssertionError('Fifth terminal was admitted')
            except ValueError:
                pass
            # Close a terminal with a foreground job, keeping other shells alive.
            command = "python3 -c 'import subprocess,time; from pathlib import Path; p=subprocess.Popen([\"sleep\",\"300\"],start_new_session=True); Path(\"/tmp/detached-terminal.pid\").write_text(str(p.pid)); print(\"DETACHED_READY\",flush=True); time.sleep(300)'\n"
            run_shell(first, t2, command, 'DETACHED_READY\r\n')
            closed_id = t2.id
            try:
                manager.handle(['sessions', first.id, 'terminals', closed_id],
                               dict(user_id=992, course_id=20, client='c'*32, close=True))
                raise AssertionError('Another student closed a terminal')
            except LookupError:
                pass
            manager.handle(['sessions', first.id, 'terminals', closed_id],
                           dict(user_id=991, course_id=20, client='c'*32, close=True))
            assert closed_id not in first.view()['terminals'] and t2.process.poll() is not None
            probe = subprocess.run(['docker', 'exec', first.container, 'python3', '-c',
                'from pathlib import Path; p=Path("/proc/"+Path("/tmp/detached-terminal.pid").read_text()+"/stat"); assert not p.exists() or p.read_text().rsplit(") ",1)[1].split()[0]=="Z"'], capture_output=True)
            assert probe.returncode == 0, 'Detached learner survived terminal close'
            # The host-side close confirms all live processes in that PTY session are gone.
            run_shell(first, t1, "cat src/probe.txt; printf '\\nOTHER_SHELL_OK\\n'\n", 'shared\r\nOTHER_SHELL_OK')
            replacement = first.new_terminal()['id']
            t2 = first.terminals[replacement]
            run_shell(first, t1, 'check-in-sim\n', 'Practice passed.', timeout=45)
            # Retrying an acknowledged input must not execute a command twice.
            reply = t1.io(dict(after=0))
            message = dict(sequence=reply['sequence'], input=base64.b64encode(b'printf x >> src/once.txt\n').decode())
            t1.io(message)
            t1.io(message)
            run_shell(first, t1, "printf '\\nINPUT_SETTLED\\n'\n", '\r\nINPUT_SETTLED\r\n')
            run_shell(first, t2, "wc -c < src/once.txt\n", '1\r\n')
            run_shell(first, t2, "ln -s /etc/passwd src/unsafe; printf '\\nLINK_READY\\n'\n", '\r\nLINK_READY\r\n')
            try:
                first.snapshot()
                raise AssertionError('Symlink was included in snapshot')
            except ValueError:
                pass
            run_shell(first, t2, "rm src/unsafe; printf '\\nLINK_REMOVED\\n'\n", '\r\nLINK_REMOVED\r\n')
            snapshot = first.snapshot()['source']
            assert decode(snapshot)['src/probe.txt'] == 'shared'
            sim = Simulation()
            result = run_target(source=snapshot, image=sys.argv[1], sample=sim.sample,
                drive=sim.drive, stop=sim.stop, event=lambda v: None)
            assert result['status'] == 'passed', result
            b = manager.handle(['sessions'], dict(user_id=992, course_id=20, source=source))
            second = manager.sessions[b['id']]
            sessions.append(second)
            assert 'src/probe.txt' not in decode(second.snapshot()['source'])
            try:
                manager.handle(['sessions', first.id], dict(user_id=992, course_id=20))
                raise AssertionError('Another user accessed the session')
            except LookupError:
                pass
            inspected = json.loads(subprocess.check_output(['docker', 'inspect', first.container]))[0]
            config = inspected['HostConfig']
            assert config['NetworkMode'] == 'none' and not inspected['Mounts']
            assert config['Memory'] == 768*1024*1024 and config['ReadonlyRootfs']
            assert config['Init']
            assert 'size=256m' in config['Tmpfs']['/workspace']
            # A new browser takes control; the previous connection cannot replay input.
            t1.io(dict(client='a'*32, attach=True))
            t1.io(dict(client='b'*32, attach=True))
            try:
                t1.io(dict(client='a'*32))
                raise AssertionError('Replaced browser retained terminal control')
            except ValueError:
                pass
            # Closing the final tab leaves the workspace ready and files intact.
            for tid in list(first.terminals):
                manager.handle(['sessions', first.id, 'terminals', tid],
                               dict(user_id=991, course_id=20, client='c'*32, close=True))
            assert first.view()['terminals'] == [] and first.state == 'ready'
            replacement = first.new_terminal()['id']
            run_shell(first, first.terminals[replacement], "cat src/probe.txt; printf '\\nFILES_RETAINED\\n'\n", 'shared\r\nFILES_RETAINED')
            first.close()
            sandbox = manager.handle(['sessions'], dict(user_id=991, course_id=20, lesson_id='ros_sandbox', source=source))
            sandbox_session = manager.sessions[sandbox['id']]
            sessions.append(sandbox_session)
            assert sandbox['lesson_id'] == 'ros_sandbox'
            assert 'src/probe.txt' not in decode(sandbox_session.snapshot()['source'])
            try:
                manager.handle(['sessions'], dict(user_id=991, course_id=20, source=source))
                raise AssertionError('Switched an active workspace to the wrong exercise')
            except ValueError:
                pass
            sandbox_session.close()
            restored = manager.handle(['sessions'], dict(user_id=991, course_id=20, source=source))
            restarted = manager.sessions[restored['id']]
            sessions.append(restarted)
            assert decode(restarted.snapshot()['source'])['src/probe.txt'] == 'shared'
            print(json.dumps(dict(status='passed', terminals=4, shared_files=True,
                user_isolation=True, persistent_sources=True, terminal_close=True,
                close_last_and_reopen=True, project_sandbox=result, live_terminal_simulation=True,
                command_stop=True, reset=True, exercise_source_isolation=True,
                physical_queue_claimed=False)), flush=True)
        finally:
            for session in sessions:
                session.close()
