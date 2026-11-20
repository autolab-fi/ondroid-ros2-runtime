"""Trusted snippets executed inside one student's container, without host access."""

START_SHELL = r'''
import json, os, sys
from pathlib import Path
pid = os.getpid()
if os.getsid(pid) != pid:
    raise RuntimeError('Terminal requires an independent process session')
start = int(Path(f'/proc/{pid}/stat').read_text().rsplit(') ', 1)[1].split()[19])
path = '/tmp/ondroid-terminal-' + sys.argv[1] + '.json'
fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
with os.fdopen(fd, 'w') as stream:
    json.dump(dict(pid=pid, start=start), stream)
os.execvp('bash', ['bash', '--noprofile', '--rcfile', sys.argv[2], '-i'])
'''

STOP_SHELL = r'''
import json, os, signal, stat, sys, time
from pathlib import Path
path = Path('/tmp/ondroid-terminal-' + sys.argv[1] + '.json')
deadline = time.monotonic() + 1
while not path.exists() and time.monotonic() < deadline:
    time.sleep(.05)
fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
with os.fdopen(fd, 'r') as stream:
    info = os.fstat(stream.fileno())
    if not stat.S_ISREG(info.st_mode) or info.st_size > 1024:
        raise RuntimeError('Invalid terminal identity')
    identity = json.loads(stream.read(1024))
root, start = identity['pid'], identity['start']
if type(root) is not int or root <= 1 or type(start) is not int:
    raise RuntimeError('Invalid terminal identity')
try:
    current = Path(f'/proc/{root}/stat').read_text().rsplit(') ', 1)[1].split()
except FileNotFoundError:
    current = None
if current and (int(current[19]) != start or int(current[3]) != root):
    raise RuntimeError('Terminal process identity changed')

tracked = {}
def members():
    processes = {}
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields = (directory/'stat').read_text().rsplit(') ', 1)[1].split()
            pid = int(directory.name)
            if fields[0] != 'Z' and pid > 1:
                processes[pid] = (int(fields[1]), int(fields[3]), int(fields[19]))
        except (FileNotFoundError, ProcessLookupError):
            pass
    found = {pid for pid, (_, sid, tick) in processes.items()
             if sid == root or tracked.get(pid) == tick}
    # check-in-sim starts its learner in a separate POSIX session. Capture the
    # process tree before stopping its parent, and retain start-time identities
    # after descendants are reparented to the container init process.
    while True:
        children = {pid for pid, (parent, _, _) in processes.items() if parent in found}
        if children <= found:
            break
        found.update(children)
    for pid in found:
        tracked[pid] = processes[pid][2]
    return found

def send(sig):
    for pid in members():
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass

send(signal.SIGHUP)
deadline = time.monotonic() + .5
while members() and time.monotonic() < deadline:
    time.sleep(.05)
send(signal.SIGKILL)
deadline = time.monotonic() + 1
while members() and time.monotonic() < deadline:
    time.sleep(.05)
if members():
    raise RuntimeError('Terminal processes did not stop')
path.unlink(missing_ok=True)
print(json.dumps(dict(closed=True)))
'''
