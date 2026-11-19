"""Run inside the learner container. Never follow student-created symlinks."""
import json
import os
from pathlib import Path
import stat
import sys

from project import encode, validate

ROOT = Path('/workspace/ros2_ws')


def snapshot():
    files = {}
    for base, dirs, names in os.walk(ROOT, followlinks=False):
        relative = Path(base).relative_to(ROOT)
        dirs[:] = [d for d in dirs if not d.startswith('.') and (relative.parts or d == 'src')]
        for name in names:
            path = Path(base)/name
            key = path.relative_to(ROOT).as_posix()
            if key != 'run.sh' and not key.startswith('src/'):
                continue
            if any(p.startswith('.') for p in Path(key).parts) or '__pycache__' in Path(key).parts:
                continue
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            try:
                info = os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or info.st_size > 65536:
                    raise ValueError('Only regular source files up to 64 KiB are supported')
                with os.fdopen(fd, 'rb', closefd=False) as stream:
                    content = stream.read(65537)
                if len(content) > 65536:
                    raise ValueError('Source file grew while saving')
                files[key] = content.decode('utf-8')
                validate({**files, 'run.sh': files.get('run.sh', '')})
            finally:
                os.close(fd)
    return encode(files)


if __name__ == '__main__':
    if sys.argv[1] == 'restore':
        files = validate(json.load(sys.stdin))
        for key, content in files.items():
            path = ROOT/key
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    else:
        print(json.dumps({'source': snapshot()}))
