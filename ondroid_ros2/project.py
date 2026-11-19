"""Portable source-only snapshots; shared by development and physical sandboxes."""
import json
from pathlib import PurePosixPath

PREFIX = '# ondroid-ros2-workspace-v1\n'
MAX_SOURCE = 192 * 1024


def validate(files):
    if not isinstance(files, dict) or not 1 <= len(files) <= 64 or 'run.sh' not in files:
        raise ValueError('Project needs run.sh and at most 64 source files')
    for name, content in files.items():
        if not isinstance(name, str) or not isinstance(content, str):
            raise ValueError('Only UTF-8 source files are supported')
        path = PurePosixPath(name)
        if (len(name) > 240 or str(path) != name or path.is_absolute() or '..' in path.parts
                or not (name == 'run.sh' or name.startswith('src/'))
                or any(part.startswith('.') for part in path.parts)):
            raise ValueError('Keep project sources in src/ and the launch command in run.sh')
        if len(content.encode()) > 65536:
            raise ValueError('Each source file must be at most 64 KiB')
    if sum(len(k.encode()) + len(v.encode()) for k, v in files.items()) > MAX_SOURCE:
        raise ValueError('Project sources exceed 192 KiB')
    return files


def encode(files):
    value = PREFIX + json.dumps(validate(files), sort_keys=True, ensure_ascii=True, separators=(',', ':'))
    if len(value.encode()) > 240 * 1024:
        raise ValueError('Encoded project exceeds 240 KiB')
    return value


def decode(source):
    return validate(json.loads(source[len(PREFIX):])) if source.startswith(PREFIX) else None


def starter(source):
    return validate({
        'run.sh': 'set -e\nsource /opt/ros/jazzy/setup.bash\ncolcon build --symlink-install >&2\nsource install/setup.bash\nexec ros2 run my_robot controller\n',
        'src/my_robot/package.xml': '<?xml version="1.0"?>\n<package format="3"><name>my_robot</name><version>0.0.1</version><description>Student ROS2 project</description><maintainer email="student@example.org">Student</maintainer><license>MIT</license><buildtool_depend>ament_python</buildtool_depend><exec_depend>rclpy</exec_depend><exec_depend>geometry_msgs</exec_depend><export><build_type>ament_python</build_type></export></package>\n',
        'src/my_robot/setup.py': "from setuptools import setup\nsetup(name='my_robot', version='0.0.1', packages=['my_robot'], data_files=[('share/ament_index/resource_index/packages', ['resource/my_robot']), ('share/my_robot', ['package.xml'])], entry_points={'console_scripts': ['controller = my_robot.controller:main']})\n",
        'src/my_robot/setup.cfg': '[develop]\nscript_dir=$base/lib/my_robot\n[install]\ninstall_scripts=$base/lib/my_robot\n',
        'src/my_robot/resource/my_robot': '',
        'src/my_robot/my_robot/__init__.py': '',
        'src/my_robot/my_robot/controller.py': source,
    })
