# Ondroid ROS2 runtime

Native ROS2 Jazzy (Ubuntu 24.04) execution for the Metropolia robotics pilot.
The same learner `solution.py` uses ordinary `rclpy` and standard message types
in simulation and on device 14. This repository contains no robot credentials.

## Components

- `bridge.py` runs inside the unprivileged ROS container. It publishes trusted
  input poses and forwards learner `Twist` messages as bounded JSON lines.
  Learner Python is a separate process with its own normal `rclpy.init()`.
- `runner.py` runs on the trusted host. It owns the Docker lifecycle, validates
  velocity, checks pose freshness and command silence, and grades independent
  position observations. Sandbox output is never trusted as a grade or pose.
- `simulation.py` is a fixed-step velocity-level differential-drive fixture.
  Its midpoint integration matches the existing browser simulator's differential
  drive model. It does not model wheel slip, magnets, motor PID or battery sag.
- `service.py` claims native backend simulation attempts with a private service
  credential. One execution per service bounds resource use. The frontend uses
  the existing browser simulator's arena and rover renderer as a display.
- The existing `lineRobot-micropython-worker` owns physical allocation, reset,
  camera observations, the robot-local motion controller and confirmed stop.

No separate ROS worker competes for the physical Temporal queue. ROS, Python
and OpenCV use the same device claim and one physical activity slot.

## Build and verify

```sh
docker build -t ondroid-ros2-runtime:local .
python3 tests/acceptance.py ../metropolia-ros2-course/examples/reach_target.py ondroid-ros2-runtime:local
docker build -f Dockerfile.service -t ondroid-ros2-service:local .
```

The acceptance script runs actual ROS containers, including success, unsafe
velocity, Python failure and cancellation. It writes a bounded JSON report in
`/tmp`; it never connects to a robot. Deployment records must use immutable local
image IDs or registry digests, rather than a mutable development tag.

## Boundaries and limits

Each attempt has a private network namespace, localhost-only ROS discovery,
read-only root, no capabilities, no privileges, one CPU, 384 MiB RAM, 96 PIDs and
a bounded `/tmp`. The container has neither the Docker socket nor MQTT secrets.
Host and worker need the Docker socket, but it never enters learner containers.
A bind-mounted input directory must have the identical absolute path on the
Docker host and in the trusted worker/service (`ROS2_INPUT_ROOT`).

Initial motion limits: 0–0.05 m/s, ±0.8 rad/s, fresh pose within 400 ms, command
silence under 500 ms. The reused robot controller additionally enforces its
600 ms deadman, wheel cap and 60 s absolute session. The bridge has an independent
80 s process lifetime for single files and 110 s for projects. Startup is bounded
at 15 s for single files and 45 s for project build/launch; queue/reset are not counted
as active control. Losing allocation or backend contact fences commands.
Unconfirmed robot or sandbox cleanup must keep the physical device blocked.

ROS coordinates are metres and radians in a right-handed map, x right and y up.
The current physical topic describes the observed **tag centre**, not encoder
odometry or a calibrated `base_link`; this is explicit in the lesson. The camera
map is converted from centimetres, y-down and clockwise degrees. Topic timestamps
preserve the host's observation timestamp; they are not API polling times.

The initial fixture is lesson 2.6: start (.40, -.185), target (.60, -.185),
tolerance .03 m, hold zero commands for .5 s. Only the host's measured trajectory
can pass. A learner publishing `robot/pose` cannot replace that trajectory.

## Integration status

This is the first vertical slice, not the complete beginner curriculum. Native
course permissions, attempt snapshots, idempotent submission, cancellation,
bounded topic/log/trajectory history and shared physical ownership are integrated
in `ondroid-backend-server`. Detailed qualification evidence and the remaining
course/telemetry work live in its `devops/docker/METROPOLIA_ROS2_PLAN.md` and the
course's `PROJECT_STATUS.md`.

## Interactive development workspaces

Set `ROS2_WORKSPACE_IMAGE` to the qualified native runtime image on the **server**
service. Its private HTTP listener on port 8081 is reachable only by the backend
and authenticates with the existing service secret. Never publish that port.
The backend derives ownership from course enrollment and JWT identity for every
operation. Development sessions never claim a physical device or enter Temporal.
`ROS2_WORKSPACE_MANAGER` identifies the containers owned by this manager (default
`1`); startup cleanup only removes containers with that exact manager label.
Qualification managers use a separate identity and cannot reap live sessions.

One container owns one student's workspace and up to four real PTY/bash shells.
Files live at `/workspace/ros2_ws`; `nano`, colcon and normal ROS2 CLI tools are
available. The built-in simulation publishes `robot/pose`, accepts `robot/cmd_vel`
and provides `simulation/reset` (`std_srvs/srv/Empty`). The browser only displays
poses. `check-in-sim` resets the practice model, builds/runs `run.sh`, checks the
target for up to 60 seconds and terminates its learner process group. It is
practice feedback, not an authoritative laboratory result. Stop other manually
started controllers with Ctrl+C before running it.

Limits: one CPU, 768 MiB RAM, 128 PIDs, 256 MiB workspace tmpfs, 128 MiB temporary
storage; no network or host mounts, read-only root and dropped capabilities.
Executable tmpfs is required for normal installed ROS console entry points.
Default capacity is **two concurrent sessions**, adjustable via
`ROS2_WORKSPACE_LIMIT` after load testing. Sessions end at 30 minutes or after
10 minutes without terminal input; polling alone does not extend them.

Source-only backups are saved every 30 seconds, on explicit End and before a lab
check. One latest backup per user/course is stored privately below
`ROS2_INPUT_ROOT/workspaces`. Only `src/` and `run.sh` are portable: at most 64
UTF-8 regular files, 64 KiB each, 192 KiB total (240 KiB encoded). Hidden files,
symlinks and binaries are not supported. Build/install/log and other generated
data are temporary. Save nano before checking: an unsaved editor buffer is not a
file. On expiry a failed backup preserves the previous valid backup and reports
the failure. A service restart kills orphan development containers and restores
only the last saved sources, never old running processes.

Check in lab freezes the entire source snapshot in the native submission and
reuses the existing exclusive physical queue, reset, 60-second active-control
limit and cleanup. The Pi rebuilds the Python package in its own ARM64 sandbox;
server build products are not copied across architectures. Plain Python attempts
remain supported by the same runner for backward compatibility.

PTY output is bounded to 64 KiB per terminal; input uses sequence acknowledgments
so HTTP retries cannot repeat commands. A new browser connection takes ownership
of a terminal and fences the previous connection. Files survive session restart;
terminal output/history and running processes are not durable.

Individual terminal tabs can be closed without ending the development session.
The server identifies the shell by its container PID and start time, then stops
the processes belonging to that terminal's POSIX session; killing a local Docker
CLI alone is insufficient. Descendants are included even if a helper such as
`check-in-sim` starts its learner in another process session. Other terminals and
saved workspace files remain.
Save nano first: unsaved editor buffers belong to the closing process. Closing
the last terminal keeps the workspace/simulator ready, and `+` opens a new shell.

Qualification commands:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tests/workspace_acceptance.py IMAGE /path/to/course/lessons/2.6/solution.py
```
