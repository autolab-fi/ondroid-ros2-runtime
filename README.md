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
read-only root, no capabilities, no privileges, one CPU, 384 MiB RAM, 64 PIDs and
a bounded `/tmp`. The container has neither the Docker socket nor MQTT secrets.
Host and worker need the Docker socket, but it never enters learner containers.
A bind-mounted input directory must have the identical absolute path on the
Docker host and in the trusted worker/service (`ROS2_INPUT_ROOT`).

Initial motion limits: 0–0.05 m/s, ±0.8 rad/s, fresh pose within 400 ms, command
silence under 500 ms. The reused robot controller additionally enforces its
600 ms deadman, wheel cap and 60 s absolute session. The bridge has an independent
80 s process lifetime. Startup is bounded separately; queue/reset are not counted
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
