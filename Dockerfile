FROM ros:jazzy-ros-base@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698
ENV PYTHONUNBUFFERED=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RCUTILS_LOGGING_USE_STDOUT=0 HOME=/tmp
USER root
RUN apt-get update && apt-get install -y --no-install-recommends nano python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
COPY ondroid_ros2 /opt/ondroid_ros2
RUN printf '#!/bin/bash\nsource /opt/ros/jazzy/setup.bash\nexec python3 /opt/ondroid_ros2/practice_check.py "$@"\n' > /usr/local/bin/check-in-sim && chmod 755 /usr/local/bin/check-in-sim
WORKDIR /tmp
USER 65534:65534
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "/opt/ondroid_ros2/bridge.py"]
