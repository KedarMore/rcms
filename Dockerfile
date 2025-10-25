FROM ros:humble-perception as build
#REPLACE WITH UNBOX CUSTOM ROS BUILD IMAGE
SHELL ["/bin/bash", "-c"]

COPY . /app/src/rcs_config_manager
WORKDIR /app
RUN source /opt/ros/humble/setup.bash && colcon build --base-path ./src/ ./src/rcs_config_manager/state_manager ./src/rcs_config_manager/robot_msgs ./src/rcs_config_manager/rcs_logger/

# FROM ros:humble-ros-core
FROM registry.gitlab.unboxrobotics.com/unbox_robotics/sort-system/v4/v4-core-config/ros2-humble-lite:june-2024
WORKDIR /app

ENV DEBIAN_FRONTEND noninteractive

    # Remove old ROS keys and source lists
RUN  rm -f /etc/apt/trusted.gpg.d/ros.gpg && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /etc/apt/sources.list.d/ros2.list
    # Add updated GPG key

RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    # Add clean source list
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list


RUN apt-get update && apt-get install python3-pip python3-transforms3d ros-humble-tf2-tools ros-humble-tf-transformations curl -y && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install requests flask fastapi uvicorn transforms3d tenacity flatten-dict
COPY --from=build /app/build  /app/build
COPY --from=build /app/install  /app/install
COPY --from=build /app/src/rcs_config_manager/cms_config /app/src/rcs_config_manager/cms_config
COPY --from=build /app/src/rcs_config_manager/rcs_logger/config /app/config
COPY ./entrypoint.sh /app/

HEALTHCHECK --interval=30s --timeout=4s --retries=1 CMD curl -f http://127.0.0.1:8006/health || exit 1

STOPSIGNAL SIGKILL

SHELL ["/bin/bash", "-c"]
ENTRYPOINT [ "/app/entrypoint.sh" ] 
