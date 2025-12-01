FROM --platform=linux/arm64 ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive

# ROS + MAVROS + 도구 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    wget \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Python libs
RUN pip3 install flask opencv-contrib-python transforms3d "numpy<2.0"

RUN rosdep update || true

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh || true

WORKDIR /ros2_ws

# 🔴 여기 중요: src 전체를 복사
COPY ./src /ros2_ws/src

# 빌드
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install --packages-select vtol_mission

# 쉘 들어왔을 때 자동으로 ROS 환경 로드
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# 기본 실행 명령 (launch만)
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch vtol_mission takeoff.launch.py"]