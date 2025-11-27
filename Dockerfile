FROM --platform=linux/arm64 ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive

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

# 파이썬 라이브러리
RUN pip3 install flask opencv-contrib-python transforms3d

# 초기화 및 지형 데이터 설치
RUN rosdep update
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /ros2_ws

# 소스 복사
COPY ./vtol_mission /ros2_ws/src/vtol_mission

# 빌드
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select vtol_mission

# 편의 설정
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# 실행 명령
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch vtol_mission takeoff.launch.py"]
