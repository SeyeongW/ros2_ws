FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive

# 1. 필수 패키지 설치
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

# 2. 파이썬 라이브러리 설치 (Flask 등)
RUN pip3 install flask opencv-contrib-python transforms3d

# 3. ROS 의존성 초기화 및 GeographicLib 설치
RUN rosdep update
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /ros2_ws

# 4. 소스 코드 복사
COPY ./vtol_mission /ros2_ws/src/vtol_mission

# 5. 빌드 설정
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select vtol_mission

# 🚨 [추가됨] 접속 시(bash) 자동으로 환경 설정 로드 (.bashrc 수정)
# 이제 'docker exec -it ... bash'로 들어가도 바로 ros2 명령어가 됩니다!
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# 6. 실행 명령 (기존과 동일)
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch vtol_mission takeoff.launch.py"]
