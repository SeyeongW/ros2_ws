# 파일 위치: /home/sw/my_drone_project/vision/Dockerfile
FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

# 1. GStreamer + Hailo + ROS Vision 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-gi \
    python3-gst-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# 2. Python 라이브러리 (Hailo, Numpy)
RUN pip3 install --no-cache-dir hailort numpy setuptools

WORKDIR /root/ws

# 3. 소스 코드 복사 (같은 폴더에 있는 파일을 컨테이너로)
COPY siyi_pose_node.py /root/ws/siyi_pose_node.py

# 4. 실행 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 5. 실행 명령
CMD ["python3", "siyi_pose_node.py"]