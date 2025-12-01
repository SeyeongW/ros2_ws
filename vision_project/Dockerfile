# 1. 호스트(Bookworm)와 호환되는 최신 ROS 2 Jazzy 사용
FROM ros:jazzy-ros-base

ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

# 2. 필수 패키지 설치
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
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-image-transport \
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. 파이썬 라이브러리 (numpy만 설치, hailo는 설치 X)
# Jazzy는 파이썬 3.12라 --break-system-packages 필요
RUN pip3 install --no-cache-dir numpy setuptools --break-system-packages

WORKDIR /root/ws
COPY siyi_pose_node.py /root/ws/siyi_pose_node.py

# ROS 환경 설정
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["python3", "siyi_pose_node.py"]