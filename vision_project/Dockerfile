FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

# 1. 필수 패키지 설치
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
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. 파이썬 라이브러리 (hailort는 여기서 설치 안 함)
RUN pip3 install --no-cache-dir numpy setuptools

WORKDIR /root/ws

# 3. 코드 복사
COPY siyi_pose_node.py /root/ws/siyi_pose_node.py
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["python3", "siyi_pose_node.py"]