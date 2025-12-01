# 1. 베이스 이미지 변경: Humble(옛날거) -> Jazzy(최신거, Ubuntu 24.04)
# 이렇게 하면 호스트의 최신 라이브러리를 문제없이 돌릴 수 있습니다.
FROM ros:jazzy-ros-base

ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

# 2. 패키지 설치 (Jazzy에 맞는 이름으로 변경됨)
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

# 3. 파이썬 라이브러리 (hailo는 안 씀, numpy만 설치)
# (주의: Jazzy는 파이썬 3.12를 쓰므로 --break-system-packages 옵션이 필요할 수 있으나 pip3가 알아서 처리함)
RUN pip3 install --no-cache-dir numpy setuptools --break-system-packages

WORKDIR /root/ws

# 4. 코드 복사
COPY siyi_pose_node.py /root/ws/siyi_pose_node.py
# 실행 시 Jazzy 환경 로드
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["python3", "siyi_pose_node.py"]
