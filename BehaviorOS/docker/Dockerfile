FROM osrf/ros:humble-desktop

# 기본 패키지
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# pyproject.toml로 의존성 설치 (코드 없이 빈 패키지로)
WORKDIR /tmp/deps
COPY pyproject.toml .
RUN mkdir -p behavioros && touch behavioros/__init__.py
RUN pip3 install . && rm -rf /tmp/deps

# PYTHONPATH 설정
ENV PYTHONPATH="/opt/behavioros:${PYTHONPATH}"

# ROS2 워크스페이스 설정
WORKDIR /ros2_ws
RUN mkdir -p src

# 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

CMD ["bash"]