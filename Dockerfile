# 기본 ROS 이미지 사용
FROM ros:foxy

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 레포지토리 코드 복사
COPY . /workspace

# 빌드
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
