# glim_ros2

LiDAR-IMU SLAM을 위한 GLIM의 ROS2 래퍼 패키지입니다. 모든 의존성이 git submodule로 포함되어 있어 클론 후 바로 빌드할 수 있습니다.

## 사전 요구사항

### 시스템 의존성 설치

```bash
# 시스템 라이브러리 설치
sudo apt install libomp-dev libboost-all-dev libmetis-dev \
                 libfmt-dev libspdlog-dev \
                 libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# ROS2 GTSAM 패키지 설치
sudo apt install ros-humble-gtsam
```

## 설치 및 빌드

### 1. 저장소 클론

```bash
# submodule과 함께 클론
git clone --recursive https://github.com/SOSLAB-IVC/glim_ros2
```

### 2. 초기 설정 (setup.sh)

```bash
cd glim_ros2
./setup.sh
```

**setup.sh가 하는 일:**
- git submodule 초기화 및 다운로드 (`glim`, `gtsam_points`, `iridescence`)
- `gtsam_points`를 호환 버전(v1.0.8)으로 체크아웃 (ros-humble-gtsam 4.2와 호환)
- `iridescence`의 하위 submodule 초기화 (imgui, implot 등)

> **참고**: `git clone --recursive`를 사용하지 않았거나, submodule이 비어있는 경우 반드시 실행해야 합니다.

### 3. 빌드

```bash
# colcon workspace로 이동 (glim_ros2가 src/ 안에 있는 경우)
cd ~/colcon_ws

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 빌드
colcon build

# 빌드 결과 소싱
source install/setup.bash
```

### 빌드 옵션

```bash
# CUDA 지원 (GPU 가속)
colcon build --cmake-args -DBUILD_WITH_CUDA=ON

# 뷰어 없이 빌드 (헤드리스 모드)
colcon build --cmake-args -DBUILD_WITH_VIEWER=OFF
```

## 실행 방법

### glim_rosnode (실시간 처리)

```bash
# 터미널 1: GLIM 노드 실행
ros2 run glim_ros glim_rosnode

# 터미널 2: rosbag 재생
ros2 bag play {rosbag2 파일 경로}

# 터미널 3: RViz 시각화 (선택사항)
rviz2 -d src/glim_ros2/src/glim_ros/rviz/glim_ros.rviz
```

### glim_rosbag (오프라인 처리)

```bash
ros2 run glim_ros glim_rosbag {rosbag2 파일 경로}
```

### offline_viewer (맵 시각화)

저장된 맵을 불러와 시각화하고 편집할 수 있습니다.

```bash
# 기본 사용법
ros2 run glim_ros offline_viewer --map_path /tmp/dump

# 옵션
ros2 run glim_ros offline_viewer --help
```

| 옵션 | 설명 |
|------|------|
| `--map_path` | 저장된 맵 폴더 경로 (필수) |
| `--config_path` | 설정 경로 (기본: `config/presets/mlx`) |
| `--debug` | 디버그 출력 활성화 |

### 설정 파일 사용

커스텀 설정 파일을 사용하려면:

```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath src/glim_ros2/src/glim/config/presets/mlx)
```

## 프로젝트 구조

```
glim_ros2/
├── src/
│   ├── glim_ros/            # ROS2 래퍼 패키지
│   ├── glim/                # GLIM 코어 알고리즘 (submodule)
│   │   └── config/          # 설정 파일들
│   │       └── presets/     # 센서별 프리셋 (mlx 등)
│   ├── gtsam_points/        # 포인트 클라우드 등록 팩터 (submodule)
│   └── iridescence/         # 3D 시각화 라이브러리 (submodule)
├── setup.sh                 # 초기 설정 스크립트
└── README.md
```

## 문제 해결

### fmt/spdlog 버전 불일치

시스템의 spdlog가 fmt v8을 사용합니다. 링크 에러 발생 시 클린 빌드:

```bash
rm -rf build install log
colcon build
```

### submodule이 비어있는 경우

```bash
# glim_ros2 디렉토리에서
./setup.sh

# 또는 수동으로
git submodule update --init --recursive
```

## 패키지 설명

| 패키지 | 설명 |
|--------|------|
| `glim_ros` | ROS2 노드 및 메시지 처리 |
| `glim` | LiDAR-IMU SLAM 코어 알고리즘 |
| `gtsam_points` | GTSAM 기반 포인트 클라우드 등록 팩터 |
| `iridescence` | OpenGL 기반 3D 시각화 라이브러리 |
