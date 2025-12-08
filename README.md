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

### Global Mapping에서 IndeterminantLinearSystemException 에러

맵핑 중 아래와 같은 에러가 발생하는 경우:

```
[global] [error] an indeterminant linear system exception was caught during global map optimization!!
Indeterminant linear system detected while working near variable (Symbol: v2)
```

`config_global_mapping_cpu.json`에서 `enable_imu`를 `false`로 설정:

```json
"enable_imu": false
```

이 설정은 Global Mapping에서 IMU를 비활성화합니다. Odometry에서는 여전히 IMU를 사용하므로 맵 품질에 큰 영향은 없습니다.

## 출력 파일 설명

`glim_rosbag` 또는 `glim_rosnode` 실행 후 `glim_ros/Log/map_날짜시간/` 폴더에 다음 파일들이 저장됩니다:

### Trajectory 파일 (TUM 형식)

모든 trajectory 파일은 TUM RGB-D 형식을 따릅니다:
```
timestamp x y z qx qy qz qw
```
- `timestamp`: 프레임 타임스탬프 (초)
- `x y z`: 위치 (미터)
- `qx qy qz qw`: 쿼터니언 회전

| 파일 | 설명 |
|------|------|
| `odom_imu.txt` | IMU 프레임의 Odometry 궤적 (루프 클로저 미적용) |
| `traj_imu.txt` | IMU 프레임의 Global Mapping 궤적 (루프 클로저 적용) |
| `odom_lidar.txt` | LiDAR 프레임의 Odometry 궤적 (루프 클로저 미적용) |
| `traj_lidar.txt` | LiDAR 프레임의 Global Mapping 궤적 (루프 클로저 적용) |

### 그래프 및 최적화 파일

| 파일 | 설명 |
|------|------|
| `graph.txt` | 팩터 그래프 메타데이터 (서브맵 수, 프레임 수, 매칭 팩터 정보) |
| `values.bin` | GTSAM 최적화 결과 (포즈, 속도, IMU 바이어스) - 바이너리 형식 |

### Submap 파일

| 파일 | 설명 |
|------|------|
| `submap_XXXX.bin` | 각 서브맵의 포인트 클라우드 데이터 |

## 패키지 설명

| 패키지 | 설명 |
|--------|------|
| `glim_ros` | ROS2 노드 및 메시지 처리 |
| `glim` | LiDAR-IMU SLAM 코어 알고리즘 |
| `glim_ext` | GLIM 확장 모듈 (ScanContext 루프 클로저 등) |
| `gtsam_points` | GTSAM 기반 포인트 클라우드 등록 팩터 |
| `iridescence` | OpenGL 기반 3D 시각화 라이브러리 |

---

## System Workflow with MLX Preset

본 시스템은 MLX LiDAR와 카메라를 사용한 LiDAR-IMU SLAM 및 RGB Colorization을 위해 최적화되어 있습니다.

### 설정 파일 구조

모든 설정 파일은 `src/glim/config/presets/mlx/` 폴더에 위치합니다:

```
config/presets/mlx/
├── config_ros.json              # ROS 토픽 설정, 확장 모듈 활성화
├── config_sensors.json          # 센서 파라미터 (LiDAR-IMU/Camera 캘리브레이션)
├── config_preprocess.json       # 포인트 클라우드 전처리 (다운샘플링, 범위 필터)
├── config_odometry_cpu.json     # CPU 기반 Odometry (GICP/VGICP/SmallGICP)
├── config_odometry_gpu.json     # GPU 기반 Odometry (VGICP-GPU)
├── config_sub_mapping_cpu.json  # 서브맵 생성 (CPU)
├── config_sub_mapping_gpu.json  # 서브맵 생성 (GPU)
├── config_global_mapping_cpu.json  # 전역 최적화 (CPU)
├── config_global_mapping_gpu.json  # 전역 최적화 (GPU)
├── config_scan_context.json     # 루프 클로저 검출 설정
├── config_gravity_estimator.json   # 중력 추정 모듈 설정
└── config_viewer.json           # 시각화 설정
```

### 데이터 흐름 파이프라인

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   LiDAR     │────▶│  Preprocessing   │────▶│   Odometry      │
│   Points    │     │  (Downsampling)  │     │  (GICP/VGICP)   │
└─────────────┘     └──────────────────┘     └────────┬────────┘
                                                      │
┌─────────────┐                                       │
│    IMU      │───────────────────────────────────────┤
│   Data      │    (Integration & Deskewing)          │
└─────────────┘                                       ▼
                                             ┌─────────────────┐
┌─────────────┐     ┌──────────────────┐     │   Sub-Mapping   │
│   Camera    │────▶│  RGB Colorizer   │◀────│   (Voxelmap)    │
│   Image     │     │  (FOV Filtering) │     └────────┬────────┘
└─────────────┘     └────────┬─────────┘              │
                             │                        ▼
                             │              ┌─────────────────┐
                             │              │  Global Mapping │
                             │              │  (Optimization) │
                             │              └────────┬────────┘
                             │                       │
                             ▼                       ▼
                    ┌─────────────────────────────────────────┐
                    │           Output Files                  │
                    │  - traj_lidar.txt (Optimized trajectory)│
                    │  - Submap point clouds                  │
                    │  - global_map_rgb_fov.pcd (RGB map)     │
                    │  - profiling_stats.txt                  │
                    └─────────────────────────────────────────┘
```

### 확장 모듈 (Extension Modules)

`config_ros.json`에서 확장 모듈을 활성화/비활성화할 수 있습니다:

#### 1. ScanContext Loop Detector
```json
"extension_modules": ["libscan_context_loop_detector.so"]
```
- 역할: ScanContext 기반 루프 클로저 검출
- 설정: `config_scan_context.json`
- 루프 폐합 시 drift 보정에 필수적

#### 2. RGB Colorizer
```json
"rgb_colorizer_enabled": true,
"rgb_image_topic": "/camera/image/compressed"
```
- 역할: 카메라 이미지를 이용한 포인트 클라우드 컬러화
- 설정: `config_sensors.json`의 카메라 파라미터
- 기능:
  - 카메라 FOV 내 포인트만 RGB 색상 적용
  - Motion compensation (LiDAR-Camera 시간차 보정)
  - Z-buffer 기반 가려짐 처리
  - 종료 시 `global_map_rgb_fov.pcd` 자동 저장

#### 3. Gravity Estimator
```json
"gravity_estimator_enabled": true
```
- 역할: IMU 데이터 기반 중력 방향 추정
- 설정: `config_gravity_estimator.json`
- 초기화 시 수평 정렬에 사용

### Odometry Registration 타입

`config_odometry_cpu.json` 또는 `config_odometry_gpu.json`에서 선택:

| 타입 | 설명 | 사용 환경 |
|------|------|----------|
| `GICP` | iVox 기반 GICP | CPU |
| `VGICP` | Voxelized GICP | CPU/GPU |
| `SmallGICP` | 경량 GICP (small_gicp 라이브러리) | CPU |

GPU 버전 (`config_odometry_gpu.json`)은 VGICP-GPU만 지원합니다.

### CPU vs GPU 설정 전환

`config_ros.json`에서 CPU/GPU 모드를 선택합니다:

```json
// GPU 모드
"odometry_so_name": "libodometry_estimation_gpu.so",
"sub_mapping_so_name": "libsub_mapping_gpu.so",

// CPU 모드
"odometry_so_name": "libodometry_estimation_cpu.so",
"sub_mapping_so_name": "libsub_mapping_cpu.so"
```

### 카메라 캘리브레이션 설정

`config_sensors.json`에서 카메라 파라미터를 설정합니다:

```json
{
  "sensors": {
    "global_shutter_camera": true,
    "image_size": [1920, 1200],
    "intrinsics": [fx, fy, cx, cy],
    "distortion_model": "plumb_bob",
    "distortion_coeffs": [k1, k2, p1, p2, k3],
    "T_lidar_camera": [tx, ty, tz, qx, qy, qz, qw]  // TUM format
  }
}
```

### 프로파일링 통계

`profiling_stats.txt`에 저장되는 주요 측정 항목:

| 항목 | 설명 |
|------|------|
| `odometry/total` | 전체 Odometry 처리 시간 |
| `odometry/icp_optimization` | ICP 최적화 시간 (CPU GICP/VGICP) |
| `odometry/small_gicp_optimization` | SmallGICP 최적화 시간 |
| `odometry_cpu/create_factors` | CPU Factor 생성 시간 |
| `odometry_gpu/create_factors` | GPU Factor 생성 시간 |

### 원점 회귀 오차 계산 (Loop Closure 평가)

`traj_lidar.txt`를 사용하여 시작-종료 위치/방향 오차를 계산할 수 있습니다:

```python
import numpy as np

# traj_lidar.txt 로드 (TUM format: timestamp x y z qx qy qz qw)
traj = np.loadtxt("traj_lidar.txt")

# 시작/종료 위치
start_pos = traj[0, 1:4]
end_pos = traj[-1, 1:4]
position_error = np.linalg.norm(end_pos - start_pos)

# 시작/종료 쿼터니언
start_quat = traj[0, 4:8]  # qx, qy, qz, qw
end_quat = traj[-1, 4:8]

# 방향 오차 계산 (quaternion difference)
from scipy.spatial.transform import Rotation
r_start = Rotation.from_quat(start_quat)
r_end = Rotation.from_quat(end_quat)
r_diff = r_start.inv() * r_end
angle_error_rad = r_diff.magnitude()
angle_error_deg = np.degrees(angle_error_rad)

print(f"Position error: {position_error*100:.2f} cm")
print(f"Orientation error: {angle_error_deg:.2f} degrees")
```

### 결과 파일 요약

프로그램 종료 시 `glim_ros/Log/map_YYYYMMDD_HHMMSS/` 폴더에 저장:

| 파일/폴더 | 설명 |
|-----------|------|
| `traj_lidar.txt` | 최적화된 LiDAR 궤적 (TUM format) |
| `odom_lidar.txt` | 루프 클로저 미적용 Odometry 궤적 |
| `profiling_stats.txt` | 연산 시간 통계 |
| `global_map_rgb_fov.pcd` | RGB 색상이 입혀진 FOV 포인트 맵 |
| `XXXXXX/` | 각 서브맵 데이터 (point clouds, poses) |
| `config/` | 사용된 설정 파일 백업 |
