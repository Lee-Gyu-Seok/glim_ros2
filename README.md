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

### CMake 4.0 호환성 문제 (VTK MPI 에러)

pip로 CMake 4.0이 설치된 경우 아래와 같은 에러가 발생할 수 있습니다:

```
CMake Error at /usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-targets.cmake:518 (set_target_properties):
  The link interface of target "VTK::mpi" contains:
    MPI::MPI_C
  but the target was not found.
```

해결 방법:

```bash
# pip로 설치된 cmake 제거
pip3 uninstall cmake

# 시스템 cmake 확인 (3.22 이상이면 OK)
cmake --version
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
├── config_odometry_ct.json      # CT 모드 Odometry (Continuous-Time GICP)
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
│   Points    │     │  (Downsampling)  │     │   (CT-GICP)     │
└─────────────┘     └──────────────────┘     └────────┬────────┘
                                                      │
┌─────────────┐                                       │
│    IMU      │───────────────────────────────────────┤
│   Data      │    (Deskewing & Initial Prediction)   │
└─────────────┘                                       │
                                                      ▼
                                            ┌─────────────────┐
                                            │   Sub-Mapping   │
                                            │  (VGICP-GPU)    │
                                            └────────┬────────┘
┌─────────────┐     ┌──────────────────┐             │
│   Camera    │────▶│  RGB Colorizer   │◀────────────┤
│   Image     │     │  (FOV Filtering) │             │
└─────────────┘     └──────────────────┘             ▼
                                            ┌─────────────────┐
                    ┌──────────────────┐    │  Global Mapping │
                    │    ScanContext   │───▶│  (VGICP-GPU)    │
                    │  Loop Detector   │    └────────┬────────┘
                    └──────────────────┘             │
                                                     ▼
                    ┌─────────────────────────────────────────┐
                    │           Output Files                  │
                    │  - traj_lidar.txt (Optimized trajectory)│
                    │  - Submap point clouds (RGB)            │
                    │  - downsampled_map.pcd                  │
                    │  - profiling_stats.txt                  │
                    └─────────────────────────────────────────┘
```

### 핵심 모듈 설명

#### 1. Odometry Estimation (오도메트리)
LiDAR 포인트클라우드와 IMU 데이터를 융합하여 실시간 자세 추정을 수행합니다.

##### GPU 모드 (VGICP-GPU)
- 설정 파일: `config_odometry_gpu.json`
- 현재 설정:
  - **Registration**: VGICP-GPU (Voxelized GICP, CUDA 가속)
  - **IMU 융합**: 가속도계/자이로스코프 데이터로 포인트클라우드 deskewing 및 초기 자세 추정
  - **최적화**: iSAM2 기반 슬라이딩 윈도우 최적화

```json
{
  "so_name": "libodometry_estimation_gpu.so",
  "vgicp_resolution": 0.25,
  "vgicp_voxelmap_levels": 2,
  "smoother_lag": 10.0,
  "disable_imu_factor": false
}
```

**`disable_imu_factor` 옵션:**

GPU 모드에서 IMU Factor Graph 사용 시 Z축 지터(수직 진동)가 발생하는 경우, `disable_imu_factor: true`로 설정하면 IMU를 deskewing에만 사용하고 Factor Graph에서는 제외합니다.

| 설정 | IMU 사용 방식 | 권장 상황 |
|-----|-------------|----------|
| `false` (기본) | Factor Graph에 ImuFactor 추가 | IMU 품질이 좋고 캘리브레이션이 정확한 경우 |
| `true` | Deskewing + 초기값 예측에만 사용 | Z축 지터 발생 시, IMU 노이즈가 큰 경우 |

`disable_imu_factor: true` 설정 시 동작:
- IMU 적분으로 포인트클라우드 deskewing 수행
- Factor Graph에는 ImuFactor 대신 등속 제약조건(BetweenFactor) 사용
- CT 모드와 유사한 IMU 활용 방식

##### CT 모드 (Continuous-Time GICP)
- 설정 파일: `config_odometry_ct.json`
- 특징:
  - **Continuous-Time ICP**: 스캔 내 연속적인 모션을 모델링하여 스캔 시작/끝 포즈를 동시 최적화
  - **iVox 기반 Scan-to-Model**: O(1) 최근접 이웃 탐색으로 빠른 매칭
  - **IMU Deskewing**: IMU 적분으로 각 포인트의 정확한 위치 보정
  - **LM 최적화**: Levenberg-Marquardt 최적화로 robust한 수렴

```json
{
  "so_name": "libodometry_estimation_ct.so",
  "use_imu": true,
  "ivox_resolution": 0.5,
  "ivox_min_points_dist": 0.03,
  "ivox_lru_thresh": 40,
  "max_correspondence_distance": 0.5,
  "lm_max_iterations": 12,
  "location_consistency_inf_scale": 1e-2,
  "constant_velocity_inf_scale": 1e-2
}
```

| 파라미터 | 설명 | 권장값 |
|---------|------|--------|
| `use_imu` | IMU deskewing 활성화 | `true` |
| `ivox_resolution` | iVox 복셀 크기 (m) | 0.3~0.5 |
| `ivox_min_points_dist` | 복셀 내 최소 점 간격 (m) | 0.03 |
| `ivox_lru_thresh` | LRU 캐시 임계값 (스캔 수) | 40 |
| `max_correspondence_distance` | 최대 대응점 거리 (m) | 0.5~1.0 |
| `lm_max_iterations` | LM 최대 반복 횟수 | 10~15 |
| `location_consistency_inf_scale` | 위치 일관성 가중치 | 1e-2 |
| `constant_velocity_inf_scale` | 등속 모션 가중치 | 1e-2 |

**GPU 모드 vs CT 모드 선택 가이드:**

| 특성 | GPU 모드 | CT 모드 |
|-----|----------|---------|
| 하드웨어 요구 | CUDA GPU 필수 | CPU만으로 동작 |
| 빠른 회전 대응 | 양호 | 우수 (연속 시간 모델링) |
| 연산 속도 | 빠름 (GPU 병렬화) | 중간 (CPU 기반) |
| 메모리 사용 | GPU 메모리 사용 | 시스템 메모리만 사용 |
| IMU 의존도 | Factor Graph에서 사용 | Deskewing + 초기값 예측에 사용 |

#### 2. Preprocessing (전처리)
LiDAR 포인트클라우드에 대한 전처리를 수행합니다.

- 설정 파일: `config_preprocess.json`
- 처리 순서:
  1. **Intensity 필터링**: 저반사율 포인트 제거
  2. **다운샘플링**: 복셀 그리드 기반 샘플링
  3. **거리 필터링**: 최소/최대 거리 범위 외 포인트 제거
  4. **Cropbox 필터링**: 특정 영역 내 포인트 제거 (선택적)
  5. **Outlier 제거**: 통계적 이상치 제거 (선택적)

```json
{
  "distance_near_thresh": 0.3,
  "distance_far_thresh": 100.0,
  "downsample_resolution": 0.1,
  "enable_intensity_filter": true,
  "intensity_min_thresh": 10.0,
  "enable_outlier_removal": false
}
```

| 파라미터 | 설명 | 권장값 |
|---------|------|--------|
| `distance_near_thresh` | 최소 거리 (m) | 0.3~1.0 |
| `distance_far_thresh` | 최대 거리 (m) | 50~100 |
| `downsample_resolution` | 다운샘플링 복셀 크기 (m) | 0.05~0.15 |
| `enable_intensity_filter` | Intensity 필터 활성화 | `true` / `false` |
| `intensity_min_thresh` | 최소 intensity 임계값 | 센서 의존적 |
| `enable_outlier_removal` | 통계적 이상치 제거 | `false` (성능 영향) |

**Intensity 필터링:**

저반사율(low intensity) 포인트는 일반적으로 노이즈이거나 신뢰도가 낮은 측정값입니다. `intensity_min_thresh` 값 미만의 포인트를 제거하여 맵 품질을 개선할 수 있습니다. 적절한 임계값은 LiDAR 센서 특성에 따라 다르므로 실험을 통해 결정해야 합니다.

#### 3. Sub Mapping (서브맵 생성)
연속된 키프레임들을 묶어 서브맵을 생성합니다.

- 설정 파일: `config_sub_mapping_gpu.json`
- 현재 설정:
  - **키프레임 전략**: 이동 거리 기반 (0.5m 이동 또는 1.57rad 회전)
  - **서브맵 크기**: 최대 15개 키프레임
  - **Registration**: VGICP-GPU
  - **다운샘플링**: 0.08m 복셀 해상도

```json
{
  "max_num_keyframes": 15,
  "keyframe_update_interval_trans": 0.5,
  "registration_error_factor_type": "VGICP_GPU",
  "submap_downsample_resolution": 0.08
}
```

#### 4. Global Mapping (전역 최적화)
서브맵 간 관계를 최적화하고 루프 클로저를 적용합니다.

- 설정 파일: `config_global_mapping_gpu.json`
- 현재 설정:
  - **IMU 사용**: 비활성화 (IndeterminantLinearSystemException 방지)
  - **Between Factor**: 비활성화
  - **Registration**: VGICP-GPU
  - **Implicit Loop**: 100m 이내 서브맵 간 자동 루프 검출

```json
{
  "enable_imu": false,
  "create_between_factors": false,
  "registration_error_factor_type": "VGICP_GPU",
  "max_implicit_loop_distance": 100.0
}
```

### 확장 모듈 (Extension Modules)

`config_ros.json`에서 확장 모듈을 활성화/비활성화할 수 있습니다:

#### 1. ScanContext Loop Detector
```json
"extension_modules": ["libscan_context_loop_detector.so"]
```
- 역할: ScanContext 기반 루프 클로저 검출
- 설정: `config_scan_context.json`
- 현재 설정:
  - **센서 높이**: 0.97m (지면 추정용)
  - **검색 범위**: 30m, FOV 120°
  - **매칭 임계값**: 0.30 (낮을수록 엄격)
  - **Odometry 기반 지면 추정**: 활성화 (핸드헬드 시나리오용)

```json
{
  "lidar_height": 0.97,
  "sc_dist_threshold": 0.30,
  "use_odometry_ground": true
}
```

#### 2. RGB Colorizer
```json
"rgb_colorizer_enabled": true,
"rgb_image_topic": "/spadi/rgb_image/image_raw/compressed"
```
- 역할: 카메라 이미지를 이용한 포인트 클라우드 컬러화
- 설정: `config_sensors.json`의 카메라 파라미터
- 기능:
  - 카메라 FOV 내 포인트만 RGB 색상 적용
  - Motion compensation (LiDAR-Camera 시간차 보정)
  - Z-buffer 기반 가려짐 처리
  - 종료 시 서브맵별 RGB 포인트 저장

#### 3. Gravity Estimator
```json
"gravity_estimator_enabled": true
```
- 역할: IMU 데이터 기반 중력 방향 추정
- 설정: `config_gravity_estimator.json`
- 초기화 시 수평 정렬에 사용

#### 4. IMU Validator (Headless)
```json
"extension_modules": ["libimu_validator_headless.so"]
```
- 역할: IMU-LiDAR 캘리브레이션 품질 검증
- 출력 항목:
  - 각속도 오차 (LiDAR 오도메트리 vs IMU 자이로스코프)
  - 축별 오차 (X, Y, Z) - 좌표계 정렬 확인용
  - 중력 정렬 오차
  - IMU 시간 오프셋 추정값
  - NID 점수 (0.5 미만이 양호)

#### 5. Memory Monitor
```json
"extension_modules": ["libmemory_monitor.so"]
```
- 역할: 메모리 사용량 모니터링
- 로그에 주기적으로 메모리 사용량 출력

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

### IMU 시간 오프셋 캘리브레이션

IMU와 LiDAR 센서는 각각 독립적인 클럭을 사용하며, 드라이버 처리 지연 등으로 인해 타임스탬프 간 오프셋이 발생할 수 있습니다. 이 오프셋은 **실행하여 측정**해야 합니다.

#### 측정 방법

1. `config_ros.json`의 `extension_modules`에 `libimu_validator_headless.so` 추가
2. rosbag 재생 또는 실시간 데이터 수집
3. 로그에서 `Estimated IMU time offset` 값 확인

```
[imu_valid] [info] Estimated IMU time offset: -0.012 sec
```

4. `config_ros.json`에 측정된 값 적용:

```json
{
  "glim_ros": {
    "imu_time_offset": -0.012  // 음수 = IMU가 LiDAR보다 빠름
  }
}
```

#### 원리

- IMU Validator는 LiDAR 오도메트리 각속도와 IMU 자이로스코프 각속도의 **상호상관(cross-correlation)**을 분석
- 두 신호의 상관관계가 최대가 되는 시간차가 오프셋 값
- 같은 센서 조합에서는 오프셋이 거의 일정하므로, 한번 측정 후 재사용 가능

#### IMU 노이즈 파라미터

`config_sensors.json`에서 IMU 노이즈 특성을 설정합니다:

```json
{
  "sensors": {
    "imu_acc_noise": 0.5,    // 가속도계 노이즈 (m/s²)
    "imu_gyro_noise": 0.5    // 자이로스코프 노이즈 (rad/s)
  }
}
```

IMU Validator의 `Angular velocity error`가 높으면 (> 0.05 rad/s) `imu_gyro_noise` 값을 높여야 합니다.

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

#### 궤적 파일 (TUM format)

| 파일 | 설명 |
|------|------|
| `traj_lidar.txt` | 최적화된 LiDAR 궤적 (루프 클로저 적용) |
| `traj_imu.txt` | 최적화된 IMU 궤적 |
| `odom_lidar.txt` | Odometry 궤적 (루프 클로저 미적용) |
| `odom_imu.txt` | Odometry IMU 궤적 |

#### 포인트클라우드 맵

| 파일 | 설명 |
|------|------|
| `raw_map.pcd` | 전체 원본 포인트 맵 (대용량) |
| `submap_map.pcd` | 서브맵 기반 포인트 맵 |
| `downsampled_map.pcd` | 다운샘플링된 맵 (`map_downsample_resolution` 적용) |

#### 그래프 및 최적화

| 파일 | 설명 |
|------|------|
| `graph.txt` | 팩터 그래프 메타데이터 (서브맵 수, 프레임 수) |
| `graph.bin` | 팩터 그래프 바이너리 데이터 |
| `values.bin` | GTSAM 최적화 결과 (포즈, 속도, IMU 바이어스) |
| `profiling_stats.txt` | 모듈별 연산 시간 통계 |

#### 서브맵 폴더 (`000000/`, `000001/`, ...)

각 서브맵 폴더에는 다음 파일이 저장됩니다:

| 파일 | 설명 |
|------|------|
| `data.txt` | 서브맵 메타데이터 및 키프레임 정보 |
| `points_compact.bin` | 서브맵 포인트클라우드 (바이너리) |
| `covs_compact.bin` | 포인트별 공분산 매트릭스 |
| `imu_rate.txt` | IMU 레이트 궤적 (고주파) |

#### 설정 백업

| 폴더 | 설명 |
|------|------|
| `config/` | 실행 시 사용된 모든 설정 파일 복사본 |
