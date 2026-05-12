# capstone

한정민

---

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `slam_launch` | SLAM 매핑 / 로컬라이제이션 / Nav2 launch 파일 및 설정 |
| `scout_ros2` | Scout Mini 로봇 베이스 드라이버 |
| `sllidar_ros2` | SLAMTEC RPLiDAR 드라이버 |
| `ros_wit_imu_node` | WIT IMU 드라이버 |
| `ugv_sdk` | AgliX UGV SDK |
| `yolo_detection` | YOLOv8 기반 객체 탐지 노드 |

---

## 변경 이력

### 2026-05-08 — LiDAR 후면 20° 블라인드 스팟 필터 적용

**배경**
SLAM 매핑 및 로컬라이제이션 시 로봇 후면 구조물이 라이다에 잡혀 노이즈가 발생하는 문제를 해결하기 위해 후면 20° 영역을 소프트웨어 필터로 차단.

**수정 파일**

- `src/slam_launch/config/lidar_filter.yaml` ← **신규 추가**
  - `laser_filters` 패키지의 `LaserScanAngularBoundsFilter` 사용
  - 통과 범위: -170° ~ +170° (후면 ±10° 차단)

- `src/slam_launch/launch/slam_online.launch.py`
  - `scan_to_scan_filter_chain` 노드 추가 (`/scan` → `/scan_filtered`)

- `src/slam_launch/launch/localization.launch.py`
  - `scan_to_scan_filter_chain` 노드 추가 (`/scan` → `/scan_filtered`)

- `src/slam_launch/config/mapper_params_online_async.yaml`
  - `scan_topic: /scan` → `scan_topic: /scan_filtered`

- `src/slam_launch/config/mapper_params_localization.yaml`
  - `scan_topic: /scan` → `scan_topic: /scan_filtered`

**토픽 흐름**
```
sllidar_node → /scan → laser_filter_node → /scan_filtered → slam_toolbox
```

**각도 조정 방법**
`src/slam_launch/config/lidar_filter.yaml` 의 `lower_angle` / `upper_angle` 값만 수정

```yaml
filter1:
  params:
    lower_angle: -2.9671  # -170°
    upper_angle:  2.9671  #  170°
```

---

### 2026-05-12 — Nav2 AMCL 로컬라이제이션 구성 및 포트 정리

**배경**
기존 slam_toolbox 기반 로컬라이제이션 외에 Nav2 AMCL 기반 로컬라이제이션 파이프라인을 새로 구성.
라이다·IMU 포트 혼선, nav2 파라미터 누락, lifecycle 자동 활성화 실패 등 다수 문제 해결.

**디바이스 포트 확정**

| 포트 | 칩셋 | 장치 |
|---|---|---|
| `/dev/ttyUSB0` | CH340 | WIT IMU (WT901) |
| `/dev/ttyUSB1` | CP2102 | SLAMTEC RPLidar |

**추가/수정 파일**

- `src/slam_launch/launch/amcl_localization.launch.py` ← **신규 추가**
  - Nav2 AMCL 기반 로컬라이제이션 통합 런치
  - robot_state_publisher, joint_state_publisher, lidar(`/dev/ttyUSB1`), laser_filter, EKF, nav2 localization, RViz 포함

- `src/slam_launch/rviz/nav2_localization.rviz` ← **신규 추가**
  - Map 디스플레이 QoS: Reliability=Reliable, Durability=Transient Local
  - 시작 시 map_server 발행 전에 RViz가 올바른 QoS로 구독하여 맵 자동 수신

- `src/slam_launch/config/nav2_params.yaml`
  - `scan_topic: /scan_filtered_filtered` → `/scan_filtered` (AMCL 토픽 오타 수정)
  - `yaml_filename: ""` → 맵 경로 명시
  - costmap scan 토픽 `/scan` → `/scan_filtered`
  - `lifecycle_manager_localization` 설정 추가 (autostart, node_names, bond_timeout)
  - AMCL `set_initial_pose: true` 추가 (시작 시 map→odom TF 즉시 발행)

- `src/slam_launch/launch/slam_online.launch.py`
  - 라이다 포트 `/dev/ttyUSB0` → `/dev/ttyUSB1`

- `src/slam_launch/config/mapper_params_localization.yaml`
  - `use_sim_time: true` → `false`

**실행 순서 (Nav2 AMCL 로컬라이제이션)**

```bash
# 전제: CAN 인터페이스 활성화
sudo ip link set can0 up type can bitrate 500000

# Terminal 1 — 로봇 베이스 (EKF가 TF 담당하므로 pub_tf=false)
ros2 launch scout_base scout_mini_base.launch.py pub_tf:=false

# Terminal 2 — IMU (/dev/ttyUSB0)
ros2 launch wit_imu_driver wit901.launch.py

# Terminal 3 — AMCL 로컬라이제이션 + RViz 자동 실행
ros2 launch slam_launch amcl_localization.launch.py map:=/home/user/map/scout_map.yaml

# Terminal 4 (선택) — Nav2 내비게이션
ros2 launch slam_launch nav2.launch.py
```

**토픽 흐름**
```
sllidar_node(/dev/ttyUSB1) → /scan → laser_filter → /scan_filtered → AMCL
wit_imu_node(/dev/ttyUSB0) → /imu/data_raw ↘
scout_base → /odom                           → EKF → odom→base_link TF
                                               AMCL → map→odom TF
```

**주요 해결 사항**
- AMCL scan_topic 오타로 인한 lifecycle 활성화 연쇄 실패 → map 미발행 문제 수정
- RViz QoS 불일치(Volatile vs Transient Local)로 맵 미표시 문제 → rviz config 파일로 해결
- AMCL 초기 위치 미설정으로 map 프레임 미생성 → `set_initial_pose: true`로 해결

**트러블슈팅 — RViz에서 맵이 안 보일 때**

nav2 map_server는 `/map`을 `Transient Local` QoS로 **한 번만** 발행한다.
RViz가 잘못된 QoS(`Volatile`)로 먼저 구독하면 연결이 맺어지지 않아 맵을 수신하지 못한다.
rviz config 파일로 기동 시 해결되지만, 그래도 맵이 안 보이면 아래 명령으로 map_server를 강제 재발행한다.

```bash
ros2 lifecycle set /map_server deactivate && ros2 lifecycle set /map_server activate
```

> **원인 요약**: publisher `Transient Local` + subscriber `Volatile` → DDS 연결 미수립 → 재구독해도 캐시 재전송 없음
> map_server를 재활성화하면 새로 발행되어 RViz가 수신 가능해짐

---

### 2026-05-12 — lidar_filter.yaml 파라미터 포맷 수정

**배경**
`scan_to_scan_filter_chain` 노드 실행 시 필터가 동작하지 않는 문제 발견.
`/scan_filtered` 토픽의 `angle_min/max`가 여전히 ±180°로 출력되어 필터링이 전혀 적용되지 않고 있었음.

**원인**
ROS2 `laser_filters` 패키지의 `FilterChain`은 파라미터를 `filter1`, `filter2`, ... (1부터 시작) 의 **nested YAML** 형식으로 읽음.
기존 파일은 `filter0.name`, `filter0.type` 방식의 flat 포맷을 사용하고 있어 파라미터가 무시됨.

**수정 파일**

- `src/slam_launch/config/lidar_filter.yaml`
  - flat 포맷(`filter0.name: ...`) → nested 포맷(`filter1: {name, type, params}`)으로 변경
  - 인덱스 `filter0` → `filter1` 로 변경

```yaml
# 수정 전 (동작 안 함)
filter0.name: rear_blind_spot
filter0.type: laser_filters/LaserScanAngularBoundsFilter
filter0.params.lower_angle: -2.9671
filter0.params.upper_angle:  2.9671

# 수정 후 (정상 동작)
filter1:
  name: rear_blind_spot
  type: laser_filters/LaserScanAngularBoundsFilter
  params:
    lower_angle: -2.9671
    upper_angle:  2.9671
```