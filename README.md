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