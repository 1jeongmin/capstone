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
filter0.params.lower_angle: -2.9671  # -170°
filter0.params.upper_angle:  2.9671  #  170°
```