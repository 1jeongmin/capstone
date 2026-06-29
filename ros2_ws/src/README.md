# ros2_ws/src — 자율주행 워크스페이스 소스

Scout Mini 자율주행 서브시스템의 ROS2 패키지 소스입니다.

## 팀 자체 작성 패키지 (이 저장소에 포함)

| 패키지 | 설명 |
|--------|------|
| `scout_mini_tools` | **핵심 주행 로직** — path follower, mission manager, AEB, e-stop, 장애물 회피, waypoint recorder, 군집주행 |
| `slam_launch` | SLAM(slam_toolbox) + EKF 런치/설정 + 작성된 층별 지도(maps) |
| `scout_mini_nav2` | Nav2 AMCL 위치추정 런치/파라미터 |
| `scout_control` | Scout Mini ros2_control / twist_mux 설정 |
| `ebimu_pkg` | EBIMU IMU 드라이버 |
| `wit_ros2_imu` | WIT IMU 드라이버 |

## 서드파티 의존성 (별도 클론 필요 — 이 저장소에 미포함)

빌드 전 아래 패키지를 `ros2_ws/src/`에 함께 클론하세요.

```bash
cd ~/ros2_ws/src

# Scout Mini 본체 드라이버
git clone https://github.com/westonrobot/scout_ros2.git
git clone https://github.com/westonrobot/ugv_sdk.git

# LiDAR
git clone https://github.com/Slamtec/sllidar_ros2.git

# SLAM
git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git

# RealSense 카메라
git clone https://github.com/IntelRealSense/realsense-ros.git
```

## 빌드

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 주요 실행 예시

```bash
# SLAM 매핑
ros2 launch slam_launch slam_online.launch.py

# 위치추정(AMCL) 기반 주행
ros2 launch scout_mini_nav2 localization_launch.py

# Waypoint 기록 / 경로 추종
ros2 run scout_mini_tools sm_waypoint_recorder
ros2 run scout_mini_tools sm_path_follower

# 안전 노드
ros2 run scout_mini_tools scout_mini_aeb
ros2 run scout_mini_tools scout_mini_e_stop

# 통합 미션
ros2 run scout_mini_tools mission_manager
```
