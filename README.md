# 🤖 자율주행 택배 로봇 (Autonomous Delivery Robot)

> **캡스톤디자인 프로젝트** — 자율주행 모바일 로봇이 건물 내부를 스스로 주행하고,
> **엘리베이터를 직접 탑승**하여 여러 층의 목적지까지 택배를 자동 배달·회수하는 시스템

자율주행 플랫폼(**Scout Mini**)과 4축 로봇팔(**OpenMANIPULATOR-X**)을 결합한 **모바일 매니퓰레이터**로,
사람의 개입 없이 `픽업 → 엘리베이터 탑승 → 층간 이동 → 호실 앞 정렬 → 배달 → 복귀`의 전체 배달 시나리오를 자율 수행합니다.

---

## 📑 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [전체 시스템 아키텍처](#2-전체-시스템-아키텍처)
3. [기술 스택](#3-기술-스택)
4. [동작 시나리오](#4-동작-시나리오)
5. [서브시스템 ① 자율주행 (Scout Mini)](#5-서브시스템--자율주행-scout-mini)
6. [서브시스템 ② 로봇팔 (OpenMANIPULATOR-X)](#6-서브시스템--로봇팔-openmanipulator-x)
7. [인식 (Perception)](#7-인식-perception)
8. [ROS2 토픽 인터페이스](#8-ros2-토픽-인터페이스)
9. [결과 및 성과](#9-결과-및-성과)
10. [한계 및 향후 과제](#10-한계-및-향후-과제)
11. [개발 환경 / 디렉토리 구성](#11-개발-환경--디렉토리-구성)

---

## 1. 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **프로젝트명** | 자율주행 택배 로봇 (엘리베이터 자율 탑승 배달 시스템) |
| **유형** | 캡스톤디자인 |
| **목표** | 건물 내부 여러 층에 걸친 택배의 무인 자동 배달/회수 |
| **핵심 차별점** | 로봇이 **엘리베이터 버튼을 직접 인식·조작**하여 층간 이동을 자율 수행 |
| **플랫폼** | Scout Mini (자율주행) + OpenMANIPULATOR-X 4-DOF (로봇팔) |
| **미들웨어** | ROS2 Humble (Ubuntu 22.04) |

**해결하고자 한 문제**
- 기존 실내 배송 로봇은 대부분 **단일 층** 내에서만 동작 → 엘리베이터라는 물리적 장벽을 넘지 못함
- 본 프로젝트는 로봇팔로 엘리베이터 호출 버튼·층수 버튼을 **물리적으로 누르고**, 카메라로 **점등/소등을 확인**하여
  사람 없이 층간 이동까지 완결되는 풀스택 배달 파이프라인을 구현

---

## 2. 전체 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                         자율주행 택배 로봇                            │
│                                                                       │
│   ┌───────────────────────────┐      ┌────────────────────────────┐  │
│   │   자율주행 (Scout Mini)    │◀────▶│   로봇팔 (OpenMANIPULATOR) │  │
│   │                           │ ROS2 │                            │  │
│   │ • SLAM (slam_toolbox)     │ 토픽 │ • 해석적 IK (수식 직접)    │  │
│   │ • AMCL 위치추정 (Nav2)    │      │ • YOLOv8 버튼 인식         │  │
│   │ • Waypoint 경로 추종      │      │ • EasyOCR 호수 인식        │  │
│   │ • AEB / 장애물 회피       │      │ • SVM 접촉 감지            │  │
│   │ • EKF (IMU + Odom 융합)   │      │ • 픽업/배달 모션           │  │
│   │ • Mission Manager         │      │ • 엘리베이터 버튼 조작     │  │
│   └─────────────┬─────────────┘      └──────────────┬─────────────┘  │
│                 │                                    │                │
│          ┌──────┴──────┐                  ┌──────────┴─────────┐      │
│          │ 2D LiDAR    │                  │ RealSense D435     │      │
│          │ IMU         │                  │ (RGB-D 카메라)     │      │
│          └─────────────┘                  └────────────────────┘      │
└─────────────────────────────────────────────────────────────────────┘
```

두 서브시스템은 **ROS2 토픽**으로 느슨하게 결합됩니다. 자율주행 측의 `Mission Manager`가
전체 미션을 오케스트레이션하고, 정지 지점에 도착할 때마다 로봇팔에게 작업을 트리거합니다.

---

## 3. 기술 스택

| 분야 | 사용 기술 |
|------|-----------|
| **로봇 플랫폼** | Scout Mini (4WD 모바일 베이스), OpenMANIPULATOR-X (4-DOF) |
| **자율주행 SLAM** | slam_toolbox (online async) |
| **위치추정** | Nav2 AMCL, EKF (`robot_localization`, IMU + Odometry 융합) |
| **경로 추종** | 자체 구현 Waypoint Path Follower (Pure-Pursuit 계열) |
| **안전** | AEB(자동 비상 제동), E-Stop, LiDAR 기반 장애물 회피 |
| **센서** | 2D LiDAR, IMU (WIT / EBIMU), Intel RealSense D435 (RGB-D) |
| **버튼 인식** | YOLOv8 (mAP50 **98.7%**), YOLO-seg + EasyOCR |
| **호수 인식** | EasyOCR |
| **역기구학(IK)** | 해석적 IK (수식 직접 유도, MoveIt2 불필요) |
| **접촉 감지** | SVM (RBF 커널, 5-fold CV F1 **0.812**) |
| **미들웨어** | ROS2 Humble |
| **시뮬레이션** | NVIDIA Isaac Sim 5.1.0 |
| **언어 / OS** | Python 3.10 / Ubuntu 22.04 |

---

## 4. 동작 시나리오

```
택배기사 앱 (목표 층수 입력)
        │  ROS2 /target_floor
        ▼
┌─[픽업]──────────────────────────────────────────┐
│ 로봇팔: 책상 위 박스 인식(YOLO) → 집기            │
│        → Scout Mini 바구니에 적재                 │
│        /robot_status = PICKUP_DONE                │
└──────────────────────────────────────────────────┘
        ▼
┌─[엘리베이터 탑승]────────────────────────────────┐
│ Scout Mini: 엘리베이터 앞으로 자율주행            │
│ 로봇팔: UP/DOWN 버튼 인식 → 누르기                │
│        → 버튼 점등 확인 → 도착 대기(소등 감지)    │
│        → 탑승 후 목표 층수 버튼 누르기            │
│        /robot_status = NUMBER_PRESSED             │
└──────────────────────────────────────────────────┘
        ▼
┌─[층간 이동 & 배달]───────────────────────────────┐
│ Scout Mini: 목표 층 도착 → 호실까지 주행          │
│ 로봇팔: 호수 인식(EasyOCR) → /room_number 발행    │
│ Scout Mini: 해당 호실 앞 정렬 → /aligned_ready    │
│ 로봇팔: 바구니에서 박스 집기 → 목적지에 배달      │
│        /robot_status = DELIVERY_DONE              │
└──────────────────────────────────────────────────┘
        ▼
┌─[복귀]──────────────────────────────────────────┐
│ Scout Mini + 로봇팔: 엘리베이터로 출발층 복귀     │
└──────────────────────────────────────────────────┘
```

> **5층 통합 미션 예시** (`mission_manager.py`): START 출발 → 523호 배달 → 521호 회수
> → (구간 내 정적/동적 장애물 자동 회피) → GOAL 도착. 정지/재개는 `/mission/pause`로 제어.

---

## 5. 서브시스템 ① 자율주행 (Scout Mini)

> 본 저장소(`1jeongmin/capstone`)의 핵심. ROS2 워크스페이스 `ros2_ws` 기반.

### 5.1 매핑 & 위치추정
- **SLAM**: `slam_toolbox`(online async)로 건물 층별 2D 점유 격자 지도 작성 (`floor12`, `floor5` 등 저장)
- **위치추정**: Nav2 **AMCL** + **EKF** 센서 융합
  - `odom_covariance_fixer` — scout_base가 공분산을 0으로 발행하는 문제를 래핑하여 `/odom_fixed`로 보정
  - EKF가 IMU + 보정된 Odometry를 융합해 안정적인 `map → base_link` 추정 제공

### 5.2 경로 주행
- **Waypoint Recorder** (`sm_waypoint_recorder`): 주행 경로를 일정 간격(기본 0.5 m)으로 기록 → CSV/YAML 저장
- **Path Follower** (`sm_path_follower`): 기록된 waypoint를 추종 (TF `map → base_link` 기반 조향/속도 제어)
- **Mission Manager** (`mission_manager`): 다중 정지 지점·로봇팔 트리거·완료 대기를 관리하는 미션 오케스트레이터
- **Platooning** (`sm_platoon_follower_from_leader`): 리더 추종형 군집 주행 실험 노드

### 5.3 안전 시스템
| 노드 | 기능 |
|------|------|
| `scout_mini_aeb` | 전방 ±20° LiDAR 최소거리 기반 **자동 비상 제동**. `/scout_mini/aeb_scale`(0~1)로 감속 스케일 출력 (≤0.5 m 완전 정지, ~1.4 m부터 감속) |
| `scout_mini_e_stop` | 감지 박스(전방 0.2~0.8 m, 좌우 ±0.25 m) 내 장애물 시 `/scout_mini/e_stop` 발행 |
| `scout_mini_obstacle_avoidance` | 좌/우 점유도 비교로 회피 조향 힌트(`/scout_mini/oa_steer`) 제공 |
| 후방 LiDAR 사각 필터 | 본체 후방 사각지대 오감지 제거 필터 |

---

## 6. 서브시스템 ② 로봇팔 (OpenMANIPULATOR-X)

> 로봇팔 제어 코드는 별도 저장소 [`elevator-button-robot`](https://github.com/uihyeong/elevator-button-robot)
> 및 워크스페이스 `colcon_ws`에서 관리. 아래는 핵심 요약.

### 6.1 해석적 역기구학 (Analytical IK)
MoveIt2 없이 4-DOF 관절각을 **수식으로 직접 유도**하여 계산 (경량·고속).

목표 위치 $(X, Y, Z)$ 에 대해:
- **θ₁** (베이스 회전): $\theta_1 = \text{atan2}(Y, X)$
- **손목 위치**: end-effector에서 $L_4$ 제거 → $r_w = \sqrt{X^2+Y^2} - L_4,\ z_w = Z$
- **코사인 법칙**으로 elbow-up / elbow-down 두 해 계산 → 관절 한계를 통과하는 첫 해 사용
- **θ₄**: 버튼에 **수평 접근**하도록 $\theta_4 = -(\theta_2 + \theta_3)$ 구속

| 링크 | 값 |
|------|-----|
| $L_1$ (base→joint2) | 0.0595 m |
| $L_2$ (joint2→joint3) | ≈ 0.1302 m |
| $L_3$ (joint3→joint4) | 0.124 m |
| $L_4$ (joint4→EE) | 0.126 m |

### 6.2 픽업 / 배달 모션
- **Joint 직접 지령**(절대 관절각) + **XYZ→IK**(좌표 입력) 혼용
- 픽업 14스텝 / 배달 15스텝 시퀀스로 `책상 → 바구니 → 목적지` 이송
- 위치 변경 시 Joint 스텝은 재실측, XYZ 스텝은 좌표만 수정하면 IK가 관절각 자동 산출

### 6.3 엘리베이터 버튼 상태 머신
```
IDLE → UPDOWN_READY → UPDOWN_PRESS → WAIT → NUMBER_READY → NUMBER_PRESS → DONE
```
- UP/DOWN 버튼 인식 → 누르기 → **점등 확인**(HSV) → **소등 감지 시 도착** 판단 → 층수 버튼 누르기
- UP/DOWN 연속 3회 실패 시 `NEED_REPOSITION` 발행 → Scout Mini에 재정렬 요청

### 6.4 접촉 감지 (Contact Detection)
- **방법 A — SVM (권장)**: FSR406 실측 데이터(19,664 슬라이딩 윈도우)로 학습한 RBF-SVM
  - 160차원 특징(최근 10샘플 × joint velocity/effort_delta/diff)
  - 5-fold CV **F1 0.812 ± 0.006**
  - 버튼 누르는 중·홈 복귀 중 감지 차단, 연속 3윈도우 × prob ≥ 0.80 시에만 접촉 확정
- **방법 B — Effort Threshold**: joint3 effort 편차 기반 단순 판정

---

## 7. 인식 (Perception)

| 대상 | 방법 | 성능 |
|------|------|------|
| **UP/DOWN 버튼** | YOLOv8 객체 검출 | **mAP50 98.7%** |
| **층수(숫자) 버튼** | YOLO-seg(분할) + EasyOCR | — |
| **호수(방 번호)** | EasyOCR | — |
| **버튼 점등/소등** | HSV 색공간 판정 | — |
| **박스(택배) 검출** | YOLOv8 | — |

- 카메라: Intel RealSense **D435** (RGB-D), end-effector(link5)에 장착, static TF로 좌표 연결
- 버튼 깊이값 보정: 카메라 TF 높이 오차를 `Z - 0.031 m`(unified) 등으로 실측 보정

---

## 8. ROS2 토픽 인터페이스

### 자율주행 ↔ 로봇팔 (서브시스템 간)
| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/target_floor` | 주행 → 팔 | `Int32` | 목표 층수 (음수=지하, 예 -2=B2) |
| `/elevator_ready` | 주행 → 팔 | `Bool` | 엘리베이터 안 버튼 앞 정지 완료 |
| `/aligned_ready` | 주행 → 팔 | `Bool` | 호실 앞 정렬 완료 |
| `/robot_status` | 팔 → 주행 | `String` | 상태값(아래) |
| `/room_number` | 팔 → 주행 | `String` | 인식된 호수 (예 "531") |

### 미션 제어 (자율주행 내부)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mission/pause` | `Bool` | true면 path_follower 정지 (정지/재개 제어) |
| `/mission/state` | `String` | 현재 미션 상태 모니터링 |
| `/start_delivery` `/delivery_done` | `Bool` | 배달 시퀀스 트리거 / 완료 |
| `/start_recover` `/recover_done` | `Bool` | 회수 시퀀스 트리거 / 완료 |
| `/scan` `/odom_fixed` `/scout_mini/aeb_scale` `/scout_mini/e_stop` | — | 센서·안전 신호 |

### `/robot_status` 상태값
`MOVING` · `PICKUP_DONE` · `BUTTON_PRESSED` · `ELEVATOR_ARRIVED` ·
`NUMBER_PRESSED` · `DELIVERY_DONE` · `NEED_REPOSITION` · `FAILED`

---

## 9. 결과 및 성과

✅ **엘리베이터 자율 탑승 배달 파이프라인 전 구간 통합 시연 성공**
- `픽업 → 엘리베이터 호출 → 탑승 → 목표 층 이동 → 호실 정렬 → 배달 → 복귀`까지 자율 수행

✅ **인식 성능**
- UP/DOWN 버튼 YOLOv8 **mAP50 98.7%**
- 호수/층수 버튼 EasyOCR + YOLO-seg로 안정적 판독

✅ **제어 성과**
- MoveIt2 없이 **해석적 IK**로 경량·실시간 관절 제어 구현
- **SVM 접촉 감지 F1 0.812** — 단순 임계값 방식 대비 오인식 대폭 감소
  (버튼 누름·홈 복귀 구간 차단 + 연속 윈도우 확률 조건)

✅ **자율주행 성과**
- slam_toolbox 매핑 + AMCL/EKF 위치추정으로 다층 실내 환경 주행
- AEB·E-Stop·장애물 회피로 정적/동적 장애물 대응
- Mission Manager로 다중 정지 지점 배달/회수 시나리오 자동화

✅ **시뮬레이션 검증**
- NVIDIA Isaac Sim 5.1.0에서 Scout Mini + 로봇팔 합체 씬으로 사전 검증 후 실로봇 이식

---

## 10. 한계 및 향후 과제

- **위치 의존성**: Joint 직접 지령 스텝은 로봇 설치 위치가 바뀌면 재실측 필요 → XYZ→IK 비중 확대 여지
- **버튼 깊이 오차**: 카메라 TF 높이 오차를 수동 보정값으로 대응 → 캘리브레이션 자동화 필요
- **엘리베이터 통신 부재**: 버튼을 물리적으로 누르고 점등/소등을 영상으로 추정 → 실제 엘리베이터 연동(BLE/IoT) 시 신뢰도 향상 가능
- **Scout Mini ↔ 팔 통합**: 일부 이동 로직(`scout.py`)은 뼈대 상태로 추가 통합 여지

---

## 11. 개발 환경 / 디렉토리 구성

**개발 환경**
- OS: Ubuntu 22.04 / ROS2: Humble / Python: 3.10
- 하드웨어: Scout Mini, OpenMANIPULATOR-X + U2D2, Intel RealSense D435, 2D LiDAR, IMU
- 시뮬레이션: NVIDIA Isaac Sim 5.1.0

**관련 저장소 / 워크스페이스**

| 구성 | 설명 |
|------|------|
| **`ros2_ws`** | 자율주행 워크스페이스 — Scout Mini 제어, SLAM, Nav2, 경로 추종, 안전 노드 |
| **`colcon_ws`** | 로봇팔 하드웨어 워크스페이스 — OpenMANIPULATOR-X, DynamixelSDK |
| [`elevator-button-robot`](https://github.com/uihyeong/elevator-button-robot) | 로봇팔 제어 로직 — IK, 버튼/호수 인식, 픽업/배달, 접촉 감지 |

**`ros2_ws/src` 주요 패키지**
```
ros2_ws/src/
├── scout_ros2 / scout_control     # Scout Mini 본체 제어
├── scout_mini_nav2                # Nav2 AMCL 위치추정
├── slam_launch                    # SLAM(slam_toolbox) + EKF + 지도
├── scout_mini_tools               # ★ 핵심: path follower, mission manager,
│                                  #   AEB, e-stop, 장애물 회피, waypoint recorder
├── sllidar_ros2 / ugv_sdk         # LiDAR / 하드웨어 SDK
├── realsense-ros                  # RealSense 카메라 드라이버
└── ebimu_pkg / wit_ros2_imu       # IMU
```

---

<p align="center"><em>2026 캡스톤디자인 — 자율주행 택배 로봇 팀</em></p>
