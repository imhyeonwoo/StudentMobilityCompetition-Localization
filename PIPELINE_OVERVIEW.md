<!--
File: imu_preproc_ihw/PIPELINE_OVERVIEW.md
Description: End‑to‑end overview of the GPS+IMU localization stack
             composed of imu_preproc_ihw (IMU preprocessing) and
             gps_imu_fusion_ihw (2D EKF).
Author(s): ihw
-->

# GPS + IMU Localization Pipeline Overview

본 문서는 **IMU 전처리(`imu_preproc_ihw`) + EKF(`gps_imu_fusion_ihw`)** 를 이용해
차량의 **2D 위치(x,y)**, **속도(vx,vy)**, **글로벌 요(ψ)** 를 추정하는 전체 파이프라인을 설명합니다.

센서 입력:  
- Ouster에서 제공하는 **/ouster/imu (sensor_msgs/Imu)**  
- u-blox GNSS에서 제공하는 **/ublox_gps_node/fix (NavSatFix)**  

중간 처리:  
- 위경도 → 기준평면 변환(`gps_global_planner/gps_to_local_cartesian`)  
- IMU 신호 필터링/바이어스 제거(`imu_preproc_ihw`)  
- 전처리된 신호 + GPS 위치를 이용한 확장 칼만 필터(`gps_imu_fusion_ihw`)

출력:  
- `/odometry/fusion` (nav_msgs/Odometry)  
- `/global_yaw` (std_msgs/Float32, rad)  
- 선택적 TF 방송 (`reference` → `gps_antenna`)

---

## 목차
- [1. 전체 데이터 흐름](#1-전체-데이터-흐름)
- [2. 좌표계 & 프레임](#2-좌표계--프레임)
- [3. 주요 ROS 토픽](#3-주요-ros-토픽)
- [4. EKF 모델](#4-ekf-모델)
  - [4.1 상태 벡터](#41-상태-벡터)
  - [4.2 입력 & 운동학](#42-입력--운동학)
  - [4.3 선형화 상태전이행렬 A](#43-선형화-상태전이행렬-a)
  - [4.4 입력행렬 B & 과정잡음 Q](#44-입력행렬-b--과정잡음-q)
  - [4.5 GPS 관측모델 H & 측정잡음 R](#45-gps-관측모델-h--측정잡음-r)
- [5. 전처리 알고리즘](#5-전처리-알고리즘)
  - [5.1 자이로 바이어스 추정](#51-자이로-바이어스-추정)
  - [5.2 가속도 바이어스 + LPF](#52-가속도-바이어스--lpf)
  - [5.3 Deadband로 소진동 억제](#53-deadband로-소진동-억제)
  - [5.4 정지 감지 & 바이어스 샘플링](#54-정지-감지--바이어스-샘플링)
  - [5.5 GPS 기반 초기 Yaw 계산](#55-gps-기반-초기-yaw-계산)
  - [5.6 중력 보정에 관한 주의](#56-중력-보정에-관한-주의)
- [6. 파라미터 목록 & 튜닝 전략](#6-파라미터-목록--튜닝-전략)
  - [6.1 imu_preproc_ihw 파라미터](#61-imu_preproc_ihw-파라미터)
  - [6.2 gps_imu_fusion_ihw 파라미터](#62-gps_imu_fusion_ihw-파라미터)
  - [6.3 파라미터 상호작용 매트릭스](#63-파라미터-상호작용-매트릭스)
  - [6.4 단계별 튜닝 워크플로우](#64-단계별-튜닝-워크플로우)
- [7. 실행 절차 (Quick Start)](#7-실행-절차-quick-start)
- [8. EKF 초기 Yaw 힌트 연동](#8-ekf-초기-yaw-힌트-연동)
- [9. Troubleshooting](#9-troubleshooting)
- [10. 확장 로드맵](#10-확장-로드맵)
- [11. 개정 이력](#11-개정-이력)

---

## 1. 전체 데이터 흐름

```
┌───────────────────────────── Sensor Drivers / Upstream Nodes ─────────────────────────────┐
│                                                                                           │
│  /ouster/imu  (100 Hz, raw-ish)    /ublox_gps_node/fix (7 Hz)                             │
│   sensor_msgs/Imu                  sensor_msgs/NavSatFix                                  │
└────────────┬───────────────────────┬──────────────────────────────────────────────────────┘
             │                       │
             │(static TF gps_antenna↔os_sensor, lat/lon ref)                               
             │                       │
             │                       └─► gps_to_local_cartesian  ──► /local_xy (PointStamped)
             │
             └─► **IMU Preprocessor** (새 기능)
                  - axis transform (os_imu→vehicle)
                  - gyro bias remove
                  - accel LPF & bias remove
                  - optional gravity removal
                  - stationary detector & auto bias
                  ↓
                /imu/processed  (sensor_msgs/Imu or custom msg)
                                  (cleaned accel_xy, omega_z)

                                      +                                     
                                      |
                                      ▼
                          **sensor_fusion_node (EKF)**
                          --------------------------------
                          1) 초기화 (GPS x,y & yaw from course)
                          2) Predict  @ IMU 100 Hz
                          3) Update   @ GPS 7 Hz
                          4) Health / gating / timeout
                          5) Publish
                              /odometry/fusion    (nav_msgs/Odometry)
                              /global_yaw          (std_msgs/Float32)
                              TF reference→gps_antenna

```

요약 순서:

1. GNSS → 평면좌표 `/local_xy`.
2. Raw IMU → 전처리(바이어스, LPF, Deadband, 초기 yaw 계산) → `/imu/processed`, `/initial_yaw`.
3. EKF가 `/local_xy` + `/imu/processed` (+ 초기 yaw) 융합.
4. 결과 `/odometry/fusion`, `/global_yaw`, 그리고 선택적 TF.

---

## 2. 좌표계 & 프레임

**참조 평면(reference frame)**  
`ref_lat`, `ref_lon` 을 기준으로 한 사상(구면→평면). 동쪽이 +x, 북쪽이 +y 인 ENU 평면.

**차량 프레임**  
현재 EKF 출력 child_frame=`gps_antenna`. (GPS 안테나 위치 = 차량 기준점으로 근사.)  
*주의:* 실제 차량 Body frame과 GPS 안테나 사이 위치 오프셋/회전이 있으면 추후 레버암 보정 필요.

**IMU 프레임 가정**  
현재 구현은 IMU XY축이 차량 평면과 정렬되어 있다고 가정.  
- IMU X → 차량 전방(+x)  
- IMU Y → 차량 좌측(+y)  
- IMU Z → 천정(+z)  
틀어져 있다면 반드시 **정렬 회전(캘리브레이션 매트릭스)** 를 전처리에서 적용.

---

## 3. 주요 ROS 토픽

| Topic | Msg Type | Producer | Consumer | Rate | 설명 |
|---|---|---|---|---|---|
| `/ouster/imu` | sensor_msgs/Imu | Ouster driver | imu_preproc_ihw | ~100 Hz | Raw accel/gyro (orientation cov=-1) |
| `/ublox_gps_node/fix` | sensor_msgs/NavSatFix | u-blox | gps_to_local_cartesian | ~8 Hz | GNSS lat/lon/alt + cov |
| `/local_xy` | geometry_msgs/PointStamped | gps_to_local_cartesian | imu_preproc_ihw, EKF | ~8 Hz | 평면 좌표 |
| `/imu/processed` | sensor_msgs/Imu | imu_preproc_ihw | EKF | ~100 Hz | bias 제거 & 필터링된 accel_xy, gyro_z |
| `/initial_yaw` | std_msgs/Float32 (latched) | imu_preproc_ihw | EKF | 1회 | GPS 진행방향 기반 초기 ψ(rad) |
| `/odometry/fusion` | nav_msgs/Odometry | EKF | downstream | ~100 Hz | (x,y,vx,vy,yaw) |
| `/global_yaw` | std_msgs/Float32 | EKF | VehicleTFBroadcaster 등 | ~100 Hz | ψ(rad) |
| (opt) `/imu/debug/*` | custom | imu_preproc_ihw | rqt_plot | - | 디버그용 |

---

## 4. EKF 모델

### 4.1 상태 벡터
$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
v_x \\
v_y \\
\psi
\end{bmatrix}
$$
`reference` 평면 기준.

---

### 4.2 입력 & 운동학

입력(제어) 벡터:
$$
\mathbf{u} =
\begin{bmatrix}
a_x^b \\
a_y^b \\
\omega_z
\end{bmatrix}
$$
- $a_x^b$, $a_y^b$: **IMU 바디 XY에서 중력제거 후**의 선형 가속도 *가 이상적이지만*, **현재 구현은 단순히 raw(또는 LPF된) XY 가속도에서 중력 수직성분만 무시한 값**을 사용하고 있습니다. 즉, roll/pitch 가 작고 차량 평면이 수평에 가깝다는 가정입니다.
- $\omega_z$: 자이로 Z rate (bias 제거 후).

### 월드 변환

$$
\begin{bmatrix} a_x \\ a_y \end{bmatrix}
=
R(\psi)
\begin{bmatrix} a_x^b \\ a_y^b \end{bmatrix}, \quad
R(\psi)=\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix}.
$$

---

### 상수 가속 근사

$$
\begin{aligned}
x_{k+1}   &= x_k + v_{x,k}\Delta t + \tfrac{1}{2} a_x \Delta t^2, \\
y_{k+1}   &= y_k + v_{y,k}\Delta t + \tfrac{1}{2} a_y \Delta t^2, \\
v_{x,k+1} &= v_{x,k} + a_x \Delta t, \\
v_{y,k+1} &= v_{y,k} + a_y \Delta t, \\
\psi_{k+1}&= \psi_k + \omega_z \Delta t.
\end{aligned}
$$


---

### 4.3 선형화 상태전이행렬 A

$$
A =
\begin{bmatrix}
1 & 0 & \Delta t & 0        & 0 \\
0 & 1 & 0        & \Delta t & 0 \\
0 & 0 & 1        & 0        & 0 \\
0 & 0 & 0        & 1        & 0 \\
0 & 0 & 0        & 0        & 1
\end{bmatrix}.
$$


---

### 4.4 입력행렬 B & 과정잡음 Q

$$
B(\psi,\Delta t) =
\begin{bmatrix}
\frac{1}{2}\Delta t^2 \cos\psi & -\frac{1}{2}\Delta t^2 \sin\psi & 0 \\
\frac{1}{2}\Delta t^2 \sin\psi &  \frac{1}{2}\Delta t^2 \cos\psi & 0 \\
\Delta t \cos\psi              & -\Delta t \sin\psi              & 0 \\
\Delta t \sin\psi              &  \Delta t \cos\psi              & 0 \\
0                              &  0                              & \Delta t
\end{bmatrix}.
$$


입력잡음 공분산:
$$
\Sigma = \operatorname{diag}(\sigma_a^2,\sigma_a^2,\sigma_\omega^2).
$$


따라서
$$
Q = B \Sigma B^T.
$$


---

### 4.5 GPS 관측모델 H & 측정잡음 R

관측은 위치만:
$$
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0
\end{bmatrix}.
$$


NavSatFix → 평면 오차 표준편차 $\sigma_{gps}$
 동일 가정:
$$
R = 
\begin{bmatrix}
\sigma_{gps}^2 & 0 \\
0 & \sigma_{gps}^2
\end{bmatrix}.
$$


---

## 5. 전처리 알고리즘

### 5.1 자이로 바이어스 추정
정지 상태에서 일정 시간 구간 평균

$$
b_g = \frac{1}{N}\sum_{i=1}^N \omega_{z,i}^{raw}.
$$

보정

$$
\omega_z^{corr} = \omega_z^{raw} - b_g.
$$


### 5.2 가속도 바이어스 + LPF
정지 동안 평균 $b_{a_x},\ b_{a_y}$ 추정 후
$$
a_x' = a_x^{raw} - b_{a_x}, \quad
a_y' = a_y^{raw} - b_{a_y}.
$$


1차 지수형 저역통과:
$$
y[k] = y[k-1] + \alpha (x[k] - y[k-1]),
\quad \alpha = 1 - e^{-2\pi f_c \Delta t}.
$$


### 5.3 Deadband로 소진동 억제
$$
|y| < a_{db} \Rightarrow y=0.
$$

### 5.4 정지 감지 & 바이어스 샘플링
조건(예시):
- $|\omega_z^{raw}| < \text{gyro\_stationary\_thr}$
- $|\|a\| - g| < a_{norm\_thr}$


충족된 연속 구간 길이 > `*_bias_window_sec` 이면 새 바이어스 갱신.

### 5.5 GPS 기반 초기 Yaw 계산
최근 GPS 샘플 2개 이상:
$$
dx = x_n - x_{n-k},\quad dy = y_n - y_{n-k},\quad dt = t_n - t_{n-k}.
$$
속도 $$ v = \frac{\sqrt{dx^2 + dy^2}}{dt} $$ 가 `yaw_speed_thresh` 이상이면
$$
\psi_0 = \mathrm{atan2}(dy, dx).
$$
필요 시 구간 평균/저역통과 적용(`yaw_window`, `yaw_alpha`).  
결과를 `/initial_yaw` 로 **transient_local latched** 발행.

### 5.6 중력 보정에 관한 주의
현재 파이프라인은 **roll/pitch 가 작고 차체가 거의 수평**이라는 전제에서 **XY 가속도만 사용**합니다.  
- 센서 orientation이 기울면 중력의 XY 투영 성분이 속도 드리프트를 유발할 수 있습니다.
- 향후 roll/pitch 추정(또는 외부 AHRS) 도입 후 중력성분 제거 필요.

---

## 6. 파라미터 목록 & 튜닝 전략

### 6.1 imu_preproc_ihw 파라미터

| 이름 | 기본 | 단위 | 설명 | 튜닝 가이드 |
|---|---|---|---|---|
| `gyro_bias_z` | 0 | rad/s | 수동 초기 자이로 바이어스 | 장비별 실측 후 반영 |
| `auto_gyro_bias` | true | bool | 정지시 자동 추정 | 초기 true → 수렴 후 false 가능 |
| `gyro_bias_window_sec` | 10 | s | 바이어스 계산 정지 구간 길이 | 짧으면 민감, 길면 안정 |
| `gyro_stationary_thr` | 0.02 | rad/s | 정지 판단 기준(자이로) | Raw 노이즈 RMS보다 약간 크게 |
| `use_accel` | true | bool | EKF에 accel 전달 여부 | 문제 디버그용으로 false 가능 |
| `acc_lpf_hz` | 1.0 | Hz | accel LPF 컷오프 | 진동 환경 따라 조정 |
| `acc_deadband` | 0.05 | m/s² | 소진동 억제 임계 | 노면 진동 크기 따라 |
| `auto_acc_bias` | true | bool | accel 바이어스 자동 | 정지 확보 어려우면 false |
| `acc_bias_window_sec` | 10 | s | accel 바이어스 추정 윈도 | 짧으면 흔들림, 길면 안정 |
| `compute_initial_yaw` | true | bool | 초기 Yaw 계산/발행 | GNSS 없음 환경이면 false |
| `yaw_speed_thresh` | 0.5 | m/s | yaw 계산 최소 진행속도 | 0.3~1.0 실험 |
| `yaw_window` | 2 | samples | yaw 이동 평균 길이 | GPS 노이즈에 따라 2~5 |

(추가 디버그용: 로그 레벨, 토픽 이름 remap 등은 launch에서 설정.)

---

### 6.2 gps_imu_fusion_ihw 파라미터

`gps_imu_fusion.yaml` 예시:

```yaml
gps_imu_sensor_fusion:
  ros__parameters:
    sigma_a:     0.5        # [m/s²] 과정 잡음 (acc)
    sigma_omega: 0.08       # [rad/s] 과정 잡음 (yaw rate)
    dt_default:  0.01       # [s] IMU dt 이상 시 fallback
    gps_cov:     0.000196   # [m²] GPS 측정 분산 (XY 동일)
    map_frame:   "reference"
    base_frame:  "gps_antenna"
    publish_tf:  true
```

#### 필드 설명 & 영향
- **sigma_a** ↑ → 모델 자유도 ↑ (accel 신호 반영 ↑, 노이즈도 ↑). 너무 크면 위치/속도 떨림. 너무 작으면 GPS 업데이트 사이에 관성 예측이 뻣뻣해 GPS jump 후 재수렴 느림.
- **sigma_omega** ↑ → yaw가 자이로 변화/잡음에 민감. ↓ → yaw drift 발생 시 GPS yaw와 괴리 유지(수렴 느림).
- **gps_cov** ↓(낮은 수치) → GPS를 강하게 믿음(필터가 GPS 좌표에 빨리 달라붙음). ↑ → GPS가 noisy 할 때 예측을 더 유지.
- **dt_default**: IMU 타임스탬프 이상 시 대체. 실제 주파수와 맞추어 설정(100 Hz → 0.01 s).
- **map_frame/base_frame**: TF 일관성 필수 (vehicle_tf_broadcaster 등과 동일).

초기 P0는 코드 내에서 설정(위치 1 m², 속도 0.4 m²/s², yaw 0.1 rad² 등). 필요하면 파라미터화 가능(향후 TODO).

---

### 6.3 파라미터 상호작용 매트릭스

| 영향받는 것 ↓ / 조정 파라미터 → | sigma_a | sigma_omega | gps_cov | gyro_bias_z | acc_deadband | acc_lpf_hz |
|---|---|---|---|---|---|---|
| **위치 떨림** | ↑ | (간접) | ↓ | - | - | ↑ |
| **속도 드리프트** | ↓ | - | - | - | ↓ | ↑ |
| **yaw 떨림** | - | ↑ | (간접) | ↓ | - | - |
| **yaw 드리프트** | - | ↓ | - | ↑ | - | - |
| **GPS jump 반응속도** | ↓ | - | ↓ | - | - | - |
| **정지시 zero‑vel 유지** | ↓ | ↓ | - | 정확히 | ↑ | ↓(느린필터) |

(화살표 의미: 파라미터 증가 시 해당 항목이 증가/감소 경향.)

---

### 6.4 단계별 튜닝 워크플로우

**Step 0 – 기본 센서 확인**  
`ros2 topic hz` / `rqt_plot` 로 IMU 100 Hz, GPS ~8 Hz 확인. QoS 경고 없도록 SensorDataQoS.

**Step 1 – 자이로 바이어스**  
차량 정지. `/imu/processed/angular_velocity/z` 평균이 0 근처인지 확인. 크면 전처리 파라미터 조정 또는 `gyro_bias_z` 수동 입력.

**Step 2 – 가속도 품질**  
LPF 컷오프/Deadband 조정해 정지시 거의 0, 출발/가속시 반응성 유지.

**Step 3 – EKF sigma_a**  
정지/저속에서 속도 드리프트가 크면 `sigma_a` ↓. 응답이 너무 느리면 ↑.

**Step 4 – EKF sigma_omega**  
정지시 yaw가 흐르면 ↓. 회전시 yaw가 뒤늦게 따라오면 ↑.

**Step 5 – gps_cov**  
GPS가 뛰면 ↑. GPS가 매우 안정적이면 ↓ 로 추정 위치 빠른 보정.

**Step 6 – 실제 주행 비교**  
기존 GPS-only yaw(legacy)와 EKF yaw 비교. 일정 속도 이상에서 ±몇 deg 이내면 OK.

---

## 7. 실행 절차 (Quick Start)

### 7.1 IMU 전처리 패키지 실행
```bash
ros2 launch imu_preproc_ihw imu_preproc_launch.py
```

### 7.2 GPS → Local 변환
```bash
ros2 run gps_global_planner gps_to_local_cartesian --ros-args \
  -p ref_lat:=37.54995 -p ref_lon:=127.05485
```

### 7.3 EKF 실행 (config 사용)
```bash
ros2 launch gps_imu_fusion_ihw gps_imu_fusion_launch.py
```

또는 직접:
```bash
ros2 run gps_imu_fusion_ihw sensor_fusion_node --ros-args \
  --params-file $(ros2 pkg prefix gps_imu_fusion_ihw)/share/gps_imu_fusion_ihw/config/gps_imu_fusion.yaml
```

---

## 8. EKF 초기 Yaw 힌트 연동

전처리 노드가 `/initial_yaw` 를 발행하면 EKF 초기화에 반영해 GPS 진행방향과 즉시 정렬 가능.

```cpp
// SensorFusionNode private:
bool have_yaw_hint_{false};
double yaw_hint_{0.0};
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_init_yaw_;

// ctor:
sub_init_yaw_ = create_subscription<std_msgs::msg::Float32>(
  "/initial_yaw",
  rclcpp::QoS(1).transient_local().reliable(),
  std::bind(&SensorFusionNode::initYawCB, this, _1));

// callback:
void initYawCB(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (initialised_) return;
  yaw_hint_ = msg->data;
  have_yaw_hint_ = true;
  RCLCPP_INFO(get_logger(), "Received yaw hint: %.3f rad", yaw_hint_);
}

// in gpsCallback() *init* block:
double yaw0 = have_yaw_hint_ ? yaw_hint_ : 0.0;
Vector5d x0; x0 << gps_xy(0), gps_xy(1), 0.0, 0.0, yaw0;
```

---

## 9. Troubleshooting

### 9.1 `/global_yaw` 가 계속 0
가능 원인:
1. EKF 초기화 전에 IMU 콜백이 리턴 (정상) → GPS 들어올 때까지 대기.
2. 자이로가 항상 0에 가까움 (bias 제거 후) → 주행 중 회전 데이터 전달 안 됨.
3. `sigma_omega` 너무 작아 yaw 고정.
4. QoS 불일치로 IMU 데이터 미수신 (ROS 로그 확인).
5. 초기 yaw 힌트 0, 차량 속도 낮아 GPS 업데이트만으로 회전 추정 어려움 → `sigma_omega` ↑ 또는 초기_yaw 사용.

### 9.2 QoS 경고: *incompatible reliability*
- Raw IMU BestEffort → 구독자를 SensorDataQoS 로 맞추기.  
  (`rclcpp::SensorDataQoS()` 사용)

### 9.3 위치가 점프
- GPS Cov 잘못 (너무 작음) → gps_cov ↑.
- 위경도 기준점(ref_lat/lon) 불일치 → GPS 변환 노드와 EKF 모두 동일 파라미터인지 확인.

### 9.4 정지중 속도 드리프트
- acc_deadband ↑
- sigma_a ↓
- accel 바이어스 재추정

---

## 10. 확장 로드맵

| 단계 | 항목 | 효과 | 난이도 |
|---|---|---|---|
| 1 | IMU‑Body misalignment 행렬 | 장착오차 보정 | ★★ |
| 2 | 레버암 보정(GPS↔IMU) | 회전시 위치 오차 감소 | ★★ |
| 3 | Zero‑Velocity Update(ZUPT) | 정지중 드리프트 억제 | ★★★ |
| 4 | Roll/Pitch 추정 + 중력 제거 | 가속도 정확도 향상 | ★★★★ |
| 5 | 차량 Kinematics 모델(CV/CTRV) | 속도/요 결합 | ★★★ |
| 6 | UKF / Error‑State EKF | 비선형성 향상 | ★★★★ |

---

## 11. 개정 이력

| 날짜 | 버전 | 작성자 | 변경 내용 |
|---|---|---|---|
| 2025‑07‑16 | 0.2 | ihw + ChatGPT | 전처리/튜닝/트러블슈팅 확장, yaw 기준 명확화. |
| 2025‑07‑16 | 0.1 | ihw + ChatGPT | 최초 작성. |

---

### 관련 문서 & 파일 경로
- `config/imu_preproc.yaml`
- `../gps_imu_fusion_ihw/config/gps_imu_fusion.yaml`
- `../gps_imu_fusion_ihw/config/parameter_tuning_guide.md`
- `../gps_imu_fusion_ihw/explanation.md`

---