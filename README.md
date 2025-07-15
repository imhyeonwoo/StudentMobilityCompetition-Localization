# GPS and IMU Sensor Fusion

ROS 2 package that fuses **GPS (7 Hz)** and **IMU (100 Hz)** using an Extended Kalman Filter (EKF) to output:

| Topic              | Type                  | Freq   | Description                                         |
|--------------------|-----------------------|--------|-----------------------------------------------------|
| `/odometry/fusion` | `nav_msgs/Odometry`   | 100 Hz | x [m], y [m], ψ [rad], vₓ, vᵧ, ω_z                |
| `/global_yaw`      | `std_msgs/Float32`    | 100 Hz | ψ (wrapped –π ~ π)                                  |
| `(TF)`             | `reference → gps_antenna` | 100 Hz | same pose as odometry                          |

---

## Installation

Clone the repo and build:

```bash
git clone https://github.com/yourname/gps_imu_fusion_ihw.git
cd gps_imu_fusion_ihw
colcon build --packages-select gps_imu_fusion_ihw
source install/setup.bash
```

---

## Launch

Launch with a YAML config:

```bash
ros2 launch gps_imu_fusion_ihw gps_imu_fusion_launch.py
```

---

## 1. Architecture

```
   /ouster/imu (100 Hz)         /ublox_gps_node/fix (7 Hz)
          │                             │
   ┌──────▼──────┐                ┌─────▼─────┐
   │  QoS: Best  │  predict()     │  local_xy │  update()
   │   IMU sub   │────────────▶   │   GPS     │──────────┐
   └─────────────┘                └───────────┘          │
               ┌────────────────────────────┐            │
               │      KalmanFilter          │<───────────┘
               └─────────┬────────┬─────────┘
                         │        │
              /odometry/fusion  /global_yaw
```

### 1.1 State Vector

$$
\mathbf{x} = \begin{bmatrix}
x \\
y \\
\psi \\
v_x \\
v_y \\
\omega_z
\end{bmatrix}
$$

### 1.2 Prediction (see “Prediction Step” pdf)

* Constant‑velocity model (`A`, Δt)
* IMU accelerations + yaw‑rate as control (`B u`)
* Process‑noise `Q = B Σ Bᵀ`, where Σ = diag(σₐ², σₐ², σ_ω²)

### 1.3 Update (GPS position only)

* `H` selects x,y rows
* GPS covariance `R = gps_cov·I₂`
* Standard EKF equations (Kalman Gain `K`, residual `y`)

Angle ψ is wrapped each step:

```cpp
psi = atan2(sin(psi), cos(psi));
```

---

## 2. Code Walk‑Through

```
include/gps_imu_fusion_ihw/kalman_filter.hpp     <-- core EKF class  
src/kalman_filter.cpp                           <-- math  
src/sensor_fusion_node.cpp                      <-- ROS2 node & QoS  
config/gps_imu_fusion.yaml                      <-- parameters  
launch/gps_imu_fusion_launch.py                 <-- launch file
```

### 2.1 Important snippets

```cpp
/* QoS so we actually receive Ouster IMU (BestEffort) */
auto imu_qos = rclcpp::SensorDataQoS();
sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/ouster/imu", imu_qos, ...);

/* ψ wrap after predict / update */
x_(4) = std::atan2(std::sin(x_(4)), std::cos(x_(4)));
```

---

## 3. Parameters

| YAML key      | Default     | Unit     | Matrix / Role         | 효과                                              | 튜닝 요령 |
|---------------|-------------|----------|------------------------|---------------------------------------------------|-----------|
| `sigma_a`     | **0.25**    | m/s²     | `Q` linear‑acc         | ↑ → GPS 의존 ↑, 진동 ↓ / 지연 ↑                  | 주행 중 위치 **치우침**, 느린 반응 → ↓ |
| `sigma_omega` | **0.03**    | rad/s    | `Q` yaw‑rate           | ↑ → GPS 헤딩 의존 ↑, ↓ → IMU 적분 ↑ (드리프트)   | 정지 시 ψ 드리프트 ↗ → ↑ |
| `gps_cov`     | **0.000196**| m²       | `R` (measurement)      | ↑ → GPS를 덜 믿음                                | GPS 수신 불량(도심 캐니언 등) 시 ↑ |
| `dt_default`  | 0.01        | s        | Δt fallback            | IMU 타임스탬프 jump 보호                          | 그대로 |
| `map_frame`   | "reference" | –        | TF parent              | –                                                 | 현장 좌표계에 맞춰 변경 |
| `base_frame`  | "gps_antenna"| –       | TF child               | –                                                 | 센서 위치명 |
| `publish_tf`  | true        | bool     | –                      | TF on/off                                         | RViz TF 겹칠 때 false |

> `P₀` (초기 공분산)는 코드에 고정: **diag(1 m, 1 m, 0.4 m/s, 0.4 m/s, 0.1 rad)**. 필요 시 `setInitialState()` 수정.

---

## 4. Practical Tuning Workflow

1. **로그 재생** (rosbag / recorded ros2 bag)
2. `rqt_plot`: `/global_yaw` vs GPS‑only yaw → 드리프트/지연 확인
3. 조정 loop
   * **Yaw 느림** → `sigma_omega` ↓
   * **Yaw 흔들림** → `sigma_omega` ↑
   * **위치 바운스** → `sigma_a` ↑
   * **위치 둔함** → `sigma_a` ↓
   * **GPS 튐** → `gps_cov` ↑  
   → 반복

---

## 5. Launch & Runtime Hints

```bash
ros2 launch gps_imu_fusion_ihw gps_imu_fusion_launch.py

# runtime param tweak
ros2 param set /gps_imu_sensor_fusion sigma_omega 0.045
ros2 param get /gps_imu_sensor_fusion gps_cov
```

