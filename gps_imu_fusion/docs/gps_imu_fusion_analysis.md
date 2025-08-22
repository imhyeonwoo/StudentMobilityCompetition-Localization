# GPS-IMU Fusion 패키지 분석 보고서

## 1. 현재 적용 상태

### 1.1 IMU Calibration 적용 현황
- **적용된 부분**:
  - 가속도계 bias: [0.0527, 0.1221, 0.1666] m/s²
  - 자이로 bias: [0.0096, 0.0003, 0.0186] rad/s
  - `kai_ekf_core.cpp`의 `setImuCalibration()` 함수를 통해 bias 값 적용

- **미적용 부분**:
  - Allan variance의 bias stability 값 (로드만 하고 활용 안 함)
  - Scale factor 보정
  - Axis misalignment 보정
  - Temperature compensation

### 1.2 EKF 구현 상태
- 15-state EKF (위치 3, 속도 3, 자세 3, 가속도 bias 3, 자이로 bias 3)
- GPS 위치/속도와 IMU 데이터 융합
- 프로세스 노이즈 및 측정 노이즈 모델링

## 2. 장단점 분석

### 2.1 장점
1. **커스터마이징 용이성**
   - 소스 코드 직접 수정 가능
   - 특정 센서 구성에 최적화 가능
   - 추가 센서 통합 시 유연한 대응

2. **단순한 의존성**
   - robot_localization 패키지 불필요
   - 최소한의 외부 라이브러리 사용

3. **전처리 로직 통합**
   - IMU calibration 데이터 직접 활용
   - 센서별 특성을 고려한 처리 가능

### 2.2 단점

#### Critical 이슈 (즉시 수정 필요)
1. **Race Condition**
   - `std::shared_mutex` 선언했지만 getter 함수에서 미사용
   - 멀티스레드 환경에서 데이터 경합 위험

2. **수치적 불안정성**
   - Gimbal lock 근처에서 NaN 발생 가능
   - asin() 입력값 범위 제한 불완전

3. **시간 동기화 부재**
   - IMU와 GPS 타임스탬프 동기화 없음
   - 센서 간 지연 시간 고려 안 함

4. **비정상 dt 처리**
   - dt ≤ 0일 때도 계속 진행
   - 필터 발산 위험

#### High 이슈
- 초기화 순서 문제 (GPS 의존적)
- Allan variance 데이터 미활용
- JSON 파싱 취약성
- 오류 복구 메커니즘 부재

## 4. 파이프라인 통합 적합성

### 4.1 현재 상태: ❌ **부적합**

**주요 문제점**:
1. 멀티스레딩 안전성 미보장
2. 시간 동기화 부재로 부정확한 상태 추정
3. 오류 발생 시 복구 불가능
4. 좌표계 변환 문제 (아래 참조)

### 4.2 개선 필요사항
1. 모든 getter 함수에 thread-safe 접근 구현
2. 센서 타임스탬프 기반 동기화
3. 공분산 행렬 체크 및 리셋 로직
4. 좌표계 변환 명확화

## 5. 좌표계 문제 분석

### 5.1 의심되는 문제
1. **base_link → map 직접 연결**
   - 일반적인 ROS2 구조: map → odom → base_link
   - 현재: map → base_link (odom 프레임 무시)

2. **원점 설정 문제**
   - 첫 GPS 데이터 수신 위치를 (0,0)으로 설정하는 것으로 추정
   - GPS to Cartesian 변환이 제대로 적용되지 않는 증상

### 5.2 확인 필요사항
- TF tree 구조 확인
- GPS reference point 설정 방식
- EKF 초기화 시 좌표 설정

## 6. 권장사항

### 6.1 단기 (테스트용)
1. scripts의 GPS 변환 도구로 좌표계 문제 디버깅
2. 간단한 시나리오에서 동작 확인
3. 로그를 통한 문제점 파악

### 6.2 장기 (프로덕션)
   
2. **현재 코드 유지 시**
   - Critical 이슈 4개 즉시 수정
   - 좌표계 변환 로직 재구현
   - 포괄적인 테스트 스위트 작성

## 7. 결론

현재 직접 구현한 EKF는 학습 목적으로는 가치가 있으나, 실제 로봇 시스템에 적용하기에는 여러 치명적 결함이 있습니다. 특히 좌표계 변환 문제와 스레드 안전성 문제는 즉각적인 시스템 오류를 유발할 수 있습니다.



## 8. ZUPT 개발 방향과 저속 헤딩 안정화 전략

### 8.1 증상 요약
- 문제: 속도가 0에 가까울 때 GNSS 기반 heading을 사용할 수 없어 yaw가 발산. 이동을 시작하면 참값으로 급히 snap, 다시 멈추면 heading이 크게 튐.
- 목표: 정지/저속 구간에서 heading 흔들림 억제, 이동 시 빠른 참값 수렴은 유지.

### 8.2 원인 분석 (현행 로직 기준)
- `ekf_fusion_node.cpp`의 `gnssVelCallback()`에서 `speed < 0.08 m/s`이면 `setGpsHeading(..., false)`로 GNSS heading 비활성화. 정지 시 yaw 제약이 약해 드리프트 발생.
- 정지 시 `setZuptNoiseScale(0.3)`만 적용되어 yaw/bias 억제가 충분히 강하지 않음.
- `publishOdometry()`는 `gps_speed > 0.08`에서 GNSS heading을 곧바로 사용해 자세를 구성해 전환시 튐이 큼.

### 8.3 상태 기계 기반 ZUPT 설계
- 상태: STATIONARY / MOVING (필요 시 ARMING 중간상태)
- 판정 신호:
  - 속도 EMA `v_lpf` (fc≈1.0 Hz)
  - IMU 자이로 노름 `|ω|`, 가속도 잔차 `|a - g|`
- 히스테리시스/시간 지연 적용: 진입/이탈 임계 및 유지시간 분리

권장 초기 파라미터:
- min_speed_for_gnss_heading: 0.4 m/s
- zupt_speed_threshold: 0.3 m/s (진입)
- zupt_release_speed: 0.45 m/s (이탈)
- zupt_time_threshold: 0.6 s (진입 유지)
- zupt_release_time: 0.3 s (이탈 유지)
- zupt_gyro_threshold: 0.012 rad/s
- zupt_accel_threshold: 0.08 m/s^2
- zupt_noise_scale: 0.08 (정지 시 yaw/gyro-bias-z Q 강하게 축소)

전이 조건 스케치:
```cpp
bool zupt_enter = (v_lpf < zupt_speed_threshold) && (gyro_norm < zupt_gyro_thr) && (accel_resid < zupt_accel_thr) for >= zupt_time;
bool zupt_exit  = (v_lpf > zupt_release_speed) for >= zupt_release_time;
```

### 8.4 정지 시 적용 제약
- Zero-Velocity Update: vN=vE=vD=0 관측을 강하게 주입(R 작게)해 속도를 0으로 수렴.
- 프로세스 억제: `KaiEkfCore::updateProcessNoiseMatrix()`에서 정지 모드 시 `gyroZ`/`gyro_biasZ` 노이즈를 `zupt_noise_scale`로 강하게 축소(≈0.05~0.10).
- 헤딩 출력 고정: 퍼블리시 단계에서 ZUPT 중에는 `last_valid_heading`을 유지(unwrap 포함).
- GNSS heading 비활성화 유지: 정지 중 `setGpsHeading(..., false)` 및 heading R 크게 유지.

### 8.5 이동 시작 전환 부드럽게
- 램프 블렌딩: ZUPT→MOVING 후 `τ_blend=0.5~1.0s` 동안 EKF yaw와 GNSS heading을 unwrap 후 가중합.
- 저역통과: 초기 GNSS heading에 0.5~1.0 Hz 1차 IIR 적용.

### 8.6 GNSS heading 게이팅 강화
- 속도: `speed > max(min_speed_for_gnss_heading, 3·σ_speed)` 충족 시만 유효.
- 변화량: 직전 대비 Δψ > 45°면 일시 게이트 후 추가 샘플 확인.
- 공분산: vN/vE 공분산으로 근사한 heading 분산이 크면 무시.

### 8.7 코드 반영 포인트
- `ekf_fusion_node.cpp`
  - `gnssVelCallback()`: 속도 EMA, 히스테리시스 기반 ZUPT 상태기계, 정지 시 `setStationary(true)`, `setZuptNoiseScale(≤0.1)`, GNSS heading 비활성화.
  - `publishOdometry()`: ZUPT 중에는 `last_valid_heading` 고정, 전환 구간엔 EKF yaw vs GNSS heading 블렌딩(unwrap, α 램프).
- `kai_ekf_core.cpp`
  - 정지 모드 Q 감쇠를 현행보다 강하게 적용. 필요시 yaw-rate≈0 약한 관측 추가 검토.

### 8.8 `config/fusion_params.yaml` 제안 항목
- min_speed_for_gnss_heading: 0.4
- zupt_speed_threshold: 0.3
- zupt_release_speed: 0.45
- zupt_time_threshold: 0.6
- zupt_release_time: 0.3
- zupt_noise_scale: 0.08
- zupt_gyro_threshold: 0.012
- zupt_accel_threshold: 0.08
- 신규(추가 구현 필요):
  - heading_filter_cutoff_hz: 0.8
  - heading_blend_time_constant: 0.7

### 8.9 테스트 지표(간단)
- 정지 10초→직진 10m→정지 10초 시나리오 반복
- 정지 중 yaw σ < 0.5°
- 전환 1초 내 yaw 오차 < 5°
- 상태 전이 불필요한 플리커 최소화