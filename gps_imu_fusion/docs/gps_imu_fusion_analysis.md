# GPS-IMU Fusion 패키지 분석 보고서 (2024.12 최신)

## 1. 현재 적용 상태

### 1.1 IMU Calibration 적용 현황
- **미적용 상태**:
  - `setImuCalibration()` 함수 존재하지 않음
  - IMU calibration 파일 파라미터 정의만 되어있고 실제 사용 안 함 (ekf_fusion.launch.py:23-36줄)
  - 가속도계 bias는 IMU 전처리 노드에서 처리된다고 주석 표시 (kai_ekf_core.cpp:356-357줄)
  
- **실제 구현**:
  - 자이로 bias만 EKF에서 온라인 추정 (kai_ekf_core.cpp:84-86, 359-361줄)
  - 초기값: gbx=0, gby=0, gbz=0
  - Allan variance, scale factor, axis misalignment 모두 미구현

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
   - 최소한의 외부 라이브러리 사용 (Eigen, GeographicLib)

3. **스레드 안전성 구현**
   - shared_mutex를 통한 읽기/쓰기 동시성 제어
   - getter는 shared_lock, setter는 unique_lock 사용

### 2.2 단점

#### Critical 이슈 (수정 완료/미완료)
1. **Race Condition** ✅ 수정됨
   - getter 함수에서 `std::shared_lock` 제대로 사용 중 (kai_ekf_core.hpp:73-101줄)
   - setter 함수에서 `std::unique_lock` 사용 (kai_ekf_core.cpp:31,36,198줄 등)

2. **수치적 불안정성** ⚠️ 부분 수정
   - asin() 입력값 범위 제한 적용됨 (kai_ekf_core.cpp:235-252줄)
   - 쿼터니언 NaN 체크 및 리셋 로직 있음 (kai_ekf_core.cpp:342-345줄)
   - 하지만 Gimbal lock 회피 로직은 불완전

3. **시간 동기화 부재** ❌ 미수정
   - IMU와 GPS 타임스탬프 동기화 없음
   - 센서 간 지연 시간 고려 안 함

4. **비정상 dt 처리** ✅ 수정됨
   - dt ≤ 0일 때 return으로 처리 (kai_ekf_core.cpp:123-125줄)

#### High 이슈 (미수정)
- 초기화 순서 문제 (GPS/GNSS velocity 모두 필요) ❌
- Allan variance 데이터 미활용 ❌
- IMU calibration JSON 파일 미사용 ❌
- 오류 복구 메커니즘 부재 ❌

## 3. 자동차 특화 구현
- Roll/Pitch를 0으로 고정 (평지 주행 가정) - kai_ekf_core.cpp:90-92, 349-351줄
- Z축 각속도(yaw rate)만 사용 - kai_ekf_core.cpp:374-380줄
- 정지 모드 시 yaw rate 0 강제 - kai_ekf_core.cpp:377-378줄

## 4. 파이프라인 통합 적합성

### 4.1 현재 상태: ⚠️ **부분 적합**

**해결된 문제**:
1. 멀티스레딩 안전성 ✅ (shared_mutex 제대로 사용)
2. 비정상 dt 처리 ✅

**미해결 문제**:
1. 시간 동기화 부재로 부정확한 상태 추정 ❌
2. 오류 발생 시 복구 메커니즘 부재 ❌
3. 좌표계 변환 문제 (아래 참조) ⚠️

### 4.2 개선 필요사항
1. ~~모든 getter 함수에 thread-safe 접근 구현~~ ✅ 완료
2. 센서 타임스탬프 기반 동기화 ❌ 필요
3. 공분산 행렬 체크 및 리셋 로직 ❌ 필요
4. 좌표계 변환 명확화 ⚠️ 부분 문제

## 5. 좌표계 문제 분석

### 5.1 확인된 구현
1. **TF 구조**
   - 실제: odom → base_link만 발행 (ekf_fusion_node.cpp:652-653줄)
   - map 프레임 미사용 (world_frame_id는 정의되어 있지만 TF 발행 안 함)
   - 일반적인 ROS2 구조(map → odom → base_link)와 다름

2. **원점 설정**
   - 건국대 일감호 고정 원점 사용 (ekf_fusion_node.cpp:238-246줄)
   - 참조점: lat=37.540091, lon=127.076555
   - UTM 좌표계로 변환 후 상대 위치 계산

### 5.2 잠재적 문제
- map → odom 변환 부재로 전역 위치 보정 어려움
- 고정 원점이므로 다른 지역에서 사용 시 수정 필요

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

## 7. 결론 (2024.12 업데이트)

### 7.1 개선된 부분
- 스레드 안전성 ✅ (mutex 제대로 구현)
- dt 예외 처리 ✅
- NaN 방지 로직 ✅
- asin() 범위 제한 ✅

### 7.2 주요 미해결 이슈
- IMU calibration 파일 미사용 ❌
- 시간 동기화 부재 ❌  
- map → odom TF 미발행 ⚠️
- 고정 원점 사용 (건국대 일감호) ⚠️

### 7.3 종합 평가
현재 구현은 기본적인 안전성 문제는 해결되었으나, IMU calibration 미적용과 시간 동기화 부재로 정밀도가 제한적입니다. 테스트 목적으로는 사용 가능하나 프로덕션 적용 시 추가 개선 필요합니다.



## 8. Heading 융합 메커니즘 재분석 및 개선 방향 (2025.08.22. 업데이트)

### 8.1 현재 구현 상태 (정확한 분석)

#### 8.1.1 실제 동작 메커니즘
1. **EKF 내부**:
   - IMU 각속도 적분 로직 존재 (`kai_ekf_core.cpp:138`: `dq(3) = 0.5f * om_ib(2,0) * dt`)
   - GPS heading과 EKF heading 간 잔차를 칼만필터로 융합
   - R(6,6) 행렬로 GPS heading 가중치 제어

2. **출력 단계 Override 문제**:
   - `publishOdometry()`:513-520에서 EKF 결과를 무시하고 직접 계산
   - 속도 ≥ 0.08 m/s: GPS 속도 벡터로 직접 heading 계산 (`atan2(vE, vN)`)
   - 속도 < 0.08 m/s: EKF heading 사용 (하지만 GPS 노이즈로 불안정)

### 8.2 핵심 문제점 재정의
1. **정지 시 발산**: GPS 속도가 0에 가까울 때 `atan2(노이즈, 노이즈)`로 무작위 방향
2. **이진 스위칭**: 0.08 m/s 경계에서 급격한 전환 (ON/OFF)
3. **EKF 융합 무시**: 출력 단계에서 EKF 결과 대신 GPS 직접 사용

### 8.3 속도 기반 3단계 융합 전략 (새로운 접근)

#### 8.3.1 속도 영역 정의
1. **저속 영역** (0 ~ 0.08 m/s): 
   - IMU gyro만 사용
   - GPS heading 완전 차단 (R = 1e10)
   - atan2 발산 방지

2. **전환 영역** (0.08 ~ 0.5 m/s):
   - GPS heading 매우 강하게 신뢰 (R = 0.001)
   - 빠른 참값 수렴 목적
   - IMU 드리프트 보정

3. **고속 영역** (> 0.5 m/s):
   - GPS/IMU 적절한 융합 (R = 0.01)
   - 8Hz 이상 부드러운 출력
   - GPS 탁탁 튀는 현상 완화

#### 8.3.2 핵심 파라미터 조절 메커니즘
```cpp
// R(6,6) 동적 조절 (측정 노이즈)
if (speed < 0.08) {
    R(6,6) = 1e10f;        // GPS heading 무시
} else if (speed < 0.5) {
    R(6,6) = 0.001f;       // GPS heading 매우 신뢰 (빠른 수렴)
} else {
    R(6,6) = 0.01f;        // 적절한 융합
}

// Q 행렬 조절 (프로세스 노이즈)
if (stationary_mode_) {
    Rw(5,5) *= 0.1f;       // gyro Z 노이즈 축소
    Rw(11,11) *= 0.1f;     // gyro bias Z 노이즈 축소
}
```

### 8.4 구현 방안

#### 8.4.1 kai_ekf_core.cpp 수정
```cpp
// 새로운 함수 추가: 속도 기반 GPS heading 노이즈 동적 설정
void KaiEkfCore::setGpsHeadingNoise(float speed) {
    if (speed < 0.08f) {
        R(6,6) = 1e10f;     // 저속: GPS heading 무시
    } else if (speed < 0.5f) {
        R(6,6) = 0.001f;    // 전환: GPS heading 강하게 신뢰
    } else {
        R(6,6) = 0.01f;     // 고속: 적절한 융합
    }
}
```

#### 8.4.2 ekf_fusion_node.cpp 수정
```cpp
void EkfFusionNode::gnssVelCallback(...) {
    // 속도별 3단계 처리
    if (speed < 0.08) {
        ekf_->setStationary(true);
        ekf_->setZuptNoiseScale(0.1f);
        ekf_->setGpsHeading(0.0f, false);
    } else if (speed < 0.5) {
        ekf_->setStationary(false);
        ekf_->setZuptNoiseScale(1.0f);
        float gps_heading = std::atan2(vel.vE, vel.vN);
        ekf_->setGpsHeading(gps_heading, true);
        ekf_->setGpsHeadingNoise(speed);  // 전환 영역 특별 처리
    } else {
        // 고속 영역 - 기존과 동일
        ekf_->setGpsHeadingNoise(speed);  // 융합 모드
    }
}

// publishOdometry()에서 EKF 결과 직접 사용하도록 수정
void EkfFusionNode::publishOdometry() {
    float heading = ekf_->getHeading_rad();  // 항상 EKF 결과 사용
    // GPS 직접 계산 제거
}
```

### 8.5 config/fusion_params.yaml 제안 수정
```yaml
# 속도 영역 임계값 (새로 추가)
speed_threshold_low: 0.08        # 저속/전환 경계 (m/s)
speed_threshold_high: 0.5        # 전환/고속 경계 (m/s)

# GPS heading 노이즈 (속도별)
gps_heading_noise_low: 1e10      # 저속 영역 (사실상 무시)
gps_heading_noise_mid: 0.001     # 전환 영역 (매우 신뢰)
gps_heading_noise_high: 0.01     # 고속 영역 (적절한 융합)

# ZUPT 관련 (기존)
zupt_speed_threshold: 0.08       # ZUPT 활성화 속도
zupt_noise_scale: 0.1            # 정지 시 프로세스 노이즈 스케일

# 기존 파라미터
use_gnss_heading: true            # GNSS heading 사용 여부
```

### 8.6 기대 효과
1. **정지 시**: IMU gyro만 사용하여 atan2 발산 방지
2. **전환 영역**: GPS heading 강하게 신뢰하여 빠른 참값 수렴
3. **고속 주행**: GPS/IMU 적절한 융합으로 부드러운 출력

### 8.7 테스트 시나리오
1. **정지 → 가속 → 정지**
   - 정지 10초 → 2 m/s까지 가속 → 10m 직진 → 정지 10초
   - 평가: 전환 시 heading snap 최소화

2. **저속 순환 주행**
   - 0.3 m/s로 원형 경로 주행
   - 평가: 전환 영역에서 heading 안정성

3. **고속 직진**
   - 1 m/s 이상으로 직진
   - 평가: GPS/IMU 융합 부드러움 (8Hz 이상 체감)

## 9. 결론 및 요약 (2025.08.22. 업데이트)

### 9.1 현재 상태 진단
- **문제의 본질**: EKF 내부에서는 GPS/IMU 융합이 작동하지만, 출력 단계(`publishOdometry`)에서 EKF 결과를 무시하고 GPS 속도 직접 사용
- **정지 시 발산**: GPS 속도 노이즈로 인한 `atan2(노이즈, 노이즈)` 문제
- **이진 스위칭**: 0.08 m/s 경계에서 급격한 ON/OFF 전환

### 9.2 제안된 해결책
**속도 기반 3단계 융합 전략**:
1. **저속** (< 0.08 m/s): IMU만 사용 (R = 1e10)
2. **전환** (0.08 ~ 0.5 m/s): GPS 강하게 신뢰 (R = 0.001)  
3. **고속** (> 0.5 m/s): 적절한 융합 (R = 0.01)

### 9.3 구현 우선순위
1. **즉시**: `publishOdometry()`에서 EKF 결과 직접 사용하도록 수정
2. **단기**: R(6,6) 동적 조절 함수 추가
3. **장기**: 히스테리시스 및 부드러운 전환 구현