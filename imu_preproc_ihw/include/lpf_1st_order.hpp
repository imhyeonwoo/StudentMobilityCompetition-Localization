#pragma once
#include <cmath>

namespace imu_preproc_ihw {

/**
 * @brief 1차 저역통과 필터 (bilinear transform 기반 discrete 설계).
 *
 * 형태:   y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 * alpha = 1 - exp(-2*pi*fc*dt) 근사 (fc << Fs)
 *
 * 더 정확한 bilinear 설계를 원하면 init()에서 coeffs를 다시 계산.
 */
class FirstOrderLPF {
public:
  FirstOrderLPF() = default;

  void reset(double y0 = 0.0) { y_ = y0; initialized_ = true; }
  void configureCutoff(double fc_hz) { fc_ = fc_hz; }

  double filter(double x, double dt) {
    if (!initialized_) { reset(x); return x; }
    if (fc_ <= 0.0 || dt <= 0.0) return x; // no filter
    double alpha = 1.0 - std::exp(-2.0 * M_PI * fc_ * dt);
    y_ += alpha * (x - y_);
    return y_;
  }

  double value() const { return y_; }

private:
  double fc_{0.0};
  double y_{0.0};
  bool   initialized_{false};
};

} // namespace imu_preproc_ihw
