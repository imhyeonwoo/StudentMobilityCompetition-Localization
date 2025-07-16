#include "imu_preproc_ihw/imu_preproc.hpp"
#include <cmath>

namespace imu_preproc_ihw
{

static inline double wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

void ImuPreprocFilter::setParams(const ImuPreprocParams& p)
{
  params_ = p;
  gyro_bias_z_ = p.gyro_bias_z;
  acc_bias_x_  = p.acc_bias_x;
  acc_bias_y_  = p.acc_bias_y;
  lpf_ax_.configureCutoff(p.acc_lpf_hz);
  lpf_ay_.configureCutoff(p.acc_lpf_hz);
  lpf_ax_.reset(0.0);
  lpf_ay_.reset(0.0);
  accum_gz_ = accum_ax_ = accum_ay_ = accum_time_ = 0.0;
  gps_buf_.clear();
}

bool ImuPreprocFilter::isStationary(double ax, double ay, double az, double gz)
{
  // 단순 기준: gyro_z < thr & XY accel 소량
  double g = 9.80665;
  double anorm = std::sqrt(ax*ax + ay*ay + az*az);
  double a_dev = std::fabs(anorm - g);  // g와의 차이
  if (std::fabs(gz) < params_.gyro_stationary_thr && a_dev < 0.5)
    return true;
  return false;
}

void ImuPreprocFilter::updateBiases(double ax, double ay, double gz, double dt)
{
  accum_time_ += dt;
  accum_gz_   += gz * dt;
  accum_ax_   += ax * dt;
  accum_ay_   += ay * dt;

  if (accum_time_ >= params_.gyro_bias_window_sec) {
    if (params_.auto_gyro_bias)
      gyro_bias_z_ = accum_gz_ / accum_time_;
    if (params_.auto_acc_bias) {
      acc_bias_x_ = accum_ax_ / accum_time_;
      acc_bias_y_ = accum_ay_ / accum_time_;
    }
    accum_time_ = accum_gz_ = accum_ax_ = accum_ay_ = 0.0;
  }
}

void ImuPreprocFilter::processImu(double ax, double ay, double az,
                                  double gx, double gy, double gz,
                                  double dt,
                                  Eigen::Vector2d &acc_xy_out,
                                  double &omega_z_out,
                                  bool stationary_flag)
{
  if (stationary_flag)
    updateBiases(ax, ay, gz, dt);
  else
    accum_time_ = accum_gz_ = accum_ax_ = accum_ay_ = 0.0; // reset window

  // bias remove
  double gz_corr = gz - gyro_bias_z_;
  double ax_corr = ax - acc_bias_x_;
  double ay_corr = ay - acc_bias_y_;

  // lpf
  double ax_f = lpf_ax_.filter(ax_corr, dt);
  double ay_f = lpf_ay_.filter(ay_corr, dt);

  // deadband
  if (std::fabs(ax_f) < params_.acc_deadband) ax_f = 0.0;
  if (std::fabs(ay_f) < params_.acc_deadband) ay_f = 0.0;

  if (!params_.use_accel) {
    ax_f = 0.0;
    ay_f = 0.0;
  }

  acc_xy_out.x() = ax_f;
  acc_xy_out.y() = ay_f;
  omega_z_out    = gz_corr;
}

void ImuPreprocFilter::pushGps(double x, double y, double t_sec)
{
  gps_buf_.push_back({x,y,t_sec});
  // cap buffer
  size_t maxlen = std::max(2, params_.yaw_window);
  while (gps_buf_.size() > maxlen) gps_buf_.pop_front();
}

std::pair<bool,double> ImuPreprocFilter::tryComputeYaw()
{
  if (!params_.compute_initial_yaw) return {false, 0.0};
  if (gps_buf_.size() < 2) return {false, 0.0};

  // use newest & oldest in buffer
  const auto &a = gps_buf_.front();
  const auto &b = gps_buf_.back();
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dt = b.t_sec - a.t_sec;
  if (dt <= 0) return {false, 0.0};
  double speed = std::hypot(dx,dy) / dt;
  if (speed < params_.yaw_speed_thresh)
    return {false, 0.0};

  double yaw = std::atan2(dy, dx);
  return {true, wrapAngle(yaw)};
}

} // namespace imu_preproc_ihw
