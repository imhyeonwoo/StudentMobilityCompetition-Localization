#pragma once
#include <deque>
#include <string>
#include <Eigen/Core>

#include "lpf_1st_order.hpp"

namespace imu_preproc_ihw
{

struct GpsSample {
  double x;
  double y;
  double t_sec;   // stamp.sec + nanosec*1e-9
};

struct ImuPreprocParams
{
  // gyro
  double gyro_bias_z{0.0};
  bool   auto_gyro_bias{true};
  double gyro_bias_window_sec{10.0};
  double gyro_stationary_thr{0.02};

  // accel
  bool   use_accel{true};
  bool   remove_gravity{false};
  double acc_bias_x{0.0};
  double acc_bias_y{0.0};
  bool   auto_acc_bias{true};
  double acc_bias_window_sec{10.0};
  double acc_lpf_hz{1.0};
  double acc_deadband{0.05};

  // yaw init
  bool   compute_initial_yaw{true};
  double yaw_speed_thresh{0.5};
  int    yaw_window{2};
};

class ImuPreprocFilter
{
public:
  ImuPreprocFilter() = default;
  explicit ImuPreprocFilter(const ImuPreprocParams& p) { setParams(p); }

  void setParams(const ImuPreprocParams& p);

  // feed raw imu sample; return processed (acc_xy, omega_z)
  void processImu(double ax, double ay, double az,
                  double gx, double gy, double gz,
                  double dt,
                  Eigen::Vector2d &acc_xy_out,
                  double &omega_z_out,
                  bool stationary_flag);

  // push gps sample
  void pushGps(double x, double y, double t_sec);

  // try compute yaw; return pair(success, yaw)
  std::pair<bool,double> tryComputeYaw();

  // stationary detection (simple magnitude thresholds)
  bool isStationary(double ax, double ay, double az,
                    double gz);

  // expose dynamic bias (if auto enabled)
  double gyroBiasZ() const { return gyro_bias_z_; }
  double accBiasX()  const { return acc_bias_x_; }
  double accBiasY()  const { return acc_bias_y_; }

private:
  // update biases in stationary window
  void updateBiases(double ax, double ay, double gz, double dt);

  ImuPreprocParams params_;

  // running filtered values
  FirstOrderLPF lpf_ax_;
  FirstOrderLPF lpf_ay_;

  // dynamic biases
  double gyro_bias_z_{0.0};
  double acc_bias_x_{0.0};
  double acc_bias_y_{0.0};

  // accumulators for bias estimation
  double accum_gz_{0.0};
  double accum_ax_{0.0};
  double accum_ay_{0.0};
  double accum_time_{0.0};

  std::deque<GpsSample> gps_buf_;
};

} // namespace imu_preproc_ihw
