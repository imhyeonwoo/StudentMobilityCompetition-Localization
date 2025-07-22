#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>

class ImuPreprocessNode : public rclcpp::Node
{
public:
  ImuPreprocessNode() : Node("imu_preprocess_node")
  {
    /* ------------ 파라미터 선언 ------------ */
    calib_duration_ = declare_parameter<double>("calib_duration", 20.0);   // [s]
    lpf_cutoff_     = declare_parameter<double>("lpf_cutoff",    15.0);   // [Hz]

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&ImuPreprocessNode::imuCb, this, std::placeholders::_1));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/processed", rclcpp::SensorDataQoS());

    start_time_ = now();
  }

private:
  /* ---------- 콜백 ---------- */
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double dt =
      (last_time_.nanoseconds() == 0) ? 0.01
      : (now() - last_time_).seconds();
    last_time_ = now();

    /* 1. 정적 캘리브레이션 구간 ---------------------------- */
    if (!calibrated_) {
      accumulateBias(*msg);
      if ((now() - start_time_).seconds() >= calib_duration_) {
        finalizeBias();
        calibrated_ = true;
        RCLCPP_INFO(get_logger(), "IMU bias calibration done.");
      }
      return;  // 아직 퍼블리시하지 않음
    }

    /* 2. 편향 제거 ------------------------------------------ */
    sensor_msgs::msg::Imu out = *msg;
    out.linear_acceleration.x -= bias_acc_[0];
    out.linear_acceleration.y -= bias_acc_[1];
    out.linear_acceleration.z -= bias_acc_[2];
    out.angular_velocity.x    -= bias_gyro_[0];
    out.angular_velocity.y    -= bias_gyro_[1];
    out.angular_velocity.z    -= bias_gyro_[2];

    /* 3. 1차 IIR LPF ---------------------------------------- */
    if (lpf_cutoff_ > 0.0) {
      const double tau = 1.0 / (2.0 * M_PI * lpf_cutoff_);
      const double alpha = dt / (tau + dt);

      auto lpf = [alpha](double x, double &prev){
        double y = alpha * x + (1.0 - alpha) * prev;
        prev = y;
        return y;
      };

      out.linear_acceleration.x = lpf(out.linear_acceleration.x, acc_prev_[0]);
      out.linear_acceleration.y = lpf(out.linear_acceleration.y, acc_prev_[1]);
      out.linear_acceleration.z = lpf(out.linear_acceleration.z, acc_prev_[2]);
      out.angular_velocity.x    = lpf(out.angular_velocity.x,    gyro_prev_[0]);
      out.angular_velocity.y    = lpf(out.angular_velocity.y,    gyro_prev_[1]);
      out.angular_velocity.z    = lpf(out.angular_velocity.z,    gyro_prev_[2]);
    }

    imu_pub_->publish(out);
  }

  /* ---------- 바이어스 누적·계산 ---------- */
  void accumulateBias(const sensor_msgs::msg::Imu &m)
  {
    sum_acc_[0]  += m.linear_acceleration.x;
    sum_acc_[1]  += m.linear_acceleration.y;
    sum_acc_[2]  += m.linear_acceleration.z - 9.81;  // 중력 제거(Z)
    sum_gyro_[0] += m.angular_velocity.x;
    sum_gyro_[1] += m.angular_velocity.y;
    sum_gyro_[2] += m.angular_velocity.z;
    ++sample_cnt_;
  }
  void finalizeBias()
  {
    for (int i = 0; i < 3; ++i) {
      bias_acc_[i]  = sum_acc_[i]  / sample_cnt_;
      bias_gyro_[i] = sum_gyro_[i] / sample_cnt_;
    }
  }

  /* ---------- 멤버 변수 ---------- */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr     imu_pub_;
  rclcpp::Time start_time_, last_time_;

  double calib_duration_, lpf_cutoff_;
  bool calibrated_{false};
  std::array<double,3> sum_acc_{}, sum_gyro_{}, bias_acc_{}, bias_gyro_{};
  std::array<double,3> acc_prev_{}, gyro_prev_{};
  size_t sample_cnt_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
