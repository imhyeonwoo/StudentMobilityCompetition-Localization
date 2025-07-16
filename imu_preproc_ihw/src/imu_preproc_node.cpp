#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "imu_preproc_ihw/imu_preproc.hpp"

using namespace std::chrono_literals;

namespace imu_preproc_ihw
{

class ImuPreprocessorNode : public rclcpp::Node
{
public:
  ImuPreprocessorNode()
  : Node("imu_preproc")
  {
    /* ==================== 파라미터 로드 ==================== */
    params_.gyro_bias_z         = declare_parameter("gyro_bias_z", 0.0);
    params_.auto_gyro_bias      = declare_parameter("auto_gyro_bias", true);
    params_.gyro_bias_window_sec= declare_parameter("gyro_bias_window_sec", 10.0);
    params_.gyro_stationary_thr = declare_parameter("gyro_stationary_thr", 0.02);

    params_.use_accel           = declare_parameter("use_accel", true);
    params_.remove_gravity      = declare_parameter("remove_gravity", false);
    params_.acc_bias_x          = declare_parameter("acc_bias_x", 0.0);
    params_.acc_bias_y          = declare_parameter("acc_bias_y", 0.0);
    params_.auto_acc_bias       = declare_parameter("auto_acc_bias", true);
    params_.acc_bias_window_sec = declare_parameter("acc_bias_window_sec", 10.0);
    params_.acc_lpf_hz          = declare_parameter("acc_lpf_hz", 1.0);
    params_.acc_deadband        = declare_parameter("acc_deadband", 0.05);

    params_.compute_initial_yaw = declare_parameter("compute_initial_yaw", true);
    params_.yaw_speed_thresh    = declare_parameter("yaw_speed_thresh", 0.5);
    params_.yaw_window          = declare_parameter("yaw_window", 2);

    imu_in_topic_  = declare_parameter("imu_in_topic",  std::string("/ouster/imu"));
    gps_xy_topic_  = declare_parameter("gps_xy_topic",  std::string("/local_xy"));
    imu_out_topic_ = declare_parameter("imu_out_topic", std::string("/imu/processed"));
    yaw_out_topic_ = declare_parameter("yaw_out_topic", std::string("/initial_yaw"));

    debug_print_   = declare_parameter("debug_print", true);
    debug_period_  = declare_parameter("debug_period", 0.5);

    filter_.setParams(params_);

    /* ==================== 퍼블리셔 ==================== */
    // processed IMU: SensorDataQoS
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        imu_out_topic_, rclcpp::SensorDataQoS());

    // initial yaw: latched once, so use transient_local reliable
    rclcpp::QoS qos_yaw(1);
    qos_yaw.transient_local();
    qos_yaw.reliable();
    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(yaw_out_topic_, qos_yaw);

    /* ==================== 서브스크라이버 ==================== */
    using std::placeholders::_1;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_in_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuPreprocessorNode::imuCB, this, _1));

    gps_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        gps_xy_topic_, 10,
        std::bind(&ImuPreprocessorNode::gpsCB, this, _1));

    /* ==================== 디버그 타이머 ==================== */
    if (debug_print_) {
      debug_timer_ = create_wall_timer(
          std::chrono::duration<double>(debug_period_),
          std::bind(&ImuPreprocessorNode::debugTimer, this));
    }

    RCLCPP_INFO(get_logger(), "ImuPreprocessorNode started.");
  }

private:
  // ----------------------------------------------------------
  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time curr(msg->header.stamp);
    double curr_t = curr.seconds();
    double dt = 0.0;
    if (last_imu_time_.nanoseconds() != 0) {
      dt = (curr - last_imu_time_).seconds();
      if (dt <= 0 || dt > 0.2) dt = 0.01; // fallback
    } else {
      dt = 0.01;
    }
    last_imu_time_ = curr;

    // raw
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    // stationary detect uses raw (or filtered?) - raw is fine
    bool stationary = filter_.isStationary(ax, ay, az, gz);

    Eigen::Vector2d acc_xy;
    double omega_z;
    filter_.processImu(ax, ay, az, gx, gy, gz, dt, acc_xy, omega_z, stationary);

    // build processed Imu msg
    auto out = sensor_msgs::msg::Imu();
    out.header = msg->header; // keep same stamp & frame_id
    // orientation unchanged; we don't trust it but keep for debugging
    out.angular_velocity.x = 0.0;
    out.angular_velocity.y = 0.0;
    out.angular_velocity.z = omega_z;

    if (params_.use_accel) {
      out.linear_acceleration.x = acc_xy.x();
      out.linear_acceleration.y = acc_xy.y();
      out.linear_acceleration.z = 0.0;
    } else {
      out.linear_acceleration.x = 0.0;
      out.linear_acceleration.y = 0.0;
      out.linear_acceleration.z = 0.0;
    }
    imu_pub_->publish(out);
  }

  // ----------------------------------------------------------
  void gpsCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    double t = rclcpp::Time(msg->header.stamp).seconds();
    filter_.pushGps(msg->point.x, msg->point.y, t);

    if (params_.compute_initial_yaw && !yaw_published_) {
      auto [ok, yaw] = filter_.tryComputeYaw();
      if (ok) {
        std_msgs::msg::Float32 m;
        m.data = static_cast<float>(yaw);
        yaw_pub_->publish(m);
        yaw_published_ = true;
        RCLCPP_INFO(get_logger(), "Initial yaw computed: %.3f rad (%.1f deg)", yaw, yaw*180.0/M_PI);
      }
    }
  }

  // ----------------------------------------------------------
  void debugTimer()
  {
    RCLCPP_INFO(get_logger(),
      "gyro_bias_z=%.5f  acc_bias=(%.3f,%.3f)",
      filter_.gyroBiasZ(), filter_.accBiasX(), filter_.accBiasY());
  }

  /* ---- 멤버 ---- */
  ImuPreprocParams params_;
  ImuPreprocFilter filter_;

  std::string imu_in_topic_, gps_xy_topic_, imu_out_topic_, yaw_out_topic_;
  bool debug_print_{true};
  double debug_period_{0.5};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  rclcpp::Time last_imu_time_;
  bool yaw_published_{false};
};

} // namespace imu_preproc_ihw

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_preproc_ihw::ImuPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
