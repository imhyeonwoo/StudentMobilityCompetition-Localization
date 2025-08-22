#ifndef EKF_FUSION_NODE_HPP
#define EKF_FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <memory>
#include <GeographicLib/UTMUPS.hpp>

// -------------------- 1. Path 메시지 헤더 추가 --------------------
#include <nav_msgs/msg/path.hpp>

#include "gps_imu_fusion/kai_ekf_core.hpp"

namespace gps_imu_fusion {

class EkfFusionNode : public rclcpp::Node {
public:
  EkfFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EkfFusionNode() = default;

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  std::unique_ptr<kai::KaiEkfCore> ekf_;
  
  struct UTMCoordinate {
    double easting;  
    double northing;  
    int zone;
    char band;
  };
  
  sensor_msgs::msg::NavSatFix latest_gnss_;
  geometry_msgs::msg::TwistWithCovarianceStamped latest_gnss_vel_;
  sensor_msgs::msg::Imu latest_imu_;
  UTMCoordinate latest_utm_;

  // -------------------- 3. Path 메시지 저장 변수 추가 --------------------
  nav_msgs::msg::Path path_msg_;
  
  bool received_gnss_ = false;
  bool received_gnss_vel_ = false;
  bool received_imu_ = false;
  
  void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void gnssVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  UTMCoordinate llToUtm(double lat, double lon);
  
  double calculateCourse(double vx, double vy);

  void updateAndPublish();
  void publishOdometry();
  void publishTransform();
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string world_frame_id_;
  std::string base_frame_id_;
  std::string gnss_frame_id_;
  std::string imu_frame_id_;
  double update_rate_;
  double mag_declination_;
  bool use_magnetic_declination_;
  bool publish_tf_;
  bool use_gnss_heading_;
  double min_speed_for_gnss_heading_;
  double zupt_speed_threshold_;
  double zupt_noise_scale_;
  double zupt_gyro_threshold_;
  double zupt_accel_threshold_;
  double reference_altitude_;
  
  bool origin_set_ = false;
  double origin_utm_x_ = 0.0;
  double origin_utm_y_ = 0.0;
  
  std::mutex data_mutex_;
  bool imu_stationary_flag_ = false;
  bool stationary_latched_ = true;  // 시작 시 정지 가정
  rclcpp::Time stationary_candidate_start_;
  rclcpp::Time moving_candidate_start_;
  double zupt_hold_time_ = 0.3;     // s
  double zupt_release_time_ = 0.3;  // s
  double last_speed_ = 0.0;
  float last_gps_heading_ = 0.0;  // GPS heading 저장용
  bool has_gnss_speed_ = false;
  
  // 전환 상태 최소 유지 시간 관련
  rclcpp::Time transition_state_start_;  // 전환 상태 진입 시간
  double transition_min_hold_time_ = 1.0;  // 전환 상태 최소 유지 시간 (s)
  bool in_transition_state_ = false;  // 전환 상태 플래그
  
  // 정지 상태 IMU 업데이트 decimation
  int stationary_imu_skip_counter_ = 0;  // 정지 상태 IMU 스킵 카운터
  int stationary_imu_decimation_factor_ = 10;  // 정지 상태에서 10번 중 1번만 업데이트
  
  rclcpp::Time last_update_time_;
  
  void loadParameters();
  
  void configureEkfParameters();
  
};

} 

#endif