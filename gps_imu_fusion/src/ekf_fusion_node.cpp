#include "gps_imu_fusion/ekf_fusion_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace gps_imu_fusion {

EkfFusionNode::EkfFusionNode(const rclcpp::NodeOptions& options)
: Node("ekf_fusion_node", options), last_update_time_(this->now()) {
  loadParameters();
  
  ekf_ = std::make_unique<kai::KaiEkfCore>();
  
  configureEkfParameters();
  
  gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/ublox_gps_node/fix", 10, std::bind(&EkfFusionNode::gnssCallback, this, std::placeholders::_1));
    
  gnss_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/ublox_gps_node/fix_velocity", 10, std::bind(&EkfFusionNode::gnssVelCallback, this, std::placeholders::_1));
    
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/processed", rclcpp::SensorDataQoS(), std::bind(&EkfFusionNode::imuCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose/filtered", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / update_rate_),
    std::bind(&EkfFusionNode::updateAndPublish, this));
    
  path_msg_.header.frame_id = world_frame_id_;
    
  RCLCPP_INFO(this->get_logger(), "EKF Fusion Node initialized");
  RCLCPP_INFO(this->get_logger(), "Listening to GNSS on: %s", gnss_sub_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Listening to GNSS velocity on: %s", gnss_vel_sub_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Listening to IMU on: %s", imu_sub_->get_topic_name());
}

void EkfFusionNode::loadParameters() {
  if (!this->has_parameter("world_frame_id")) {
    RCLCPP_INFO(this->get_logger(), "Parameters not loaded from file, using defaults");
    
    this->declare_parameter("world_frame_id", "map");
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("gnss_frame_id", "gps");
    this->declare_parameter("imu_frame_id", "imu_link");
    this->declare_parameter("update_rate", 50.0); 
    this->declare_parameter("mag_declination", 0.0); 
    this->declare_parameter("use_magnetic_declination", false);
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("use_gnss_heading", true);
    this->declare_parameter("min_speed_for_gnss_heading", 0.5);  
  this->declare_parameter("zupt_speed_threshold", 0.15);
  this->declare_parameter("zupt_noise_scale", 0.01);
    this->declare_parameter("reference_altitude", 39.5);
    // ZUPT IMU 기준 임계치
    this->declare_parameter("zupt_gyro_threshold", 0.02);
    this->declare_parameter("zupt_accel_threshold", 0.2);
    this->declare_parameter("accel_noise", 0.05);
    this->declare_parameter("gyro_noise", 0.00175);
    this->declare_parameter("accel_bias_noise", 0.01);
    this->declare_parameter("gyro_bias_noise", 0.00025);
    this->declare_parameter("accel_bias_tau", 100.0);
    this->declare_parameter("gyro_bias_tau", 50.0);
    this->declare_parameter("gps_pos_noise_ne", 3.0);
    this->declare_parameter("gps_pos_noise_d", 6.0);
    this->declare_parameter("gps_vel_noise_ne", 0.5);
    this->declare_parameter("gps_vel_noise_d", 1.0);
    this->declare_parameter("init_pos_unc", 10.0);
    this->declare_parameter("init_vel_unc", 1.0);
    this->declare_parameter("init_att_unc", 0.34906);
    this->declare_parameter("init_hdg_unc", 3.14159);
    this->declare_parameter("init_accel_bias_unc", 0.981);
    this->declare_parameter("init_gyro_bias_unc", 0.01745);
  } else {
    RCLCPP_INFO(this->get_logger(), "Parameters loaded from file");
  }
  
  world_frame_id_ = this->get_parameter("world_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  gnss_frame_id_ = this->get_parameter("gnss_frame_id").as_string();
  imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
  update_rate_ = this->get_parameter("update_rate").as_double();
  mag_declination_ = this->get_parameter("mag_declination").as_double() * M_PI / 180.0;  
  use_magnetic_declination_ = this->get_parameter("use_magnetic_declination").as_bool();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  use_gnss_heading_ = this->get_parameter("use_gnss_heading").as_bool();
  min_speed_for_gnss_heading_ = this->get_parameter("min_speed_for_gnss_heading").as_double();
  zupt_speed_threshold_ = this->get_parameter("zupt_speed_threshold").as_double();
  zupt_noise_scale_ = this->get_parameter("zupt_noise_scale").as_double();
  reference_altitude_ = this->get_parameter("reference_altitude").as_double();
  zupt_gyro_threshold_ = this->get_parameter("zupt_gyro_threshold").as_double();
  zupt_accel_threshold_ = this->get_parameter("zupt_accel_threshold").as_double();
  
  RCLCPP_INFO(this->get_logger(), "Update rate: %.1f Hz", update_rate_);
  RCLCPP_INFO(this->get_logger(), "Magnetic declination: %.2f deg (%s)", 
              this->get_parameter("mag_declination").as_double(),
              use_magnetic_declination_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "GNSS heading: %s (min speed: %.1f m/s)", 
              use_gnss_heading_ ? "enabled" : "disabled",
              min_speed_for_gnss_heading_);
  RCLCPP_INFO(this->get_logger(), "ZUPT: threshold=%.2f m/s, noise_scale=%.3f, gyro_thr=%.3f rad/s, accel_thr=%.3f m/s^2", 
              zupt_speed_threshold_, zupt_noise_scale_, zupt_gyro_threshold_, zupt_accel_threshold_);
  RCLCPP_INFO(this->get_logger(), "Reference altitude: %.1f m", reference_altitude_);
}

void EkfFusionNode::configureEkfParameters() {
  kai::EkfParams params;
  params.accel_noise = this->get_parameter("accel_noise").as_double();
  params.gyro_noise = this->get_parameter("gyro_noise").as_double();
  params.accel_bias_noise = this->get_parameter("accel_bias_noise").as_double();
  params.gyro_bias_noise = this->get_parameter("gyro_bias_noise").as_double();
  params.accel_bias_tau = this->get_parameter("accel_bias_tau").as_double();
  params.gyro_bias_tau = this->get_parameter("gyro_bias_tau").as_double();
  params.gps_pos_noise_ne = this->get_parameter("gps_pos_noise_ne").as_double();
  params.gps_pos_noise_d = this->get_parameter("gps_pos_noise_d").as_double();
  params.gps_vel_noise_ne = this->get_parameter("gps_vel_noise_ne").as_double();
  params.gps_vel_noise_d = this->get_parameter("gps_vel_noise_d").as_double();
  params.init_pos_unc = this->get_parameter("init_pos_unc").as_double();
  params.init_vel_unc = this->get_parameter("init_vel_unc").as_double();
  params.init_att_unc = this->get_parameter("init_att_unc").as_double();
  params.init_hdg_unc = this->get_parameter("init_hdg_unc").as_double();
  params.init_accel_bias_unc = this->get_parameter("init_accel_bias_unc").as_double();
  params.init_gyro_bias_unc = this->get_parameter("init_gyro_bias_unc").as_double();
  
  ekf_->setParameters(params);
  
  RCLCPP_INFO(this->get_logger(), "EKF parameters configured:");
  RCLCPP_INFO(this->get_logger(), "  GPS position noise (NE/D): %.1f / %.1f m", 
              params.gps_pos_noise_ne, params.gps_pos_noise_d);
  RCLCPP_INFO(this->get_logger(), "  GPS velocity noise (NE/D): %.2f / %.2f m/s", 
              params.gps_vel_noise_ne, params.gps_vel_noise_d);
  RCLCPP_INFO(this->get_logger(), "  IMU accel/gyro noise: %.3f m/s² / %.5f rad/s", 
              params.accel_noise, params.gyro_noise);
              
}


EkfFusionNode::UTMCoordinate EkfFusionNode::llToUtm(double lat, double lon) {
  UTMCoordinate result;
  int zone;
  bool northp;
  
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, result.easting, result.northing);
  result.zone = zone;
  result.band = northp ? 'N' : 'S';
  
  return result;
}

double EkfFusionNode::calculateCourse(double vx, double vy) {
  return std::atan2(vy, vx);
}

void EkfFusionNode::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(this->get_logger(), "Received GNSS data: lat=%.6f, lon=%.6f, alt=%.2f",
               msg->latitude, msg->longitude, msg->altitude);
  
  latest_gnss_ = *msg;
  
  latest_utm_ = llToUtm(msg->latitude, msg->longitude);
  
  RCLCPP_DEBUG(this->get_logger(), "Converted to UTM: zone=%d%c, easting=%.2f, northing=%.2f",
               latest_utm_.zone, latest_utm_.band, latest_utm_.easting, latest_utm_.northing);
  
  if (!origin_set_) {
    // 건국대 일감호를 원점으로 설정
    double ref_lat = 37.540091;
    double ref_lon = 127.076555;
    auto ref_utm = llToUtm(ref_lat, ref_lon);
    origin_utm_x_ = ref_utm.easting;
    origin_utm_y_ = ref_utm.northing;
    origin_set_ = true;
    RCLCPP_INFO(this->get_logger(), "Origin set at Konkuk University Ilgamho: %.2f, %.2f (zone %d%c)", 
                origin_utm_x_, origin_utm_y_, ref_utm.zone, ref_utm.band);
  }
  
  kai::GpsCoordinate coor;
  coor.lat = msg->latitude * M_PI / 180.0;  
  coor.lon = msg->longitude * M_PI / 180.0;
  coor.alt = msg->altitude;  
  
  RCLCPP_DEBUG(this->get_logger(), "Updating EKF with GPS coordinates: %.6f, %.6f rad, %.2f m", 
               coor.lat, coor.lon, coor.alt);
  ekf_->gpsCoordinateUpdateEkf(coor);
  
  // GPS 공분산을 EKF에 전달
  if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN ||
      msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    std::array<double, 9> pos_cov;
    for(int i = 0; i < 9; i++) {
      pos_cov[i] = msg->position_covariance[i];
    }
    
    // 속도 공분산 (최신 속도 메시지에서)
    std::array<double, 9> vel_cov = {0};
    if (received_gnss_vel_) {
      for(int i = 0; i < 9; i++) {
        vel_cov[i] = latest_gnss_vel_.twist.covariance[i];
      }
    }
    
    ekf_->setGpsCovariance(pos_cov, vel_cov);
    RCLCPP_DEBUG(this->get_logger(), "GPS covariance updated: pos_var=[%.4f, %.4f, %.4f]", 
                 pos_cov[0], pos_cov[4], pos_cov[8]);
  }
  
  if (!received_gnss_) {
    RCLCPP_INFO(this->get_logger(), "First GNSS data received");
  }
  received_gnss_ = true;
}

void EkfFusionNode::gnssVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(this->get_logger(), "Received GNSS velocity data: vx=%.3f, vy=%.3f, vz=%.3f m/s",
               msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  
  latest_gnss_vel_ = *msg;
  
  kai::GpsVelocity vel;
  
  vel.vN = msg->twist.twist.linear.x;  
  vel.vE = msg->twist.twist.linear.y;  
  vel.vD = -msg->twist.twist.linear.z; 
  
  RCLCPP_DEBUG(this->get_logger(), "Updating EKF with GPS velocity: vN=%.3f, vE=%.3f, vD=%.3f m/s", 
               vel.vN, vel.vE, vel.vD);
  ekf_->gpsVelocityUpdateEkf(vel);
  
  // GPS 속도로부터 heading 계산 및 설정
  double speed = std::sqrt(vel.vN * vel.vN + vel.vE * vel.vE);
  has_gnss_speed_ = true;
  last_speed_ = speed;
  
  // 0.08m/s 기준으로 ZUPT 및 GPS heading 제어
  const double SPEED_THRESHOLD = 0.08;  // 0.08m/s 기준
  
  if (speed < SPEED_THRESHOLD) {
    // 정지 상태: ZUPT 활성화하여 yaw drift 방지
    ekf_->setStationary(true);
    ekf_->setZuptNoiseScale(0.3f);  // 프로세스 노이즈 적당히 줄임 (너무 작으면 안됨)
    ekf_->setGpsHeading(0.0f, false);  // GPS heading 사용 안 함
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "ZUPT ACTIVE: speed %.3f m/s < %.3f m/s, yaw update stopped", 
                         speed, SPEED_THRESHOLD);
  } else {
    // 이동 상태: ZUPT 비활성화, GPS heading 사용
    ekf_->setStationary(false);
    ekf_->setZuptNoiseScale(1.0f);  // 프로세스 노이즈 원상복구
    
    if (use_gnss_heading_) {
      float gps_heading = std::atan2(vel.vE, vel.vN);  // 북쪽 기준 heading
      ekf_->setGpsHeading(gps_heading, true);
      last_gps_heading_ = gps_heading;  // GPS heading 저장
      RCLCPP_DEBUG(this->get_logger(), "GPS heading set: %.2f rad (%.1f deg) from velocity [vN=%.2f, vE=%.2f], speed=%.2f m/s", 
                   gps_heading, gps_heading * 180.0 / M_PI, vel.vN, vel.vE, speed);
    }
  }
  
  if (!received_gnss_vel_) {
    RCLCPP_INFO(this->get_logger(), "First GNSS velocity data received");
  }
  received_gnss_vel_ = true;
}

void EkfFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // -------------------- 이 함수를 수정합니다 --------------------
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_DEBUG(this->get_logger(), "Received IMU data: gyro=[%.3f, %.3f, %.3f] rad/s, accel=[%.3f, %.3f, %.3f] m/s²",
               msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
               msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  
  latest_imu_ = *msg;
  
  if (!received_imu_) {
    RCLCPP_INFO(this->get_logger(), "First IMU data received");
  }
  received_imu_ = true;
  
  // --- 여기에 가드 조건 추가 ---
  // GPS 위치 및 속도 데이터가 아직 도착하지 않았다면, 
  // EKF 업데이트를 시작하지 않고 함수를 종료합니다.
  if (!received_gnss_ || !received_gnss_vel_) {
    return;
  }
  // --- 여기까지 추가 ---

  kai::ImuData imu_data;
  
  imu_data.gyroX = msg->angular_velocity.x;
  imu_data.gyroY = msg->angular_velocity.y;
  imu_data.gyroZ = msg->angular_velocity.z;
  
  imu_data.accX = msg->linear_acceleration.x;
  imu_data.accY = msg->linear_acceleration.y;
  imu_data.accZ = msg->linear_acceleration.z;
  
  imu_data.hX = 1.0;
  imu_data.hY = 0.0;
  imu_data.hZ = 0.0;
  
  if (use_magnetic_declination_) {
    float cos_dec = cos(mag_declination_);
    float sin_dec = sin(mag_declination_);
    float hx_temp = imu_data.hX;
    imu_data.hX = hx_temp * cos_dec - imu_data.hY * sin_dec;
    imu_data.hY = hx_temp * sin_dec + imu_data.hY * cos_dec;
    
    RCLCPP_DEBUG(this->get_logger(), "Applied magnetic declination: %.2f deg, hX=%.2f, hY=%.2f", 
                 mag_declination_ * 180.0 / M_PI, imu_data.hX, imu_data.hY);
  }
  
  // 타임스탬프 처리: 헤더에 타임스탬프가 없으면 현재 시간 사용
  uint64_t time_us;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    auto now = this->now();
    time_us = now.seconds() * 1000000LL + now.nanoseconds() / 1000LL;
    RCLCPP_WARN_ONCE(this->get_logger(), "IMU message has no timestamp, using current time");
  } else {
    time_us = msg->header.stamp.sec * 1000000LL + msg->header.stamp.nanosec / 1000LL;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Updating EKF with IMU data, timestamp: %ld us", time_us);
  
  // IMU 기반 정지 감지는 비활성화 - GPS 속도로만 판단
  // (향후 필요시 OR 조건으로 추가 가능)
  
  ekf_->imuUpdateEkf(time_us, imu_data);
  
  // IMU 공분산을 EKF에 전달
  std::array<double, 9> gyro_cov = {0};
  std::array<double, 9> accel_cov = {0};
  
  // angular_velocity_covariance가 유효한 경우
  if (msg->angular_velocity_covariance[0] > 0) {
    for(int i = 0; i < 9; i++) {
      gyro_cov[i] = msg->angular_velocity_covariance[i];
    }
  }
  
  // linear_acceleration_covariance가 유효한 경우
  if (msg->linear_acceleration_covariance[0] > 0) {
    for(int i = 0; i < 9; i++) {
      accel_cov[i] = msg->linear_acceleration_covariance[i];
    }
  }
  
  // 공분산이 하나라도 유효하면 EKF에 전달
  if (gyro_cov[0] > 0 || accel_cov[0] > 0) {
    ekf_->setImuCovariance(gyro_cov, accel_cov);
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "IMU covariance updated: gyro_var=[%.6f, %.6f, %.6f], accel_var=[%.6f, %.6f, %.6f]", 
                          gyro_cov[0], gyro_cov[4], gyro_cov[8],
                          accel_cov[0], accel_cov[4], accel_cov[8]);
  }
}

void EkfFusionNode::updateAndPublish() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (!received_gnss_ || !received_gnss_vel_ || !received_imu_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                        "Waiting for sensor data: GNSS=%s, GNSS Vel=%s, IMU=%s", 
                        received_gnss_ ? "received" : "waiting",
                        received_gnss_vel_ ? "received" : "waiting",
                        received_imu_ ? "received" : "waiting");
    return;
  }
  
  bool is_initialized = ekf_->initialized();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                      "EKF initialization status: %s", is_initialized ? "INITIALIZED" : "NOT INITIALIZED");
  
  if (!is_initialized) {
    // 이 부분은 이제 imuCallback의 가드 조건 때문에 거의 호출되지 않지만,
    // 만약을 위해 안전장치로 남겨둡니다.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "EKF not initialized yet, though all sensors are being received. Waiting for IMU callback to trigger initialization.");
    return;
  }
  
  publishOdometry();
  if (publish_tf_) {
    publishTransform();
  }
  
  last_update_time_ = this->now();
}

void EkfFusionNode::publishOdometry() {
  if (!ekf_->initialized()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "EKF not initialized yet, skipping odometry publication");
    return;
  }
  
  rclcpp::Time now = this->now();
  
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = base_frame_id_;
  
  if (origin_set_) {
    // EKF에서 추정한 위치를 가져옴 (라디안 단위)
    double ekf_lat_rad = ekf_->getLatitude_rad();
    double ekf_lon_rad = ekf_->getLongitude_rad();
    double ekf_alt_m = ekf_->getAltitude_m();
    
    // 라디안을 도(degree)로 변환
    double ekf_lat_deg = ekf_lat_rad * 180.0 / M_PI;
    double ekf_lon_deg = ekf_lon_rad * 180.0 / M_PI;
    
    // EKF 추정 위치를 UTM으로 변환
    UTMCoordinate ekf_utm = llToUtm(ekf_lat_deg, ekf_lon_deg);
    
    // 로컬 좌표계로 변환 (원점 기준)
    odom.pose.pose.position.x = ekf_utm.easting - origin_utm_x_;
    odom.pose.pose.position.y = ekf_utm.northing - origin_utm_y_;
    odom.pose.pose.position.z = ekf_alt_m - reference_altitude_; 
    
    RCLCPP_DEBUG(this->get_logger(), "Position relative to origin: x=%.2f, y=%.2f, z=%.2f m", 
                 odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Origin not set yet, using zero position");
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
  }
  
  float roll = ekf_->getRoll_rad();
  float pitch = ekf_->getPitch_rad();
  float heading;
  
  // 현재 GPS 속도 계산 (최신 데이터 사용)
  double gps_speed = std::sqrt(
    latest_gnss_vel_.twist.twist.linear.x * latest_gnss_vel_.twist.twist.linear.x +
    latest_gnss_vel_.twist.twist.linear.y * latest_gnss_vel_.twist.twist.linear.y);
  
  if (use_gnss_heading_ && gps_speed > 0.08) {
    // 매번 최신 GPS 속도로부터 heading 계산 - main처럼 동작
    heading = calculateCourse(latest_gnss_vel_.twist.twist.linear.x, latest_gnss_vel_.twist.twist.linear.y);
    RCLCPP_DEBUG(this->get_logger(), "Using GNSS heading: %.2f rad from velocity data", heading);
  } else {
    heading = ekf_->getHeading_rad();
    RCLCPP_DEBUG(this->get_logger(), "Using EKF estimated heading: %.2f rad", heading);
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Attitude: roll=%.2f, pitch=%.2f, heading=%.2f rad", roll, pitch, heading);
  
  // NaN 방지를 위해 roll, pitch, heading 값 확인 (안전장치로 유지)
  if (std::isnan(roll) || std::isnan(pitch) || std::isnan(heading)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "NaN detected in roll, pitch, or heading. Skipping odometry publication.");
    return;
  }

  tf2::Quaternion q;
  q.setRPY(roll, pitch, heading);
  
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  
  double vn = ekf_->getVelNorth_ms();
  double ve = ekf_->getVelEast_ms();
  double vd = ekf_->getVelDown_ms();
  
  odom.twist.twist.linear.x = vn;  
  odom.twist.twist.linear.y = ve;  
  odom.twist.twist.linear.z = -vd; 
  
  RCLCPP_DEBUG(this->get_logger(), "Velocity: vN=%.2f, vE=%.2f, vD=%.2f m/s", vn, ve, vd);
  
  odom.twist.twist.angular.x = latest_imu_.angular_velocity.x;
  odom.twist.twist.angular.y = latest_imu_.angular_velocity.y;
  odom.twist.twist.angular.z = latest_imu_.angular_velocity.z;
  
  // EKF 공분산 사용
  auto pos_cov = ekf_->getPositionCovariance();
  auto vel_cov = ekf_->getVelocityCovariance();
  auto orient_cov = ekf_->getOrientationCovariance();
  
  // 위치 공분산 설정 (3x3)
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      odom.pose.covariance[i * 6 + j] = pos_cov[i * 3 + j];
    }
  }
  
  // 자세 공분산 설정 (3x3, 오른쪽 아래)
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      odom.pose.covariance[(i+3) * 6 + (j+3)] = orient_cov[i * 3 + j];
    }
  }
  
  // 속도 공분산 설정 (3x3)
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      odom.twist.covariance[i * 6 + j] = vel_cov[i * 3 + j];
    }
  }
  
  // 각속도 공분산은 IMU 원본 사용 (EKF에서 추정하지 않음)
  if (latest_imu_.angular_velocity_covariance[0] > 0) {
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        odom.twist.covariance[(i+3) * 6 + (j+3)] = latest_imu_.angular_velocity_covariance[i * 3 + j];
      }
    }
  }

  odom_pub_->publish(odom);
  
  // 현재 속도 계산
  double current_speed = std::sqrt(
    odom.twist.twist.linear.x * odom.twist.twist.linear.x + 
    odom.twist.twist.linear.y * odom.twist.twist.linear.y);
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                      "Published odometry: pos=[%.2f, %.2f, %.2f], heading=%.2f deg, speed=%.2f m/s", 
                      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                      heading * 180.0 / M_PI, current_speed);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  pose_pub_->publish(pose);

  // 경로 업데이트 및 발행 코드
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = odom.header;
  current_pose.pose = odom.pose.pose;

  path_msg_.poses.push_back(current_pose);
  path_msg_.header.stamp = now;
  path_pub_->publish(path_msg_);
}

void EkfFusionNode::publishTransform() {
  if (!ekf_->initialized() || !origin_set_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                         "Not publishing transform: initialized=%s, origin_set=%s", 
                         ekf_->initialized() ? "true" : "false", 
                         origin_set_ ? "true" : "false");
    return;
  }

  rclcpp::Time now = this->now();

  float roll = ekf_->getRoll_rad();
  float pitch = ekf_->getPitch_rad();
  float heading;
  
  // 현재 GPS 속도 계산 (최신 데이터 사용)
  double gps_speed = std::sqrt(
    latest_gnss_vel_.twist.twist.linear.x * latest_gnss_vel_.twist.twist.linear.x +
    latest_gnss_vel_.twist.twist.linear.y * latest_gnss_vel_.twist.twist.linear.y);
  
  if (use_gnss_heading_ && gps_speed > 0.08) {
    // 매번 최신 GPS 속도로부터 heading 계산 - main처럼 동작
    heading = calculateCourse(latest_gnss_vel_.twist.twist.linear.x, latest_gnss_vel_.twist.twist.linear.y);
  } else {
    heading = ekf_->getHeading_rad();
  }

  // NaN 방지를 위해 roll, pitch, heading 값 확인 (안전장치로 유지)
  if (std::isnan(roll) || std::isnan(pitch) || std::isnan(heading)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "NaN detected in roll, pitch, or heading. Skipping transform publication.");
    return;
  }
  
  tf2::Quaternion q;
  q.setRPY(roll, pitch, heading);
  
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "odom";
  transform.child_frame_id = base_frame_id_;
  
  // EKF에서 추정한 위치를 가져옴 (라디안 단위)
  double ekf_lat_rad = ekf_->getLatitude_rad();
  double ekf_lon_rad = ekf_->getLongitude_rad();
  double ekf_alt_m = ekf_->getAltitude_m();
  
  // 라디안을 도(degree)로 변환
  double ekf_lat_deg = ekf_lat_rad * 180.0 / M_PI;
  double ekf_lon_deg = ekf_lon_rad * 180.0 / M_PI;
  
  // EKF 추정 위치를 UTM으로 변환
  UTMCoordinate ekf_utm = llToUtm(ekf_lat_deg, ekf_lon_deg);
  
  // 로컬 좌표계로 변환 (원점 기준)
  transform.transform.translation.x = ekf_utm.easting - origin_utm_x_;
  transform.transform.translation.y = ekf_utm.northing - origin_utm_y_;
  transform.transform.translation.z = ekf_alt_m - reference_altitude_; 
  
  transform.transform.rotation.w = q.w();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();

  tf_broadcaster_->sendTransform(transform);
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                       "Published transform: %s -> %s", 
                       world_frame_id_.c_str(), base_frame_id_.c_str());
}

}