/* global_yaw_complementary_node.cpp
 * ─────────────────────────────────────────────────────────────
 * /local_xy        → GPS‑기반 로컬 카르테시안 좌표 (PointStamped)
 * /imu/processed   → 보정된 IMU 메시지 (Imu)
 * 퍼블리시         → /global_yaw (std_msgs/Float32, [rad])
 *
 * ─ 파라미터 ─
 *   v_min        [m/s]  : GPS 헤딩을 신뢰할 최소 속도
 *   dist_min     [m]    : 초기화용 최소 이동 거리
 *   k_corr       [-]    : GPS 보정 게인 (0‥1)
 *   gps_timeout  [s]    : GPS 헤딩 최신성 제한
 *   debug        [bool] : DEBUG 로그 출력 여부
 *
 *   예시 YAML
 *   ───────────
 *   global_yaw_complementary:
 *     ros__parameters:
 *       v_min:        0.5
 *       dist_min:     1.0
 *       k_corr:       0.02
 *       gps_timeout:  1.0
 *       debug:        false
 * ─────────────────────────────────────────────────────────────
 */

 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/point_stamped.hpp>
 #include <sensor_msgs/msg/imu.hpp>
 #include <std_msgs/msg/float32.hpp>
 #include <cmath>
 
 namespace /* anonymous */ {
 inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
 }  // namespace
 
 class GlobalYawComplementary : public rclcpp::Node
 {
 public:
   GlobalYawComplementary() : Node("global_yaw_complementary")
   {
     /* ─── 파라미터 선언 ─── */
     v_min_       = declare_parameter<double>("v_min",       0.5);
     dist_min_    = declare_parameter<double>("dist_min",    1.0);
     k_corr_      = declare_parameter<double>("k_corr",      0.02);
     gps_timeout_ = declare_parameter<double>("gps_timeout", 1.0);
     debug_       = declare_parameter<bool  >("debug",       false);
 
     /* ─── 토픽 I/O ─── */
     sub_xy_ = create_subscription<geometry_msgs::msg::PointStamped>(
       "/local_xy", 10,
       std::bind(&GlobalYawComplementary::xyCallback, this, std::placeholders::_1));
 
     sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
       "/imu/processed", rclcpp::SensorDataQoS(),
       std::bind(&GlobalYawComplementary::imuCallback, this, std::placeholders::_1));
 
     pub_yaw_ = create_publisher<std_msgs::msg::Float32>("/global_yaw", 10);
 
     RCLCPP_INFO(get_logger(), "global_yaw_complementary_node launched.");
   }
 
 private:
   /* ===== GPS 콜백 ===== */
   void xyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
   {
     const double x = msg->point.x;
     const double y = msg->point.y;
     const double t = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
 
     if (has_prev_xy_) {
       const double dx   = x - prev_x_;
       const double dy   = y - prev_y_;
       const double dist = std::hypot(dx, dy);
       if (dist < 1e-6) return;  // 잡음 무시
 
       /* 속도 계산 */
       double speed = 0.0;
       if (last_xy_time_ > 0.0) {
         const double dt = t - last_xy_time_;
         if (dt > 0.0) speed = dist / dt;
       }
 
       const double yaw_gps = wrap(std::atan2(dy, dx));
 
       /* 초기 yaw 결정 */
       if (!init_done_) {
         cum_dist_ += dist;
         if (cum_dist_ >= dist_min_ && speed >= v_min_) {
           yaw_        = yaw_gps;
           init_done_  = true;
           RCLCPP_INFO(get_logger(), "initial yaw = %.2f°", yaw_ * 180.0 / M_PI);
         }
       }
       /* 초기화 후 GPS yaw 기록 (속도 조건 만족 시) */
       else if (speed >= v_min_) {
         last_gps_yaw_  = yaw_gps;
         last_gps_time_ = t;
         gps_valid_     = true;
       }
     }
 
     /* 상태 업데이트 */
     prev_x_        = x;
     prev_y_        = y;
     has_prev_xy_   = true;
     last_xy_time_  = t;
   }
 
   /* ===== IMU 콜백 ===== */
   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
   {
     if (!init_done_) return;  // yaw 초기화 前엔 무시
 
     const double t = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
 
     if (prev_imu_time_ < 0.0) {  // 첫 샘플
       prev_imu_time_ = t;
       return;
     }
 
     const double dt = t - prev_imu_time_;
     prev_imu_time_  = t;
     if (dt <= 0.0 || dt > 0.2) return;  // 5 Hz 미만 or 시간 역전
 
     /* ➊ IMU 적분 */
     yaw_ = wrap(yaw_ + msg->angular_velocity.z * dt);
 
     /* ➋ GPS 보정 (최근값이 유효할 때만) */
     if (gps_valid_ && (t - last_gps_time_) <= gps_timeout_) {
       const double delta = wrap(last_gps_yaw_ - yaw_);
       yaw_ = wrap(yaw_ + k_corr_ * delta);
     } else {
       gps_valid_ = false;  // 타임아웃
     }
 
     /* 퍼블리시 */
     std_msgs::msg::Float32 out;
     out.data = static_cast<float>(yaw_);
     pub_yaw_->publish(out);
 
     if (debug_) {
       RCLCPP_DEBUG(get_logger(), "yaw = %.2f°", yaw_ * 180.0 / M_PI);
     }
   }
 
   /* ─── 멤버 변수 ─── */
   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_xy_;
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            sub_imu_;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              pub_yaw_;
 
   /* 파라미터 */
   double v_min_, dist_min_, k_corr_, gps_timeout_;
   bool   debug_;
 
   /* GPS/로컬 좌표 상태 */
   double prev_x_{0.0}, prev_y_{0.0};
   bool   has_prev_xy_{false};
   double last_xy_time_{-1.0};
   double cum_dist_{0.0};
 
   /* IMU 상태 */
   double prev_imu_time_{-1.0};
   double yaw_{0.0};
   bool   init_done_{false};
 
   /* GPS yaw 보정용 */
   double last_gps_yaw_{0.0};
   double last_gps_time_{-1.0};
   bool   gps_valid_{false};
 };
 
 /* ===== main ===== */
 int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<GlobalYawComplementary>());
   rclcpp::shutdown();
   return 0;
 }
 