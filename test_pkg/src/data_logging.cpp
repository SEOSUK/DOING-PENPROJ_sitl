#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <functional>
#include <chrono>
#include <cmath>  // M_PI
#include <limits>

using std::placeholders::_1;
using steady_clk = std::chrono::steady_clock;

class DataLoggingMsg : public rclcpp::Node
{
public:
  DataLoggingMsg() : Node("data_logging_msg")
  {
    // QoS
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 6), qos_profile);

    // Subscribers
    cf_battery_voltage_subscriber_ =
      this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
        "/cf2/MJ_Battery", qos,
        std::bind(&DataLoggingMsg::cf_battery_voltage_callback, this, _1));

    cf_SU_force_input_subscriber_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pen/global_SU_Force_input", qos,
        std::bind(&DataLoggingMsg::cf_force_input_callback, this, _1));

    cf_MJ_force_input_subscriber_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pen/global_MJ_Force_input", qos,
        std::bind(&DataLoggingMsg::cf_MJ_force_input_callback, this, _1));

    cf_raw_force_input_subscriber_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pen/global_raw_Force_input", qos,
        std::bind(&DataLoggingMsg::cf_raw_force_input_callback, this, _1));

    cf_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pen/pose", qos,
        std::bind(&DataLoggingMsg::cf_pose_callback, this, _1));

    // Publisher
    data_logging_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging_msg", qos);

    // Loop (100 Hz)
    const double control_loop_hz = 100.0;
    auto period = std::chrono::duration<double>(1.0 / control_loop_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DataLoggingMsg::control_loop, this));

    // 시간 기준점
    t0_ = steady_clk::now();
  }

private:
  void cf_battery_voltage_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg->values.empty()) {
      battery_voltage_ = msg->values[0];
      has_battery_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "MJ_Battery message has empty values[]");
    }
  }

  void cf_force_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {
      global_SU_force_input_[0] = msg->data[0];
      global_SU_force_input_[1] = msg->data[1];
      global_SU_force_input_[2] = msg->data[2];
      has_force_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "global_Force_input size < 3");
    }
  }

  void cf_MJ_force_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {
      global_MJ_force_input_[0] = msg->data[0];
      global_MJ_force_input_[1] = msg->data[1];
      global_MJ_force_input_[2] = msg->data[2];
      has_force_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "global_Force_input size < 3");
    }
  }  

  void cf_raw_force_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {
      global_raw_force_input_[0] = msg->data[0];
      global_raw_force_input_[1] = msg->data[1];
      global_raw_force_input_[2] = msg->data[2];
      has_force_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "global_Force_input size < 3");
    }
  }


  void cf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Position
    global_xyz_meas_[0] = msg->pose.position.x;
    global_xyz_meas_[1] = msg->pose.position.y;
    global_xyz_meas_[2] = msg->pose.position.z;

    // Orientation -> roll/pitch/yaw + continuous yaw
    const auto &qmsg = msg->pose.orientation;

    // 간단 가드: NaN/비정상 쿼터니언 무시
    if (!std::isfinite(qmsg.x) || !std::isfinite(qmsg.y) ||
        !std::isfinite(qmsg.z) || !std::isfinite(qmsg.w)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Pose quaternion has NaN/Inf -> ignored");
      return;
    }

    tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    double delta = y - prev_yaw_;
    if (delta > M_PI)       yaw_offset_ -= 2.0 * M_PI;
    else if (delta < -M_PI) yaw_offset_ += 2.0 * M_PI;

    roll_ = r;
    pitch_ = p;
    yaw_ = y;
    yaw_continuous_ = y + yaw_offset_;
    prev_yaw_ = y;

    // tf2 -> Eigen (행 단위로 복사)
    tf2::Vector3 row0 = m.getRow(0), row1 = m.getRow(1), row2 = m.getRow(2);
    R_B_ << row0.x(), row0.y(), row0.z(),
            row1.x(), row1.y(), row1.z(),
            row2.x(), row2.y(), row2.z();
  }

  void control_loop()
  {
    // 경과 시간[s] (steady clock 기준, 단조증가)
    timer_tick++;
    timer_real = timer_tick / 100;
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(17);  // 선택: 성능 최적화

    // [0] time(s)
    out.data.push_back(timer_real);

    // [1] Vbat
    out.data.push_back(battery_voltage_);

    // [2..4] Fx,Fy,Fz
    out.data.push_back(global_SU_force_input_[0]);
    out.data.push_back(global_SU_force_input_[1]);
    out.data.push_back(global_SU_force_input_[2]);

    // [2..4] Fx,Fy,Fz
    out.data.push_back(global_MJ_force_input_[0]);
    out.data.push_back(global_MJ_force_input_[1]);
    out.data.push_back(global_MJ_force_input_[2]);

    // [2..4] Fx,Fy,Fz
    out.data.push_back(global_raw_force_input_[0]);
    out.data.push_back(global_raw_force_input_[1]);
    out.data.push_back(global_raw_force_input_[2]);
    
    // [5..7] x,y,z
    out.data.push_back(global_xyz_meas_[0]);
    out.data.push_back(global_xyz_meas_[1]);
    out.data.push_back(global_xyz_meas_[2]);

    // [8..10] roll,pitch,yaw_cont
    out.data.push_back(roll_);
    out.data.push_back(pitch_);
    out.data.push_back(yaw_continuous_);

    data_logging_pub_->publish(out);
  }

  // ROS
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_battery_voltage_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_raw_force_input_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_SU_force_input_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_MJ_force_input_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;


  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_logging_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // States
  Eigen::Vector3d global_SU_force_input_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_MJ_force_input_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_raw_force_input_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_xyz_meas_    = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_B_ = Eigen::Matrix3d::Identity();

  double battery_voltage_{0.0};
  bool has_battery_{false};
  bool has_force_{false};

  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double yaw_offset_{0.0}, prev_yaw_{0.0}, yaw_continuous_{0.0};

  double timer_tick;
  double timer_real;

  // 시간 기준
  steady_clk::time_point t0_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLoggingMsg>());
  rclcpp::shutdown();
  return 0;
}
