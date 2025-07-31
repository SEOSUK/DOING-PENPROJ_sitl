#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CfCommunicatorSIM : public rclcpp::Node
{
public:
  CfCommunicatorSIM() : Node("cf_communicator_sim"),
  body_omega_dot_filter(3, 1, 1.0 / 50),
  global_xyz_vel_dot_filter(3, 1, 1.0 / 50),  
  global_xyz_meas_dot_filter(3, 1, 1.0 / 50),
  body_rpy_meas_dot_filter(3, 1, 1.0 / 50),
  thrust_data_filter(3, 1, 1.0 / 50)
// 두 번째 인자가 cof이다.
// cof를 늘리면(숫자를 키우면) delay가 줄어들고 노이즈가 커진다.
// cof를 줄이면(숫자를 줄아ㅣ면) 그 반대이다.

// 지금 sensor data의 딜레이가 어마무시하잖아??
// 그러니, 1. cof 튜닝 야물딱지게 하기.
// 2번: mob 결과에다가 low pass filter를 걸어서, phase를 끼워맞추기.

  {
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


    control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
    auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);


    // Subscribers
    cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", qos_settings,
      std::bind(&CfCommunicatorSIM::cf_pose_callback, this, _1));

    cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_velocity", qos_settings,
      std::bind(&CfCommunicatorSIM::cf_vel_callback, this, _1));

    cf_thrust_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_Command_thrust", qos_settings,
      std::bind(&CfCommunicatorSIM::cf_thrust_callback, this, _1));

    cf_torque_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/SEUK_pid_rate_cmd", qos_settings,
      std::bind(&CfCommunicatorSIM::cf_torque_callback, this, _1));      

    // Publishers
    pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pen/pose", qos_settings);
    acc_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/acc", qos_settings);
    vel_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/vel", qos_settings);
    thrust_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/thrust", qos_settings);
    omega_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/omega", qos_settings);
    alpha_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/alpha", qos_settings);
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/torque", qos_settings);

    // Timer
    timer_ = this->create_wall_timer(
      control_loop_period, std::bind(&CfCommunicatorSIM::timer_callback, this));


    }

private:
  // Callback: Pose

  void data_publish()
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();  // 🟢 타임스탬프 명시
    pose_msg.header.frame_id = "cf2";  // 🟢 프레임 명시 (TF를 위한 필수값)
    pose_msg.pose.position.x = global_xyz_meas[0];
    pose_msg.pose.position.y = global_xyz_meas[1];
    pose_msg.pose.position.z = global_xyz_meas[2];
    pose_msg.pose.orientation.x = global_quat_meas[0];
    pose_msg.pose.orientation.y = global_quat_meas[1];
    pose_msg.pose.orientation.z = global_quat_meas[2];
    pose_msg.pose.orientation.w = global_quat_meas[3];

    pose_pub_->publish(pose_msg);


    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data.push_back(global_xyz_vel_meas[0]);
    vel_msg.data.push_back(global_xyz_vel_meas[1]);
    vel_msg.data.push_back(global_xyz_vel_meas[2]);
    vel_pub_->publish(vel_msg);


    std_msgs::msg::Float64MultiArray acc_msg;
    acc_msg.data.push_back(global_xyz_acc(0));
    acc_msg.data.push_back(global_xyz_acc(1));
    acc_msg.data.push_back(global_xyz_acc(2));
    acc_pub_->publish(acc_msg);


    std_msgs::msg::Float64MultiArray thrust_msg;
    thrust_msg.data.push_back(global_command_Force[0]);
    thrust_msg.data.push_back(global_command_Force[1]);
    thrust_msg.data.push_back(global_command_Force[2]);
    thrust_pub_->publish(thrust_msg);

    std_msgs::msg::Float64MultiArray omega_msg;
    omega_msg.data.push_back(body_omega_meas(0));
    omega_msg.data.push_back(body_omega_meas(1));
    omega_msg.data.push_back(body_omega_meas(2));
    omega_pub_->publish(omega_msg);    

    std_msgs::msg::Float64MultiArray alpha_msg;
    alpha_msg.data.push_back(body_alpha_meas(0));
    alpha_msg.data.push_back(body_alpha_meas(1));
    alpha_msg.data.push_back(body_alpha_meas(2));
    alpha_pub_->publish(alpha_msg);      

    std_msgs::msg::Float64MultiArray torque_msg;
    torque_msg.data.push_back(torque_data_(0));
    torque_msg.data.push_back(torque_data_(1));
    torque_msg.data.push_back(torque_data_(2));
    torque_pub_->publish(torque_msg);      

    
  // // ✅ 상태 출력 추가 (topic echo 느낌)
  // RCLCPP_INFO(this->get_logger(),
  //             "POSE   [%.3f %.3f %.3f] [%.3f %.3f %.3f %.3f]",
  //             global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2],
  //             global_quat_meas[0], global_quat_meas[1], global_quat_meas[2], global_quat_meas[3]);

  // RCLCPP_INFO(this->get_logger(),
  //             "ACC    [%.3f %.3f %.3f]",
  //             global_xyz_acc(0), global_xyz_acc(1), global_xyz_acc(2));

  // RCLCPP_INFO(this->get_logger(),
  //             "VEL    [%.3f %.3f %.3f]",
  //             global_xyz_vel_meas(0), global_xyz_vel_meas(1), global_xyz_vel_meas(2));

  // RCLCPP_INFO(this->get_logger(),
  //             "THRUST [%.3f %.3f %.3f]",
  //             thrust_data_(0), thrust_data_(1), thrust_data_(2));

  // RCLCPP_INFO(this->get_logger(),
  //             "OMEGA  [%.3f %.3f %.3f] (rad/s)",
  //             body_omega_meas(0), body_omega_meas(1), body_omega_meas(2));
    
  }


  void cf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    global_xyz_meas[0] = msg->pose.position.x;
    global_xyz_meas[1] = msg->pose.position.y;
    global_xyz_meas[2] = msg->pose.position.z;
    global_quat_meas[0] = msg->pose.orientation.x;
    global_quat_meas[1] = msg->pose.orientation.y;
    global_quat_meas[2] = msg->pose.orientation.z;
    global_quat_meas[3] = msg->pose.orientation.w;


    tf2::Quaternion quat(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // Yaw 불연속 보정
    double delta_yaw = yaw - prev_yaw;
    if (delta_yaw > M_PI) {
        yaw_offset -= 2.0 * M_PI;  // -360도 보정
    } else if (delta_yaw < -M_PI) {
        yaw_offset += 2.0 * M_PI;  // +360도 보정
    }
    yaw_continuous = yaw + yaw_offset;  // 연속 yaw 업데이트
    prev_yaw = yaw;


    // RPY 업데이트
    body_rpy_meas[0] = roll;
    body_rpy_meas[1] = pitch;
    body_rpy_meas[2] = yaw_continuous;  // 보정된 Yaw 사용


    // Rotation matrix 업데이트
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_B(i, j) = mat[i][j];
        }
    }

    // TODO: R_B 라는 Rotation matrix 만들기. quaternion 조합해서 body to world frame 변환 매트릭스 만들어야 함.
    // global_vel = R_B * body_vel 이런 식으로 사용할 예정

  }

  // Callback: Velocity
  void cf_vel_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      body_xyz_vel_meas(i) = msg->values[i];
    
    global_xyz_vel_meas = R_B * body_xyz_vel_meas;
  }

  // Callback: Thrust
  void cf_thrust_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
      thrust_data_raw[2] = msg->values[0];
      thrust_data_ = thrust_data_filter.apply(thrust_data_raw);

      global_command_Force = R_B * thrust_data_ / 100000.0;
  }

  void cf_torque_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      torque_data_(i) = msg->values[i];
  }

  void numerical_calc()
  {

    global_xyz_vel_meas = global_xyz_meas_dot_filter.apply((global_xyz_meas - global_xyz_meas_prev) * control_loop_hz);
    global_xyz_meas_prev = global_xyz_meas;
    global_xyz_vel_dot_raw = (global_xyz_vel_meas - global_xyz_vel_meas_prev) * control_loop_hz;
    global_xyz_acc = global_xyz_vel_dot_filter.apply(global_xyz_vel_dot_raw);
    global_xyz_vel_meas_prev = global_xyz_vel_meas;
    body_xyz_acc = R_B.transpose() * global_xyz_acc;
    // pose to omega calculation, omega to alpha calculation
    // 시뮬 개억까로 인하여 각속도 데이터를 direct feedback 받을 수 없으니 일단 수치적으로 각도->각속도->각가속도 계산함


    body_rpy_meas_dot_raw = (body_rpy_meas - body_rpy_meas_prev) * control_loop_hz;
    body_rpy_meas_dot = body_rpy_meas_dot_filter.apply(body_rpy_meas_dot_raw);
    body_rpy_meas_prev = body_rpy_meas;

    double sin_roll = sin(body_rpy_meas[0]);
    double cos_roll = cos(body_rpy_meas[0]);

    double cos_pitch = cos(body_rpy_meas[1]);

    omega_eulerRate_Mapping_matrix << 1, sin_roll * tan(body_rpy_meas[1]), cos_roll * tan(body_rpy_meas[1]),
                                      0, cos_roll, -sin_roll,
                                    0, sin_roll / cos_pitch, cos_roll / cos_pitch;

    body_omega_meas = omega_eulerRate_Mapping_matrix * body_rpy_meas_dot;
    
    body_omega_dot_raw = (body_omega_meas - body_omega_prev) * control_loop_hz;
    body_alpha_meas = body_omega_dot_filter.apply(body_omega_dot_raw);
    body_omega_prev = body_omega_meas;


  }

  // Timer: Publishing
  void timer_callback()
  {

    numerical_calc();
    data_publish();

  }

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_thrust_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_torque_subscriber_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omega_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr alpha_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal Data (Eigen)
  Eigen::VectorXd global_xyz_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd global_xyz_meas_prev = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd global_quat_meas = Eigen::VectorXd::Zero(4);

  Eigen::VectorXd global_xyz_vel_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_xyz_vel_meas = Eigen::VectorXd::Zero(3);

Eigen::VectorXd global_xyz_vel_dot_raw = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd global_xyz_vel_meas_prev = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd global_xyz_acc = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_xyz_acc = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd body_rpy_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_rpy_meas_prev = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_rpy_meas_dot_raw = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_rpy_meas_dot = Eigen::VectorXd::Zero(3);
  Eigen::Matrix3d omega_eulerRate_Mapping_matrix;

  Eigen::VectorXd body_omega_dot_raw = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_omega_prev = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_omega_meas = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd body_alpha_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_alpha_prev = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd body_alpha_raw = Eigen::VectorXd::Zero(3);


  Eigen::Vector3d thrust_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_data_raw{Eigen::Vector3d::Zero()};
  Eigen::Vector3d torque_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d global_command_Force{Eigen::Vector3d::Zero()};

  Eigen::Matrix3d R_B;

  FilteredVector global_xyz_meas_dot_filter;
  FilteredVector body_rpy_meas_dot_filter;
  FilteredVector body_omega_dot_filter;  
  FilteredVector global_xyz_vel_dot_filter;
  FilteredVector thrust_data_filter;
  double prev_yaw, yaw_offset, yaw_continuous;
  double control_loop_hz, control_loop_period;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CfCommunicatorSIM>());
  rclcpp::shutdown();
  return 0;
}