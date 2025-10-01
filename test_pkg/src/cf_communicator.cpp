#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CfCommunicator : public rclcpp::Node
{
public:
  CfCommunicator()
  : Node("cf_communicator")
  {
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
    auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);

    num_control_loop_hz = this->declare_parameter<double>("num_control_loop_hz", 100.0);
    auto num_control_loop_period = std::chrono::duration<double>(1.0 / num_control_loop_hz);


    // Subscribers
    cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", qos_settings,
      std::bind(&CfCommunicator::cf_pose_callback, this, _1));

    cf_acc_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_stateEstimate_acc", qos_settings,
      std::bind(&CfCommunicator::cf_acc_callback, this, _1));

    cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_velocity", qos_settings,
      std::bind(&CfCommunicator::cf_vel_callback, this, _1));

    cf_thrust_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_Command_thrust", qos_settings,
      std::bind(&CfCommunicator::cf_thrust_callback, this, _1));

      
    cf_torque_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/SEUK_pid_rate_cmd", qos_settings,
      std::bind(&CfCommunicator::cf_torque_callback, this, _1));


    cf_omega_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_omega", qos_settings,
      std::bind(&CfCommunicator::cf_omega_callback, this, _1));

    cf_battery_voltage_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_Battery", qos_settings,
      std::bind(&CfCommunicator::cf_battery_voltage_callback, this, _1));



    // ADD0915
    cf_world_force_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/su_wrench_world_force", qos_settings,
      std::bind(&CfCommunicator::cf_world_force_callback, this, _1));



    // Publishers
    pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pen/pose", qos_settings);
    acc_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/acc", qos_settings);
    vel_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/vel", qos_settings);
    thrust_SU_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/global_SU_Force_input", qos_settings);
    thrust_MJ_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/global_MJ_Force_input", qos_settings);
    thrust_raw_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/global_raw_Force_input", qos_settings);
    torque_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/body_torque_input", qos_settings);
    omega_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/omega", qos_settings);
    alpha_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/alpha", qos_settings);

    global_force_firmware_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/global_Force_input", qos_settings); // from firmware


    // Timer: 100Hz
    timer_ = this->create_wall_timer(
      control_loop_period, std::bind(&CfCommunicator::timer_callback, this));


    num_cal_timer_ = this->create_wall_timer(
      num_control_loop_period, std::bind(&CfCommunicator::num_cal_timer_callback, this));
  }

private:
  // Callback: Pose
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

  // Callback: Acceleration
  void cf_acc_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      acc_data_(i) = msg->values[i];  // float → double 변환 자동
  }

  // Callback: Velocity
  void cf_vel_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      vel_data_(i) = msg->values[i];
  }

  // Callback: Thrust
  void cf_thrust_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    thrust_data_[2] = msg->values[0];

    global_command_Force_raw = R_B * thrust_data_ / 100000.0;
    global_command_Force_SU = battery_scale_factor * global_command_Force_raw;

    global_command_Force_MJ = global_command_Force_raw / (-0.2714 * battery_voltage + 1.8286);

    }

  // Callback: Omega
  void cf_omega_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      omega_data_(i) = msg->values[i] * M_PI / 180;
  }

  void cf_battery_voltage_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    
    battery_voltage = msg->values[0];

    // 0819
    battery_scale_factor = 0.320569 * battery_voltage - 0.150439;
  }

  void cf_torque_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    torque_data_[0] = msg->values[0];
    torque_data_[1] = msg->values[1];
    torque_data_[2] = msg->values[2];
  }

  void cf_world_force_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    cf_world_force_[0] = msg->values[0];
    cf_world_force_[1] = msg->values[1];
    cf_world_force_[2] = msg->values[2];

      RCLCPP_INFO(
        this->get_logger(),
        "from Firmware = [%.3f, %.3f, %.3f]",
        cf_world_force_[0], cf_world_force_[1], cf_world_force_[2]
      );    
      RCLCPP_INFO(
        this->get_logger(),
        "from PID = [%.3f, %.3f, %.3f]",
        global_command_Force_raw[0], global_command_Force_raw[1], global_command_Force_raw[2]
      );    

  }  

  // Timer: Publishing
  void timer_callback()
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



    std_msgs::msg::Float64MultiArray acc_msg;
    acc_msg.data.push_back(acc_data_(0));
    acc_msg.data.push_back(acc_data_(1));
    acc_msg.data.push_back(acc_data_(2));
    acc_pub_->publish(acc_msg);

    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data.push_back(vel_data_(0));
    vel_msg.data.push_back(vel_data_(1));
    vel_msg.data.push_back(vel_data_(2));
    vel_pub_->publish(vel_msg);





    std_msgs::msg::Float64MultiArray global_SU_command_force_msg;
    global_SU_command_force_msg.data.push_back(global_command_Force_SU(0));
    global_SU_command_force_msg.data.push_back(global_command_Force_SU(1));
    global_SU_command_force_msg.data.push_back(global_command_Force_SU(2));
    thrust_SU_pub_->publish(global_SU_command_force_msg);

    std_msgs::msg::Float64MultiArray global_MJ_command_force_msg;
    global_MJ_command_force_msg.data.push_back(global_command_Force_MJ(0));
    global_MJ_command_force_msg.data.push_back(global_command_Force_MJ(1));
    global_MJ_command_force_msg.data.push_back(global_command_Force_MJ(2));
    thrust_MJ_pub_->publish(global_MJ_command_force_msg);

    std_msgs::msg::Float64MultiArray global_command_force_raw_msg;
    global_command_force_raw_msg.data.push_back(global_command_Force_raw(0));
    global_command_force_raw_msg.data.push_back(global_command_Force_raw(1));
    global_command_force_raw_msg.data.push_back(global_command_Force_raw(2));
    thrust_raw_pub_->publish(global_command_force_raw_msg);



    std_msgs::msg::Float64MultiArray omega_msg;
    omega_msg.data.push_back(omega_data_(0));
    omega_msg.data.push_back(omega_data_(1));
    omega_msg.data.push_back(omega_data_(2));
    omega_pub_->publish(omega_msg);

    std_msgs::msg::Float64MultiArray alpha_msg;
    alpha_msg.data.push_back(alpha_data_(0));
    alpha_msg.data.push_back(alpha_data_(1));
    alpha_msg.data.push_back(alpha_data_(2));
    alpha_pub_->publish(alpha_msg);

    std_msgs::msg::Float64MultiArray global_command_torque_msg;
    global_command_torque_msg.data.push_back(torque_data_(0));
    global_command_torque_msg.data.push_back(torque_data_(1));
    global_command_torque_msg.data.push_back(torque_data_(2));
    torque_pub_->publish(global_command_torque_msg);

    std_msgs::msg::Float64MultiArray global_force_firmware_msg;
    global_force_firmware_msg.data.push_back(cf_world_force_(0));
    global_force_firmware_msg.data.push_back(cf_world_force_(1));
    global_force_firmware_msg.data.push_back(cf_world_force_(2));
    global_force_firmware_pub_->publish(global_force_firmware_msg);

  }



  void num_cal_timer_callback()
  {
    // TODO: omega_data_ 를 수치미분하여 alpha_data_를 만들기.
    // Note: 수치미분

  }





  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_acc_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_thrust_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_torque_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_omega_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_battery_voltage_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_world_force_subscriber_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_SU_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_MJ_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omega_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr alpha_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_force_firmware_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_, num_cal_timer_;

  // Internal Data (Eigen)
  Eigen::VectorXd pose_data_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd global_xyz_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd global_quat_meas = Eigen::VectorXd::Zero(4);


  Eigen::Vector3d acc_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d alpha_data_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_B;
  Eigen::Vector3d global_command_Force_SU{Eigen::Vector3d::Zero()};
  Eigen::Vector3d global_command_Force_MJ{Eigen::Vector3d::Zero()};
  Eigen::Vector3d global_command_Force_raw{Eigen::Vector3d::Zero()};
  Eigen::Vector3d torque_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cf_world_force_{Eigen::Vector3d::Zero()};



  Eigen::Vector3d body_rpy_meas;


  double prev_yaw, yaw_offset, yaw_continuous, battery_voltage;
  double control_loop_hz, control_loop_period, num_control_loop_hz, num_control_loop_period;
  double battery_scale_factor;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CfCommunicator>());
  rclcpp::shutdown();
  return 0;
}