#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

using std::placeholders::_1;

class CfCommunicator : public rclcpp::Node
{
public:
  CfCommunicator()
  : Node("cf_communicator")
  {
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

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

    cf_omega_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/MJ_omega", qos_settings,
      std::bind(&CfCommunicator::cf_omega_callback, this, _1));


    // Publishers
    pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pen/pose", qos_settings);
    acc_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/acc", qos_settings);
    vel_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/vel", qos_settings);
    thrust_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/thrust", qos_settings);
    omega_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/omega", qos_settings);

    // Timer: 200Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&CfCommunicator::timer_callback, this));
  }

private:
  // Callback: Pose
  void cf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_data_[0] = msg->pose.position.x;
    pose_data_[1] = msg->pose.position.y;
    pose_data_[2] = msg->pose.position.z;
    pose_data_[3] = msg->pose.orientation.x;
    pose_data_[4] = msg->pose.orientation.y;
    pose_data_[5] = msg->pose.orientation.z;
    pose_data_[6] = msg->pose.orientation.w;
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
    for (size_t i = 0; i < 3; ++i)
      thrust_data_(i) = msg->values[i];
  }

  // Callback: Omega
  void cf_omega_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    for (size_t i = 0; i < 3; ++i)
      omega_data_(i) = msg->values[i] * M_PI / 180;
  }


  // Timer: Publishing
  void timer_callback()
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();  // 🟢 타임스탬프 명시
    pose_msg.header.frame_id = "cf2";  // 🟢 프레임 명시 (TF를 위한 필수값)
    pose_msg.pose.position.x = pose_data_[0];
    pose_msg.pose.position.y = pose_data_[1];
    pose_msg.pose.position.z = pose_data_[2];
    pose_msg.pose.orientation.x = pose_data_[3];
    pose_msg.pose.orientation.y = pose_data_[4];
    pose_msg.pose.orientation.z = pose_data_[5];
    pose_msg.pose.orientation.w = pose_data_[6];

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

    std_msgs::msg::Float64MultiArray thrust_msg;
    thrust_msg.data.push_back(thrust_data_(0));
    thrust_msg.data.push_back(thrust_data_(1));
    thrust_msg.data.push_back(thrust_data_(2));
    thrust_pub_->publish(thrust_msg);

    std_msgs::msg::Float64MultiArray omega_msg;
    omega_msg.data.push_back(omega_data_(0));
    omega_msg.data.push_back(omega_data_(1));
    omega_msg.data.push_back(omega_data_(2));
    omega_pub_->publish(omega_msg);    
  }

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_acc_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_thrust_subscriber_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_omega_subscriber_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omega_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal Data (Eigen)
  Eigen::VectorXd pose_data_ = Eigen::VectorXd::Zero(7);


  Eigen::Vector3d acc_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_data_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_data_{Eigen::Vector3d::Zero()};

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CfCommunicator>());
  rclcpp::shutdown();
  return 0;
}
