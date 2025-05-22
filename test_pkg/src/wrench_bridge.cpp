#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include "crazyflie_interfaces/msg/position.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

using std::placeholders::_1;

class GzToRosWrenchBridge : public rclcpp::Node
{
public:
  GzToRosWrenchBridge()
  : Node("gz_to_ros_wrench_bridge")
  {

    auto qos_settings = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 6),
    rmw_qos_profile_sensor_data);
    // Declare user parameters
    declare_parameter("control_loop_hz", 200.0);  // [s]
    declare_parameter("cutoff_freq", 5.0);        // [Hz]

    control_loop_hz_ = get_parameter("control_loop_hz").as_double();
    cutoff_freq_ = get_parameter("cutoff_freq").as_double();

    // Compute sampling period and alpha from control_loop_hz
    sampling_period_ = 1.0 / control_loop_hz_;
    double tau = 1.0 / (2.0 * M_PI * cutoff_freq_);
    alpha_ = sampling_period_ / (tau + sampling_period_);

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sampling_period_),
      std::bind(&GzToRosWrenchBridge::timerCallback, this));

    // Gazebo transport
    gz_node_.Subscribe("/gz/EE_forcetorque", &GzToRosWrenchBridge::gzCallback, this);


    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", 10);

    cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", qos_settings,  // Topic name and QoS depth
      std::bind(&GzToRosWrenchBridge::cf_pose_subscriber, this, std::placeholders::_1));



  }

private:

  void cf_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
      global_xyz_meas[0] = msg->pose.position.x;
      global_xyz_meas[1] = msg->pose.position.y;
      global_xyz_meas[2] = msg->pose.position.z;

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
  }

  void gzCallback(const gz::msgs::Wrench &msg)
  {
    latest_wrench_.force.x = msg.force().x();
    latest_wrench_.force.y = msg.force().y();
    latest_wrench_.force.z = msg.force().z();

    latest_wrench_.torque.x = msg.torque().x();
    latest_wrench_.torque.y = msg.torque().y();
    latest_wrench_.torque.z = msg.torque().z();
  }

  void timerCallback()
  {
    geometry_msgs::msg::Wrench filtered;

    filtered.force.x  = alpha_ * latest_wrench_.force.x  + (1 - alpha_) * prev_filtered_.force.x;
    filtered.force.y  = alpha_ * latest_wrench_.force.y  + (1 - alpha_) * prev_filtered_.force.y;
    filtered.force.z  = alpha_ * latest_wrench_.force.z  + (1 - alpha_) * prev_filtered_.force.z;

    filtered.torque.x = alpha_ * latest_wrench_.torque.x + (1 - alpha_) * prev_filtered_.torque.x;
    filtered.torque.y = alpha_ * latest_wrench_.torque.y + (1 - alpha_) * prev_filtered_.torque.y;
    filtered.torque.z = alpha_ * latest_wrench_.torque.z + (1 - alpha_) * prev_filtered_.torque.z;

    prev_filtered_ = filtered;


    // 변환: Body → Global
    Eigen::Vector3d f_body(filtered.force.x, filtered.force.y, filtered.force.z);
    Eigen::Vector3d t_body(filtered.torque.x, filtered.torque.y, filtered.torque.z);

    Eigen::Vector3d f_global = R_B * f_body;
    Eigen::Vector3d t_global = R_B * t_body;

    geometry_msgs::msg::Wrench wrench_global;
    wrench_global.force.x = f_global.x();
    wrench_global.force.y = f_global.y();
    wrench_global.force.z = f_global.z();
    wrench_global.torque.x = t_global.x();
    wrench_global.torque.y = t_global.y();
    wrench_global.torque.z = t_global.z();

    publisher_->publish(wrench_global);
  }


  double control_loop_hz_;  // ← 추가
  double sampling_period_;
  double cutoff_freq_;
  double alpha_;

  geometry_msgs::msg::Wrench latest_wrench_;
  geometry_msgs::msg::Wrench prev_filtered_;

  Eigen::Vector3d global_xyz_meas;
  Eigen::Vector3d body_rpy_meas;
  Eigen::Matrix3d R_B;
  double yaw_continuous, prev_yaw, yaw_offset;

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
  gz::transport::Node gz_node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzToRosWrenchBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
