#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include "test_pkg/su_rot.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>

using namespace std::chrono_literals;


class su_rviz : public rclcpp::Node {
public:
    su_rviz() : Node("su_rviz"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        //PUBLISHER GROUP
        cf_vel_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/cf_1/velocity", 10);
        cf_EE_vel_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/cf_1_EE/velocity", 10);

        cf_Force_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/cf_1_EE/force", 10);

        cf_box_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/box_marker", 10);

        normal_vector_hat_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/normal_vector_hat", 10);


        //SUBSCRIBER GROUP
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pen/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_rviz::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/vel", qos_settings,
          std::bind(&su_rviz::cf_velocity_subscriber, this, std::placeholders::_1));
          
        cf_omega_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/omega", qos_settings,
          std::bind(&su_rviz::cf_omega_subscriber, this, std::placeholders::_1));

        global_EE_xyz_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_xyzrpy", qos_settings,
          std::bind(&su_rviz::global_EE_xyz_callback, this, std::placeholders::_1));

        global_EE_xyz_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_xyzrpy_vel", qos_settings,
          std::bind(&su_rviz::global_EE_xyz_vel_callback, this, std::placeholders::_1));

        global_xyz_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pen/EE_des_xyzYaw", qos_settings,
        std::bind(&su_rviz::global_xyz_cmd_callback, this, std::placeholders::_1));

        Chat_rpy_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/Chat_rpy", qos_settings,
          std::bind(&su_rviz::Chat_rpy_callback, this, std::placeholders::_1));

        force_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
          "/ee/force_wrench", qos_settings,
          std::bind(&su_rviz::force_callback, this, std::placeholders::_1));


        global_EE_force_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/global_EE_force", qos_settings,
          std::bind(&su_rviz::global_EE_force_callback, this, std::placeholders::_1));

          global_normal_hat_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/normal_hat", qos_settings,
          std::bind(&su_rviz::global_normal_hat_callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
          10ms, std::bind(&su_rviz::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {
        cf_EE_pose_FK_tf_publisher();   // End Effector postion 시각화
        cf_EE_vel_FK_arrow_publisher();
        cf_box_marker_publisher();  // 직육면체 박스 마커 시각화



        cf_xyz_cmd_tf_publisher();   //Crazyflie cmd position 시각화

        // cf_pose_tf_publisher();   //Crazyflie position 시각화  이거 안쏴도 firmware에서 자동으로 쏴줌
        cf_vel_arrow_publisher();    // crazyflie velocity 시각화
        Normal_vector_estim_arrow_publisher();   //Normal Vector

        cf_Force_arrow_publisher();    // crazyflie force 시각화
        Chat_frame_tf_publisher();    // Chat frame 시각화
      }


      void cf_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
          // Pose vector: [x, y, z]
          global_xyz_meas[0] = msg->pose.position.x;
          global_xyz_meas[1] = msg->pose.position.y;
          global_xyz_meas[2] = msg->pose.position.z;

          global_xyz_quat[0] = msg->pose.orientation.x;
          global_xyz_quat[1] = msg->pose.orientation.y;
          global_xyz_quat[2] = msg->pose.orientation.z;
          global_xyz_quat[3] = msg->pose.orientation.w;



          tf2::Quaternion quat(
              msg->pose.orientation.x,
              msg->pose.orientation.y,
              msg->pose.orientation.z,
              msg->pose.orientation.w
          );

          tf2::Matrix3x3 mat(quat);
          double roll, pitch, yaw;
          mat.getRPY(roll, pitch, yaw);

          // Yaw 불연속 보정
          double delta_yaw = yaw - prev_yaw;
          if (delta_yaw > M_PI)
              yaw_offset -= 2.0 * M_PI;
          else if (delta_yaw < -M_PI)
              yaw_offset += 2.0 * M_PI;

          yaw_continuous = yaw + yaw_offset;
          prev_yaw = yaw;

          body_rpy_meas[0] = roll;
          body_rpy_meas[1] = pitch;
          body_rpy_meas[2] = yaw_continuous;

          // 회전행렬 업데이트
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              R_B(i, j) = mat[i][j];
      }

    void cf_velocity_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      global_xyz_vel_meas[0] = msg->data[0];
      global_xyz_vel_meas[1] = msg->data[1];
      global_xyz_vel_meas[2] = msg->data[2];
    }

    void cf_omega_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      body_omega_meas[0] = msg->data[0];
      body_omega_meas[1] = msg->data[1];
      body_omega_meas[2] = msg->data[2];
    }


    void global_EE_xyz_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_EE_xyz_meas[0] = msg->data[0];
      global_EE_xyz_meas[1] = msg->data[1];
      global_EE_xyz_meas[2] = msg->data[2];
      global_EE_rpy_meas[0] = msg->data[3];
      global_EE_rpy_meas[1] = msg->data[4];
      global_EE_rpy_meas[2] = msg->data[5];
    }

    void global_EE_xyz_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_EE_xyz_vel_meas[0] = msg->data[0];
      global_EE_xyz_vel_meas[1] = msg->data[1];
      global_EE_xyz_vel_meas[2] = msg->data[2];

      body_omega_meas[0] = msg->data[3];
      body_omega_meas[1] = msg->data[4];
      body_omega_meas[2] = msg->data[5];
    }

    void global_xyz_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_xyz_cmd[0] = msg->data[0];
      global_xyz_cmd[1] = msg->data[1];
      global_xyz_cmd[2] = msg->data[2];
      drone_yaw = msg->data[3];
    }

    void Chat_rpy_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      Chat_rpy[0] = msg->data[0];
      Chat_rpy[1] = msg->data[1];
      Chat_rpy[2] = msg->data[2];
      Chat_rpy[3] = msg->data[3];
      if (Chat_rpy[3] == 1) contact_flag = true;
      else if (Chat_rpy[3] == 0) contact_flag = false;

    }

    void force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg){
      global_force_meas[0] = msg->force.x;
      global_force_meas[1] = msg->force.y;
      global_force_meas[2] = msg->force.z;
    }

    void global_EE_force_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_force_des[0] = msg->data[0];
      global_force_des[1] = msg->data[1];
      global_force_des[2] = msg->data[2];
    }

    void global_normal_hat_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      normal_vector_hat[0] = msg->data[0];
      normal_vector_hat[1] = msg->data[1];
      normal_vector_hat[2] = msg->data[2];
      normal_vector_hat[3] = msg->data[3];
      if (normal_vector_hat[3] == 1) estimation_flag = true;
      else if (normal_vector_hat[3] == 0) estimation_flag = false;
    }


    void cf_box_marker_publisher() {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "box";
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // 중심 위치: 드론 기준 x축으로 1m 앞
      marker.pose.position.x = 1.2;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0.25;  // 높이 중심

      // 방향 없음 (회전 없음)
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // 직육면체 크기 설정 (x: 0.1, y: 2.0, z: 0.5)
      marker.scale.x = 0.1;
      marker.scale.y = 2.0;
      marker.scale.z = 0.5;

      // 밝은 나무색 (light brown: RGB 205, 133, 63) + 투명도
      marker.color.r = 205.0 / 255.0;
      marker.color.g = 133.0 / 255.0;
      marker.color.b = 63.0 / 255.0;
      marker.color.a = 0.6;


      cf_box_marker_publisher_->publish(marker);
    }


    void cf_xyz_cmd_tf_publisher(){
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "xyzYaw_cmd";

      transformStamped.transform.translation.x = global_xyz_cmd[0];
      transformStamped.transform.translation.y = global_xyz_cmd[1];
      transformStamped.transform.translation.z = global_xyz_cmd[2];

      tf2::Quaternion quat;
      quat.setRPY(0, 0, drone_yaw);
      transformStamped.transform.rotation.x = quat.x();
      transformStamped.transform.rotation.y = quat.y();
      transformStamped.transform.rotation.z = quat.z();
      transformStamped.transform.rotation.w = quat.w();

      tf_broadcaster_->sendTransform(transformStamped);
    }


    void cf_pose_tf_publisher() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "cf2";

        // 위치 설정
        transformStamped.transform.translation.x = global_xyz_meas[0];
        transformStamped.transform.translation.y = global_xyz_meas[1];
        transformStamped.transform.translation.z = global_xyz_meas[2];

        // 쿼터니언 기반 회전 설정
        tf2::Quaternion quat(
            global_xyz_quat[0],
            global_xyz_quat[1],
            global_xyz_quat[2],
            global_xyz_quat[3]
        );

        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }

    void cf_vel_arrow_publisher()
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "body_vel";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = global_xyz_meas[0];
      start_point.y = global_xyz_meas[1];
      start_point.z = global_xyz_meas[2];

      end_point.x = start_point.x + global_xyz_vel_meas[0];
      end_point.y = start_point.y + global_xyz_vel_meas[1];
      end_point.z = start_point.z + global_xyz_vel_meas[2];

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      if (global_xyz_vel_meas.norm() < 0.001)
      {
        marker.scale.x = 0.0;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
      }
      else
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      cf_vel_arrow_publisher_->publish(marker);
    }

    void Normal_vector_estim_arrow_publisher()
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "Normal_vector_hat";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = global_EE_xyz_meas[0];
      start_point.y = global_EE_xyz_meas[1];
      start_point.z = global_EE_xyz_meas[2];

      end_point.x = start_point.x + normal_vector_hat[0] / 2;
      end_point.y = start_point.y + normal_vector_hat[1] / 2;
      end_point.z = start_point.z + normal_vector_hat[2] / 2;

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      if (estimation_flag)
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }
      else
      {
        marker.scale.x = 0.0;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
      }

      marker.color.a = 1.0;
      marker.color.r = 1.0;  // 빨강
      marker.color.g = 0.5;  // 초록 (주황색)
      marker.color.b = 0.0;  // 파랑 없음

      if (estimation_flag) normal_vector_hat_publisher_->publish(marker);
    }


    void cf_EE_vel_FK_arrow_publisher(){

      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "EE_vel";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = global_EE_xyz_meas[0];
      start_point.y = global_EE_xyz_meas[1];
      start_point.z = global_EE_xyz_meas[2];

      end_point.x = start_point.x + global_EE_xyz_vel_meas[0];
      end_point.y = start_point.y + global_EE_xyz_vel_meas[1];
      end_point.z = start_point.z + global_EE_xyz_vel_meas[2];

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);


      if(global_EE_xyz_vel_meas.norm() < 0.001)
      {
        marker.scale.x = 0.0;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
      }
      else
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      cf_EE_vel_arrow_publisher_->publish(marker);
      }


    void cf_EE_pose_FK_tf_publisher()
    {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = this->get_clock()->now();
        transform_msg.header.frame_id = "world";  // 부모 프레임
        transform_msg.child_frame_id = "EE_frame"; // 자식 프레임 (EE)

        transform_msg.transform.translation.x = global_EE_xyz_meas[0];
        transform_msg.transform.translation.y = global_EE_xyz_meas[1];
        transform_msg.transform.translation.z = global_EE_xyz_meas[2];

        tf2::Quaternion quat;
        quat.setRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();

        // TF Broadcast
        tf_broadcaster_->sendTransform(transform_msg);
    }

    void Chat_frame_tf_publisher()
    {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = this->get_clock()->now();
      transform_msg.header.frame_id = "world";  // 부모 프레임
      transform_msg.child_frame_id = "Chat_frame"; //

      if(contact_flag)
      {
        transform_msg.transform.translation.x = global_EE_xyz_meas[0];
        transform_msg.transform.translation.y = global_EE_xyz_meas[1];
        transform_msg.transform.translation.z = global_EE_xyz_meas[2];

        tf2::Quaternion quat;
        quat.setRPY(Chat_rpy[0], Chat_rpy[1], Chat_rpy[2]);

        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();
      }
      else
      {
        transform_msg.transform.translation.x = 0;
        transform_msg.transform.translation.y = 0;
        transform_msg.transform.translation.z = 0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);

        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();
      }
      // TF Broadcast
      tf_broadcaster_->sendTransform(transform_msg);
    }

    void cf_Force_arrow_publisher(){

      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "Force_meas";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = global_EE_xyz_meas[0];
      start_point.y = global_EE_xyz_meas[1];
      start_point.z = global_EE_xyz_meas[2];

      end_point.x = start_point.x + global_force_meas[0] * 10;
      end_point.y = start_point.y + global_force_meas[1] * 10;
      end_point.z = start_point.z + global_force_meas[2] * 10;

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      if(contact_flag)
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }
      else
      {
        marker.scale.x = 0.00;
        marker.scale.y = 0.00;
        marker.scale.z = 0.00;
      }

      marker.color.a = 1.0;
      marker.color.r = 1.6;
      marker.color.g = 0.8;
      marker.color.b = 1.0;

      cf_Force_arrow_publisher_->publish(marker);
      }


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_vel_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_EE_vel_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_Force_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_box_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr normal_vector_hat_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_cmd_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Chat_rpy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_force_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_normal_hat_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_omega_subscriber_;



    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::VectorXd global_xyz_quat = Eigen::VectorXd::Zero(4);
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d global_rpy_meas;
    Eigen::Vector3d body_omega_meas;

    Eigen::Vector3d body_force_meas;
    Eigen::Vector3d global_force_meas;

    Eigen::Vector3d global_xyz_vel_meas;
    Eigen::Vector3d body_xyz_vel_meas;
    Eigen::Matrix3d R_B;

    Eigen::Vector3d global_EE_xyz_meas;
    Eigen::Vector3d global_EE_rpy_meas;
    Eigen::Vector3d global_EE_xyz_vel_meas;
    Eigen::Vector3d global_xyz_cmd;

    Eigen::Vector3d global_force_des;
    Eigen::VectorXd normal_vector_hat = Eigen::VectorXd::Zero(4);


    Eigen::VectorXd Chat_rpy = Eigen::VectorXd::Zero(4);

    bool contact_flag = false;
    bool estimation_flag = false;


    double drone_yaw, yaw_offset, yaw_continuous, prev_yaw;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_rviz>());
    rclcpp::shutdown();
    return 0;
}
