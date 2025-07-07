#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;


class su_rviz : public rclcpp::Node {
public:
    su_rviz() : Node("px4_rviz"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
    // rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
    // .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    // .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    auto qos_settings = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 6),
    rmw_qos_profile_sensor_data);


    //SUBSCRIBER GROUP
    px4_data_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/data_logging_msg", qos_settings,  // Topic name and QoS depth
      std::bind(&su_rviz::px4_data_subscriber_callback, this, std::placeholders::_1));



      Linear_velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Linear_velocity_arrow", 10);

      Desired_Linear_velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Desired_Linear_velocity_arrow", 10);


      Acceleration_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Acceleration_arrow", 10);

      Desired_Acceleration_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Desired_Acceleration_arrow", 10);


      Angular_Velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/Angular_Velocity_arrow", 10);

      Desired_Angular_Velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/Desired_Angular_Velocity_arrow", 10);


      Desired_Torque_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/Desired_Torque_arrow", 10);

      COM_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/com_update_marker", 10);

      COM_default_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/com_default_update_marker", 10);

        timer_ = this->create_wall_timer(
          10ms, std::bind(&su_rviz::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {

        broadcast_ned_to_enu_tf();

        Position_tf_publisher();
        Desired_Position_tf_publisher();

        COM_update_marker_publisher();
        COM_default_update_marker_publisher();
        // Linear_velocity_arrow_publisher();
        // Desired_Linear_velocity_arrow_publisher();

        // accel_arrow_publsiher();
        // Desired_accel_arrow_publsiher();

        // angular_velocity_arrow_publisher();
        // Desired_angular_velocity_arrow_publisher();

        // Desired_torque_arrow_publisher();
      }



      void COM_update_marker_publisher()
      {
          auto marker = visualization_msgs::msg::Marker();
          marker.header.frame_id = "palletrone"; // body 기준
          marker.header.stamp = this->get_clock()->now();
          marker.ns = "com_update";
          marker.id = 0;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.action = visualization_msgs::msg::Marker::ADD;

          
          marker.pose.position.x = com_update[0] * 10 + gravity_body[0];
          marker.pose.position.y = com_update[1] * 10 + gravity_body[1];
          marker.pose.position.z = com_update[2] * 10 + gravity_body[2];

          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.05;  // 지름
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;

          marker.color.a = 1.0;   // 불투명도
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;

          COM_marker_publisher_->publish(marker);
      }


      void COM_default_update_marker_publisher()
      {
          auto marker = visualization_msgs::msg::Marker();
          marker.header.frame_id = "palletrone"; // body 기준
          marker.header.stamp = this->get_clock()->now();
          marker.ns = "com_default_update";
          marker.id = 0;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.action = visualization_msgs::msg::Marker::ADD;

          marker.pose.position.x =  gravity_body[0];
          marker.pose.position.y =  gravity_body[1];
          marker.pose.position.z =  gravity_body[2];

          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.05;  // 지름
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;

          marker.color.a = 1.0;   // 불투명도
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;

          COM_default_marker_publisher_->publish(marker);
      }      


      void broadcast_ned_to_enu_tf()
      {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->get_clock()->now();
        static_tf.header.frame_id = "ned_world";
        static_tf.child_frame_id = "world";

        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, M_PI, M_PI);  // 90도 Z축 회전
        static_tf.transform.rotation.x = quat.x();
        static_tf.transform.rotation.y = quat.y();
        static_tf.transform.rotation.z = quat.z();
        static_tf.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(static_tf);
      }

      void Position_tf_publisher()
      {
          geometry_msgs::msg::TransformStamped transform_msg;
          transform_msg.header.stamp = this->get_clock()->now();
          transform_msg.header.frame_id = "world";  // 부모 프레임
          transform_msg.child_frame_id = "palletrone"; // 자식 프레임 (EE)

          transform_msg.transform.translation.x = position[0];
          transform_msg.transform.translation.y = position[1];
          transform_msg.transform.translation.z = position[2];

          tf2::Quaternion quat;
          quat.setRPY(attitude[0], attitude[1], attitude[2]);

          transform_msg.transform.rotation.x = quat.x();
          transform_msg.transform.rotation.y = quat.y();
          transform_msg.transform.rotation.z = quat.z();
          transform_msg.transform.rotation.w = quat.w();

          // TF Broadcast
          tf_broadcaster_->sendTransform(transform_msg);
      }

      void Desired_Position_tf_publisher()
      {
          geometry_msgs::msg::TransformStamped transform_msg;
          transform_msg.header.stamp = this->get_clock()->now();
          transform_msg.header.frame_id = "world";  // 부모 프레임
          transform_msg.child_frame_id = "desired_position"; // 자식 프레임 (EE)

          transform_msg.transform.translation.x = desired_position[0];
          transform_msg.transform.translation.y = desired_position[1];
          transform_msg.transform.translation.z = desired_position[2];

          tf2::Quaternion quat;
          quat.setRPY(desired_attitude[0], desired_attitude[1], desired_attitude[2]);

          transform_msg.transform.rotation.x = quat.x();
          transform_msg.transform.rotation.y = quat.y();
          transform_msg.transform.rotation.z = quat.z();
          transform_msg.transform.rotation.w = quat.w();

          // TF Broadcast
          tf_broadcaster_->sendTransform(transform_msg);
      }


      void Linear_velocity_arrow_publisher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Linear_velocity";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + linear_velocity[0];
        end_point.y = start_point.y + linear_velocity[1];
        end_point.z = start_point.z + linear_velocity[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.01;   // shaft 두께 (0.1 m)
        marker.scale.y = 0.02;   // head 직경
        marker.scale.z = 0.02;   // head 길이

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Linear_velocity_arrow_publisher_->publish(marker);
      }


      void Desired_Linear_velocity_arrow_publisher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Desired_Linear_velocity";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + desired_linear_velocity[0];
        end_point.y = start_point.y + desired_linear_velocity[1];
        end_point.z = start_point.z + desired_linear_velocity[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.01;   // shaft 두께 (0.1 m)
        marker.scale.y = 0.02;   // head 직경
        marker.scale.z = 0.02;   // head 길이

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        Desired_Linear_velocity_arrow_publisher_->publish(marker);
      }


      void accel_arrow_publsiher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "position";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "acceleration";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = 0;
        start_point.y = 0;
        start_point.z = 0;
        // RE_acceleration_filtered = alpha * RE_acceleration + (1.0 - alpha) * RE_acceleration_filtered;

        end_point.x = start_point.x + acceleration[0];
        end_point.y = start_point.y + acceleration[1];
        end_point.z = start_point.z + acceleration[2] + 9.81;

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.01;   // shaft 두께 (0.1 m)
        marker.scale.y = 0.02;   // head 직경
        marker.scale.z = 0.02;   // head 길이

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Acceleration_arrow_publisher_->publish(marker);
      }


      void Desired_accel_arrow_publsiher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "position";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Desired_acceleration";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = 0;//position[0];
        start_point.y = 0;//position[1];
        start_point.z = 0;//position[2];
        // RE_acceleration_desired_filtered = alpha * RE_desired_acceleration + (1.0 - alpha) * RE_acceleration_desired_filtered;

        end_point.x = start_point.x + desired_acceleration[0];
        end_point.y = start_point.y + desired_acceleration[1];
        end_point.z = start_point.z + desired_acceleration[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.01;   // shaft 두께 (0.1 m)
        marker.scale.y = 0.02;   // head 직경
        marker.scale.z = 0.02;   // head 길이

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        Desired_Acceleration_arrow_publisher_->publish(marker);
    }

    void angular_velocity_arrow_publisher()
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "Angular_velocity";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = position[0];
      start_point.y = position[1];
      start_point.z = position[2];

      end_point.x = start_point.x + RE_angular_velocity[0];
      end_point.y = start_point.y + RE_angular_velocity[1];
      end_point.z = start_point.z + RE_angular_velocity[2];

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      marker.scale.x = 0.01;  // shaft
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      Angular_Velocity_arrow_publisher_->publish(marker);
    }

    void Desired_angular_velocity_arrow_publisher()
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "Desired_Angular_velocity";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;



      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = position[0];
      start_point.y = position[1];
      start_point.z = position[2];

      end_point.x = start_point.x + RE_desired_angular_velocity[0] * 50;
      end_point.y = start_point.y + RE_desired_angular_velocity[1] * 50;
      end_point.z = start_point.z + RE_desired_angular_velocity[2] * 50;

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      marker.scale.x = 0.01;  // shaft
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;

      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      Desired_Angular_Velocity_arrow_publisher_->publish(marker);
    }


    void Desired_torque_arrow_publisher()
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "Desired_Torque";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = position[0];
      start_point.y = position[1];
      start_point.z = position[2];

      end_point.x = start_point.x + RE_desired_torque[0];
      end_point.y = start_point.y + RE_desired_torque[1];
      end_point.z = start_point.z + RE_desired_torque[2];

      marker.points.push_back(start_point);
      marker.points.push_back(end_point);

      marker.scale.x = 0.01;  // shaft
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;

      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      Desired_Torque_arrow_publisher_->publish(marker);
    }




    void px4_data_subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
    //////////////////////////
    // Data Analysis SCHEME //
    //////////////////////////
    // 시각화 시 모든 데이터의 header frame은 world frame으로 정렬할 것임
    // 따라서, body frame 기준으로 정의된 토픽들은 R_B를 곱한 뒤에 시각화 하였을 때 정상적으로 rviz에 보일 것임.
    // ex) desired_angular_velocity = R_B * desired_angular_velocity; => RVIZ에서 profit 함 => desired_angular_velocity는 body frame 기준으로 잘 정의 됨.

      //////////////////////////
      // Data Analysis SCHEME //
      //////////////////////////
      // 모든 데이터는 world frame 기준으로 시각화됨
      // 따라서 body frame 기준으로 정의된 데이터는 R_B를 곱해 변환

      // Position [World]
      position[0] = msg->data[0];
      position[1] = msg->data[1];
      position[2] = msg->data[2];

      desired_position[0] = msg->data[3];
      desired_position[1] = msg->data[4];
      desired_position[2] = msg->data[5];

      // Linear Velocity [World]
      linear_velocity[0] = msg->data[6];
      linear_velocity[1] = msg->data[7];
      linear_velocity[2] = msg->data[8];

      desired_linear_velocity[0] = msg->data[9];
      desired_linear_velocity[1] = msg->data[10];
      desired_linear_velocity[2] = msg->data[11];

      // Attitude (RPY) [Body]
      attitude[0] = msg->data[12];
      attitude[1] = msg->data[13];
      attitude[2] = msg->data[14];
      set_Rotation_matrix(); // updates R_B

      desired_attitude[0] = msg->data[15];
      desired_attitude[1] = msg->data[16];
      desired_attitude[2] = msg->data[17];

      // Angular Velocity [Body]
      angular_velocity[0] = msg->data[18];
      angular_velocity[1] = msg->data[19];
      angular_velocity[2] = msg->data[20];
      RE_angular_velocity = R_B * angular_velocity;

      desired_angular_velocity[0] = msg->data[21];
      desired_angular_velocity[1] = msg->data[22];
      desired_angular_velocity[2] = msg->data[23];
      RE_desired_angular_velocity = R_B * desired_angular_velocity;

      // Desired Force [Body]
      desired_force[0] = msg->data[24];
      desired_force[1] = msg->data[25];
      desired_force[2] = msg->data[26];
      RE_desired_force = R_B * desired_force;

      // Desired Torque [Body]
      desired_torque[0] = msg->data[27];
      desired_torque[1] = msg->data[28];
      desired_torque[2] = msg->data[29];
      RE_desired_torque = R_B * desired_torque;

      // Individual Motor Thrust [4 motors]
      individual_motor_thrust[0] = msg->data[30];
      individual_motor_thrust[1] = msg->data[31];
      individual_motor_thrust[2] = msg->data[32];
      individual_motor_thrust[3] = msg->data[33];

      // Servo Angle [4 DOF]
      servo_angle[0] = msg->data[34];
      servo_angle[1] = msg->data[35];
      servo_angle[2] = msg->data[36];
      servo_angle[3] = msg->data[37];

      // Desired Servo Angle [4 DOF]
      desired_servo_angle[0] = msg->data[38];
      desired_servo_angle[1] = msg->data[39];
      desired_servo_angle[2] = msg->data[40];
      desired_servo_angle[3] = msg->data[41];

      // Acceleration [Body]
      acceleration[0] = msg->data[42];
      acceleration[1] = msg->data[43];
      acceleration[2] = msg->data[44];
      RE_acceleration = R_B * acceleration;

      // Desired Acceleration [Body]
      desired_acceleration[0] = msg->data[45];
      desired_acceleration[1] = msg->data[46];
      desired_acceleration[2] = msg->data[47];
      RE_desired_acceleration = desired_acceleration;
      RE_desired_acceleration[2] += 80; // offset for visualization or compensator

      // CoM Estimation
      present_com[0] = msg->data[48];
      present_com[1] = msg->data[49];
      present_com[2] = msg->data[50];

      past_com[0] = msg->data[51];
      past_com[1] = msg->data[52];
      past_com[2] = msg->data[53];

      com_tilde[0] = msg->data[54];
      com_tilde[1] = msg->data[55];
      com_tilde[2] = msg->data[56];

      com_update[0] = msg->data[57];
      com_update[1] = msg->data[58];
      com_update[2] = msg->data[59];

      // Torque Disturbance Estimate [Body]
      torque_dhat[0] = msg->data[60];
      torque_dhat[1] = msg->data[61];
      torque_dhat[2] = msg->data[62];

      // PWM Command [8 channels]
      PWM_cmd[0] = msg->data[63];
      PWM_cmd[1] = msg->data[64];
      PWM_cmd[2] = msg->data[65];
      PWM_cmd[3] = msg->data[66];
      PWM_cmd[4] = msg->data[67];
      PWM_cmd[5] = msg->data[68];
      PWM_cmd[6] = msg->data[69];
      PWM_cmd[7] = msg->data[70];


    }

    void set_Rotation_matrix()
    {
      Rz << cos(attitude[2]), -sin(attitude[2]), 0,
            sin(attitude[2]),  cos(attitude[2]), 0,
                 0  ,       0  , 1;

      Ry << cos(attitude[1]), 0, sin(attitude[1]),
                   0 , 1,     0,
           -sin(attitude[1]), 0, cos(attitude[1]);

      Rx << 1,      0       ,       0,
            0, cos(attitude[0]), -sin(attitude[0]),
            0, sin(attitude[0]),  cos(attitude[0]);

      R_B = Rz * Ry * Rx;    /// Body 에서 Global로 변환해주는 Matrix이다.
                             /// Ex) [global value] = R_B * [body value]

      gravity << 0, 0, -0.1;
      gravity_body = R_B.transpose() * gravity;
    }


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Linear_velocity_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Linear_velocity_arrow_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Acceleration_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Acceleration_arrow_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Angular_Velocity_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Angular_Velocity_arrow_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Torque_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr COM_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr COM_default_marker_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr px4_data_subscriber;


    // Position
    Eigen::Vector3d position;                  // data[0] ~ data[2]
    Eigen::Vector3d desired_position;          // data[3] ~ data[5]

    // Linear Velocity
    Eigen::Vector3d linear_velocity;           // data[6] ~ data[8]
    Eigen::Vector3d desired_linear_velocity;   // data[9] ~ data[11]

    // Attitude (RPY)
    Eigen::Vector3d attitude;                  // data[12] ~ data[14]
    Eigen::Vector3d desired_attitude;          // data[15] ~ data[17]

    // Angular Velocity
    Eigen::Vector3d angular_velocity;          // data[18] ~ data[20]
    Eigen::Vector3d desired_angular_velocity;  // data[21] ~ data[23]

    // Desired Force & Torque
    Eigen::Vector3d desired_force;             // data[24] ~ data[26]
    Eigen::Vector3d desired_torque;            // data[27] ~ data[29]

    // Individual Motor Thrust (4 motors assumed)
    Eigen::VectorXd individual_motor_thrust = Eigen::VectorXd::Zero(4); // data[30] ~ data[33]

    // Servo Angles (4 DOF assumed)
    Eigen::VectorXd servo_angle         = Eigen::VectorXd::Zero(4);     // data[34] ~ data[37]
    Eigen::VectorXd desired_servo_angle = Eigen::VectorXd::Zero(4);     // data[38] ~ data[41]

    // Acceleration
    Eigen::Vector3d acceleration;             // data[42] ~ data[44]
    Eigen::Vector3d desired_acceleration;     // data[45] ~ data[47]

    // Center of Mass (CoM) related
    Eigen::Vector3d present_com;              // data[48] ~ data[50]
    Eigen::Vector3d past_com;                 // data[51] ~ data[53]
    Eigen::Vector3d com_tilde;                // data[54] ~ data[56]
    Eigen::Vector3d com_update;               // data[57] ~ data[59]

    // Torque disturbance observer estimate
    Eigen::Vector3d torque_dhat;              // data[60] ~ data[62]

    // PWM Command (e.g., 8 channels)
    Eigen::VectorXd PWM_cmd = Eigen::VectorXd::Zero(8);                 // data[63] ~ data[70]

    // Rotation matrices for attitude computation
    Eigen::Matrix3d Rz, Ry, Rx, R_B;

    // Optional filtered acceleration (if used in your controller)
    Eigen::Vector3d acceleration_filtered = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_acceleration_filtered = Eigen::Vector3d::Zero();


    // Angular Velocity [World frame for visualization]
    Eigen::Vector3d RE_angular_velocity;           // = R_B * angular_velocity
    Eigen::Vector3d RE_desired_angular_velocity;   // = R_B * desired_angular_velocity

    // Force & Torque [World frame for visualization]
    Eigen::Vector3d RE_desired_force;              // = R_B * desired_force
    Eigen::Vector3d RE_desired_torque;             // = R_B * desired_torque

    // Acceleration [World frame for visualization]
    Eigen::Vector3d RE_acceleration;               // = R_B * acceleration
    Eigen::Vector3d RE_desired_acceleration;       // = R_B * desired_acceleration

    Eigen::Vector3d gravity;               // data[57] ~ data[59]
    Eigen::Vector3d gravity_body;               // data[57] ~ data[59]


    const double cutoff_freq = 5.0;     // Hz
    const double dt = 0.01;             // 주기 (예: 100Hz 동작 시)
    const double tau = 1.0 / (2 * M_PI * cutoff_freq);
    const double alpha = dt / (tau + dt);

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_rviz>());
    rclcpp::shutdown();
    return 0;
}