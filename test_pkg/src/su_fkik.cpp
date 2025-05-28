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
#include <cmath>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "crazyflie_interfaces/msg/position.hpp"
#include <deque>

using namespace std::chrono_literals;


class su_fkik : public rclcpp::Node {
public:
su_fkik() : Node("su_fkik"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
    body_rpy_meas_dot_filter(3, 3.0, 1.0 / 50.0),
    body_omega_dot_filter(3, 3.0, 1.0 / 50.0),
    global_xyz_meas_dot_raw_filter(3, 3.0, 1.0 / 50.0)
   {

        // FROM YAML /////////////////////////////////////////////////////
        std::vector<double> offset_vec = this->declare_parameter<std::vector<double>>("End_Effector_Offset", {0.075, 0.0, 0.02});
        EE_offset_d << Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
        simulation_Flag = this->declare_parameter<bool>("Simulation", true);
        inverse_kinematics_Flag = this->declare_parameter<bool>("Inverse_Kinematics", true);

        control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
        auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);
        numerical_calc_loop_hz = this->declare_parameter<double>("numerical_calc_loop_hz", 100.0);
        auto numerical_calc_loop_period = std::chrono::duration<double>(1.0 / numerical_calc_loop_hz);


        // QoS Setting ////////////////////////////////////////////////////
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                          .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Publisher Group ////////////////////////////////////////////////
        global_xyz_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/xyzrpy", qos_settings);
        global_xyz_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/xyzrpy_vel", qos_settings);
        global_EE_xyz_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_xyzrpy", qos_settings);
        global_EE_xyz_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_xyzrpy_vel", qos_settings);
        global_xyz_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/cmd_xyzYaw", qos_settings);
        cf_global_xyzYaw_publisher_ = this->create_publisher<crazyflie_interfaces::msg::Position>("cf2/cmd_position", 10);

        test_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/test", qos_settings);


        // SUBSCIRIBER Group ////////////////////////////////////////////
        cf_pose_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_fkik::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/vel", qos_settings,
          std::bind(&su_fkik::cf_velocity_subscriber, this, std::placeholders::_1));

        cf_omega_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/omega", qos_settings,
          std::bind(&su_fkik::cf_omega_subscriber, this, std::placeholders::_1));

        global_EE_cmd_xyzYaw_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_des_xyzYaw", qos_settings,
          std::bind(&su_fkik::global_EE_cmd_xyzYaw_subscriber, this, std::placeholders::_1));


        // Low Pass Filter Init ////////////////////////////////////////////
          body_rpy_meas_dot_filter = FilteredVector(3, 3, 1 / numerical_calc_loop_hz);
          body_omega_dot_filter = FilteredVector(3, 3, 1 / numerical_calc_loop_hz);
          global_xyz_meas_dot_raw_filter = FilteredVector(3, 3, 1 / numerical_calc_loop_hz);


        // Timer ///////////////////////////////////////////////////////////
          control_loop_timer_ = this->create_wall_timer(
            control_loop_period, std::bind(&su_fkik::control_loop_callback, this));


      }


    private:

      void control_loop_callback()
      {
        cf_EE_vel_FK();
        cf_EE_position_FK();
        cf_EE_position_IK();
        data_publish();
      }



      void data_publish()
      {
        std_msgs::msg::Float64MultiArray global_xyz_msg;
        global_xyz_msg.data.push_back(global_xyz_meas[0]);
        global_xyz_msg.data.push_back(global_xyz_meas[1]);
        global_xyz_msg.data.push_back(global_xyz_meas[2]);
        global_xyz_msg.data.push_back(body_rpy_meas[0]);
        global_xyz_msg.data.push_back(body_rpy_meas[1]);
        global_xyz_msg.data.push_back(body_rpy_meas[2]);
        global_xyz_publisher_->publish(global_xyz_msg);

        std_msgs::msg::Float64MultiArray global_xyz_vel_msg;
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[0]);
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[1]);
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[2]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[0]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[1]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[2]);
        global_xyz_vel_publisher_->publish(global_xyz_vel_msg);

        std_msgs::msg::Float64MultiArray global_EE_xyz_msg;
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[0]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[1]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[2]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[0]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[1]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[2]);
        global_EE_xyz_publisher_->publish(global_EE_xyz_msg);

        crazyflie_interfaces::msg::Position global_xyzYaw_cmd_msg;
        global_xyzYaw_cmd_msg.x = global_xyz_cmd[0];
        global_xyzYaw_cmd_msg.y = global_xyz_cmd[1];
        global_xyzYaw_cmd_msg.z = global_xyz_cmd[2];
        global_xyzYaw_cmd_msg.yaw = global_EE_yaw_cmd * 180 / M_PI;
       cf_global_xyzYaw_publisher_->publish(global_xyzYaw_cmd_msg);


        std_msgs::msg::Float64MultiArray test_msg;
        test_msg.data.push_back(global_xyz_cmd[0]);
        test_msg.data.push_back(global_xyz_cmd[1]);
        test_msg.data.push_back(global_xyz_cmd[2]);
        test_msg.data.push_back(global_EE_yaw_cmd);
        test_msg.data.push_back(global_xyz_meas[0]);
        test_msg.data.push_back(global_xyz_meas[1]);
        test_msg.data.push_back(global_xyz_meas[2]);
        test_msg.data.push_back(body_rpy_meas[2]);
        test_publisher_->publish(test_msg);




        std_msgs::msg::Float64MultiArray global_EE_xyz_vel_msg;
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[0]);
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[1]);
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[2]);
        global_EE_xyz_vel_publisher_->publish(global_EE_xyz_vel_msg);


        std_msgs::msg::Float64MultiArray global_xyz_cmd_msg;
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[0]);
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[1]);
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[2]);
        global_xyz_cmd_msg.data.push_back(global_EE_yaw_cmd);
        global_xyz_cmd_publisher_->publish(global_xyz_cmd_msg);
      }


    void cf_pose_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Pose vector: [x, y, z, qx, qy, qz, qw]
        global_xyz_meas[0] = msg->data[0];
        global_xyz_meas[1] = msg->data[1];
        global_xyz_meas[2] = msg->data[2];

        tf2::Quaternion quat(
            msg->data[3],
            msg->data[4],
            msg->data[5],
            msg->data[6]
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

    void cf_velocity_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        global_xyz_vel_meas[0] = msg->data[0];
        global_xyz_vel_meas[1] = msg->data[1];
        global_xyz_vel_meas[2] = msg->data[2];

        // 필요시 global 변환도 여기에 포함 가능
        // global_xyz_vel_meas = R_B * body_xyz_vel_meas;
    }

    void cf_omega_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        body_omega_meas[0] = msg->data[0];
        body_omega_meas[1] = msg->data[1];
        body_omega_meas[2] = msg->data[2];

        // 필요시 global 변환도 여기에 포함 가능
        // global_xyz_vel_meas = R_B * body_xyz_vel_meas;
    }


    void global_EE_cmd_xyzYaw_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      global_EE_xyz_cmd[0] = msg->data[0];
      global_EE_xyz_cmd[1] = msg->data[1];
      global_EE_xyz_cmd[2] = msg->data[2];
      global_EE_yaw_cmd = msg->data[3];
    }

    void cf_EE_position_FK() {
      //TODO: global_xyz_meas, body_rpy_meas 를 조합해서 end effector position data를 만들기
      global_EE_xyz_meas = global_xyz_meas + R_B * EE_offset_d;


    }

    void cf_EE_vel_FK() {
      //TODO: global_xyz_vel_meas, body_rpy_meas, angular_velocity 를 조합해서 end effector velocity data를 만들기
      global_EE_xyz_vel_meas = global_xyz_vel_meas + body_omega_meas.cross(R_B * EE_offset_d);


    }

    void cf_EE_position_IK() {
      //TODO: End Effector poisition, End Effector yaw command를 받아서 drone position, drone yaw command로 변환하기.

      R_B_cmd << cos(body_rpy_meas[2]), -sin(body_rpy_meas[2]), 0,
                sin(body_rpy_meas[2]),   cos(body_rpy_meas[2]), 0,
                0,                       0,                       1;


      if (inverse_kinematics_Flag) global_xyz_cmd = global_EE_xyz_cmd - R_B_cmd * EE_offset_d;
      else global_xyz_cmd = global_EE_xyz_cmd;

      // RCLCPP_INFO(this->get_logger(), "global_xyz_cmd [%lf] [%lf] [%lf] [%lf]",
      // global_xyz_cmd[0], global_xyz_cmd[1], global_xyz_cmd[2], global_EE_yaw_cmd);
    }


    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr numerical_calc_timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_cmd_publisher_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_global_xyzYaw_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_cmd_xyzYaw_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_omega_subscriber_;

    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d global_xyz_meas_dot_raw;
    Eigen::Vector3d global_xyz_meas_prev;
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d body_rpy_meas_prev;
    Eigen::Vector3d body_rpy_meas_dot_raw;
    Eigen::Vector3d body_rpy_meas_dot;

    Eigen::Vector3d body_rpy_meas_dot_prev;
    Eigen::Vector3d body_rpy_meas_ddot_raw;
    Eigen::Vector3d body_rpy_meas_ddot;

    Eigen::Matrix3d omega_eulerRate_Mapping_matrix;
    Eigen::Vector3d body_omega_meas;
    Eigen::Vector3d body_omega_prev;
    Eigen::Vector3d body_omega_dot_raw;
    Eigen::Vector3d body_alpha_meas;

    Eigen::Vector3d global_rpy_meas;
    Eigen::Vector3d global_xyz_vel_meas;
    Eigen::Vector3d body_xyz_vel_meas;
    Eigen::Matrix3d R_B;
    Eigen::Matrix3d R_B_cmd;


    Eigen::Vector3d global_EE_xyz_meas;
    Eigen::Vector3d global_EE_xyz_vel_meas;
    Eigen::Vector3d EE_offset_d;

    Eigen::Vector3d global_EE_xyz_cmd;
    Eigen::Vector3d global_xyz_cmd;
    double global_EE_yaw_cmd;
    double global_yaw_cmd;


    FilteredVector body_rpy_meas_dot_filter;
    FilteredVector body_omega_dot_filter;
    FilteredVector global_xyz_meas_dot_raw_filter;


    double prev_yaw, yaw_offset, yaw_continuous;
    bool simulation_Flag;
    bool inverse_kinematics_Flag;
    double control_loop_hz = 0;
    double numerical_calc_loop_hz = 0;

    struct TimeStampedVector {
      rclcpp::Time stamp;
      Eigen::Vector3d data;
    };

    std::deque<TimeStampedVector> pose_buffer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_fkik>());
    rclcpp::shutdown();
    return 0;
}

