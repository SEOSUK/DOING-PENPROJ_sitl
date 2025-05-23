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
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


class wrench_observer : public rclcpp::Node {
public:
    wrench_observer() : Node("wrench_observer"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {

        //////////////////////////////////////////////////////////////////
        // FROM YAML /////////////////////////////////////////////////////
        control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
        auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);

        

        // 관성 모멘트 구성 (대각행렬 가정)
        Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();        
        mass = this->declare_parameter<double>("mass", 1.0);
        std::vector<double> Momentum_of_inertia_com = this->declare_parameter<std::vector<double>>("Momentum_of_inertia_com", {0.01, 0.01, 0.01});
        std::vector<double> com_offset_vec = this->declare_parameter<std::vector<double>>("com_offset_vec", {0.01, 0.01, 0.01});

        Momentum_of_inertia_total = Eigen::Matrix3d::Zero();
        Momentum_of_inertia_total.diagonal() << Momentum_of_inertia_com[0], Momentum_of_inertia_com[1], Momentum_of_inertia_com[2];


        com_offset << com_offset_vec[0], com_offset_vec[1], com_offset_vec[2];
        // 평행축 정리 적용
        Eigen::Matrix3d J_O = Momentum_of_inertia_total
                            + mass * ((com_offset.transpose() * com_offset)(0) * identity
                            - com_offset * com_offset.transpose());

        // 결과를 Vector3 형태로 저장
        Momentum_of_inertia << J_O(0, 0), J_O(1, 1), J_O(2, 2);
        
        force_dob_fc = this->declare_parameter<double>("force_dob_fc", 100.0);
        Tau_dob_fc = this->declare_parameter<double>("Tau_dob_fc", 100.0); 


        //////////////////////////////////////////////////////////////////
        // QoS Setting ////////////////////////////////////////////////////        
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                          .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);




        // Publisher Group ////////////////////////////////////////////////
        external_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", qos_settings);
        thrust_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/MJ_Force", qos_settings);
        global_force_meas_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/MJ_Force_meas", qos_settings);


        //SUBSCRIBER GROUP
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf2/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&wrench_observer::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf/velocity", qos_settings,
          std::bind(&wrench_observer::cf_meas_velocity_callback, this, std::placeholders::_1));

        cf_acc_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf2/MJ_stateEstimate_acc", qos_settings,
          std::bind(&wrench_observer::cf_meas_acc_callback, this, std::placeholders::_1));

        cf_thrust_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf2/MJ_Command_thrust", qos_settings,
          std::bind(&wrench_observer::cf_thrust_callback, this, std::placeholders::_1));

        cf_desired_torque_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf/Desired_torque", qos_settings,
          std::bind(&wrench_observer::cf_desired_torque_callback, this, std::placeholders::_1));
          


        init_dob();
        start_time_ = this->now();

        timer_ = this->create_wall_timer(
          control_loop_period, std::bind(&wrench_observer::timer_callback, this));
        
}


    private:
    void timer_callback()
    {
        if ((this->now() - start_time_).seconds() < 5.0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for DoB stabilization...");
            return;
        }

        Update_dob();

        data_publish();
    }


    
      void data_publish()
      {

        geometry_msgs::msg::Wrench external_wrench_msg;
        external_wrench_msg.force.x = global_force_hat[0];
        external_wrench_msg.force.y = global_force_hat[1];
        external_wrench_msg.force.z = global_force_hat[2];
        external_wrench_msg.torque.x = body_tau_hat[0];
        external_wrench_msg.torque.y = body_tau_hat[1];
        external_wrench_msg.torque.z = body_tau_hat[2];

        external_wrench_publisher_->publish(external_wrench_msg);


        geometry_msgs::msg::Twist global_command_Force_msg;
        global_command_Force_msg.linear.x = global_command_Force[0];
        global_command_Force_msg.linear.y = global_command_Force[1];
        global_command_Force_msg.linear.z = global_command_Force[2];
        global_command_Force_msg.angular.x = global_force_meas[0];
        global_command_Force_msg.angular.y = global_force_meas[1];
        global_command_Force_msg.angular.z = global_force_meas[2];
        thrust_publisher_->publish(global_command_Force_msg);

        std_msgs::msg::Float64MultiArray global_force_meas_msg;
        global_force_meas_msg.data.push_back(global_force_meas[0]);
        global_force_meas_msg.data.push_back(global_force_meas[1]);
        global_force_meas_msg.data.push_back(global_force_meas[2]);
        global_force_meas_publisher_->publish(global_force_meas_msg);
      }



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



    void cf_thrust_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
    {
      Thrust[2] = msg->values[0];

      global_command_Force = R_B * Thrust / 100000;
    }

    void cf_desired_torque_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
    {
      desired_torque[0] = msg->values[0];
      desired_torque[1] = msg->values[1];
      desired_torque[2] = msg->values[2];
      
    }


    void cf_meas_velocity_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
    {
      global_xyz_vel_meas[0] = msg->values[0];
      global_xyz_vel_meas[1] = msg->values[1];
      global_xyz_vel_meas[2] = msg->values[2];
    }


    void cf_meas_acc_callback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
    {
      body_acc_meas[0] = msg->values[0];
      body_acc_meas[1] = msg->values[1];
      body_acc_meas[2] = msg->values[2];
      global_acc_meas = R_B * body_acc_meas;
      global_force_meas = mass * global_acc_meas;
    }    
      

  void init_dob()
  {
    //------------------Force DoB P_n inv Q----------------------

    MinvQ_A_F <<  -sqrt(2) * force_dob_fc, -pow(force_dob_fc,2),
                            1.0,		                0.0;

    MinvQ_B_F <<           1.0,                    0.0;

    MinvQ_C_F <<  mass * pow(force_dob_fc,2), 			0.0;


    //------------------Force DoB Q filter----------------------
    Q_A_F <<   -sqrt(2) * force_dob_fc,  - pow(force_dob_fc,2),
                          1.0,		               0.0;

    Q_B_F <<             1.0,                   0.0;

    Q_C_F <<             0.0,          pow(force_dob_fc,2);




    //------------------Torque DoB P_n inv Q----------------------

    MinvQ_A_Tau <<  -sqrt(2) * Tau_dob_fc, - pow(Tau_dob_fc,2),
                            1.0,		                0.0;

    MinvQ_B_Tau <<         1.0,                    0.0;

    MinvQ_C_Taux <<  Momentum_of_inertia[0] * pow(Tau_dob_fc,2), 			0.0;
    MinvQ_C_Tauy <<  Momentum_of_inertia[1] * pow(Tau_dob_fc,2), 			0.0;
    MinvQ_C_Tauz <<  Momentum_of_inertia[2] * pow(Tau_dob_fc,2), 			0.0;

    
    //------------------Torque DoB Q filter----------------------
    Q_A_Tau <<   -sqrt(2) * Tau_dob_fc,  - pow(Tau_dob_fc,2),
                          1.0,		               0.0;

    Q_B_Tau <<           1.0,                   0.0;

    Q_C_Tau <<           0.0,            pow(Tau_dob_fc,2);


  }


  void Update_dob(){
    ///////////////////////////
    // -- Force DOB -- //
    ///////////////////////////    
    // [Nominal Force]
    // X_direction
    state_MinvQ_dot_Fx = MinvQ_A_F * state_MinvQ_Fx + MinvQ_B_F * global_xyz_vel_meas[0];
    state_MinvQ_Fx += state_MinvQ_dot_Fx / control_loop_hz;
    // y_direction
    state_MinvQ_dot_Fy = MinvQ_A_F * state_MinvQ_Fy + MinvQ_B_F * global_xyz_vel_meas[1];
    state_MinvQ_Fy += state_MinvQ_dot_Fy / control_loop_hz;
    // z_direction
    state_MinvQ_dot_Fz = MinvQ_A_F * state_MinvQ_Fz + MinvQ_B_F * global_xyz_vel_meas[2];
    state_MinvQ_Fz += state_MinvQ_dot_Fz / control_loop_hz;
    // 최종정리
    Nominal_Force[0] = MinvQ_C_F * state_MinvQ_Fx;
    Nominal_Force[1] = MinvQ_C_F * state_MinvQ_Fy;
    Nominal_Force[2] = MinvQ_C_F * state_MinvQ_Fz;

    // [Filtered Force]
    // x-direction
    state_Q_dot_Fx = Q_A_F * state_Q_Fx + Q_B_F * global_command_Force[0];
    state_Q_Fx += state_Q_dot_Fx / control_loop_hz;
    // y-direction
    state_Q_dot_Fy = Q_A_F * state_Q_Fy + Q_B_F * global_command_Force[1];
    state_Q_Fy += state_Q_dot_Fy / control_loop_hz;
    // y-direction
    state_Q_dot_Fz = Q_A_F * state_Q_Fz + Q_B_F * global_command_Force[2];
    state_Q_Fz += state_Q_dot_Fz / control_loop_hz;
    // 최종정리
    Filtered_Force[0] = Q_C_F * state_Q_Fx;
    Filtered_Force[1] = Q_C_F * state_Q_Fy;
    Filtered_Force[2] = Q_C_F * state_Q_Fz;

    global_force_hat = Nominal_Force - Filtered_Force;


    ///////////////////////////
    // -- Torque DOB -- //
    ///////////////////////////    
    // [Nominal Torque]
    // X_direction
    state_MinvQ_dot_Taux = MinvQ_A_Tau * state_MinvQ_Taux + MinvQ_B_Tau * body_omega_meas[0];
    state_MinvQ_Taux += state_MinvQ_dot_Taux / control_loop_hz;
    // y_direction
    state_MinvQ_dot_Tauy = MinvQ_A_Tau * state_MinvQ_Tauy + MinvQ_B_Tau * body_omega_meas[1];
    state_MinvQ_Tauy += state_MinvQ_dot_Tauy / control_loop_hz;
    // z_direction
    state_MinvQ_dot_Tauz = MinvQ_A_Tau * state_MinvQ_Tauz + MinvQ_B_Tau * body_omega_meas[2];
    state_MinvQ_Tauz += state_MinvQ_dot_Tauz / control_loop_hz;
    // 최종정리
    Nominal_Tau[0] = MinvQ_C_Taux * state_MinvQ_Taux;
    Nominal_Tau[1] = MinvQ_C_Tauy * state_MinvQ_Tauy;
    Nominal_Tau[2] = MinvQ_C_Tauz * state_MinvQ_Tauz;

    // [Filtered Tau]
    // x-direction
    state_Q_dot_Taux = Q_A_Tau * state_Q_Taux + Q_B_Tau * desired_torque[0];
    state_Q_Taux += state_Q_dot_Taux / control_loop_hz;
    // y-direction
    state_Q_dot_Tauy = Q_A_Tau * state_Q_Tauy + Q_B_Tau * desired_torque[1];
    state_Q_Tauy += state_Q_dot_Tauy / control_loop_hz;
    // y-direction
    state_Q_dot_Tauz = Q_A_Tau * state_Q_Tauz + Q_B_Tau * desired_torque[2];
    state_Q_Tauz += state_Q_dot_Tauz / control_loop_hz;
    // 최종정리
    Filtered_Tau[0] = Q_C_Tau * state_Q_Taux;
    Filtered_Tau[1] = Q_C_Tau * state_Q_Tauy;
    Filtered_Tau[2] = Q_C_Tau * state_Q_Tauz;

    body_tau_hat = Nominal_Tau - Filtered_Tau;


  }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr external_wrench_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr thrust_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_force_meas_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_acc_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_thrust_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_desired_torque_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



    Eigen::Vector3d global_xyz_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_xyz_vel_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d Thrust = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_command_Force = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rpy_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_omega_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d Nominal_Force, Filtered_Force, global_force_hat;
    Eigen::Vector3d Nominal_Tau, Filtered_Tau, body_tau_hat;

    Eigen::Matrix3d R_B = Eigen::Matrix3d::Identity();

    // 상태공간 변수들 (Fx/Fy/Fz, Taux/Tauy/Tauz 별도)
    Eigen::Vector2d state_Q_Fx, state_Q_dot_Fx;
    Eigen::Vector2d state_Q_Fy, state_Q_dot_Fy;
    Eigen::Vector2d state_Q_Fz, state_Q_dot_Fz;
    Eigen::Vector2d state_MinvQ_Fx, state_MinvQ_dot_Fx;
    Eigen::Vector2d state_MinvQ_Fy, state_MinvQ_dot_Fy;
    Eigen::Vector2d state_MinvQ_Fz, state_MinvQ_dot_Fz;

    Eigen::Vector2d state_Q_Taux, state_Q_dot_Taux;
    Eigen::Vector2d state_Q_Tauy, state_Q_dot_Tauy;
    Eigen::Vector2d state_Q_Tauz, state_Q_dot_Tauz;
    Eigen::Vector2d state_MinvQ_Taux, state_MinvQ_dot_Taux;
    Eigen::Vector2d state_MinvQ_Tauy, state_MinvQ_dot_Tauy;
    Eigen::Vector2d state_MinvQ_Tauz, state_MinvQ_dot_Tauz;

    // 시스템 행렬 (공통 사용)
    Eigen::Matrix2d Q_A_F, MinvQ_A_F;
    Eigen::Vector2d Q_B_F, MinvQ_B_F;
    Eigen::RowVector2d Q_C_F, MinvQ_C_F;

    Eigen::Matrix2d Q_A_Tau, MinvQ_A_Tau;
    Eigen::Vector2d Q_B_Tau, MinvQ_B_Tau;
    Eigen::RowVector2d Q_C_Tau, MinvQ_C_Taux, MinvQ_C_Tauy, MinvQ_C_Tauz;

    double yaw_offset = 0.0, prev_yaw = 0.0, yaw_continuous = 0.0;
    double force_dob_fc = 0, Tau_dob_fc = 0;
    double mass = 0;
    double control_loop_hz;
    Eigen::Vector3d Momentum_of_inertia = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Momentum_of_inertia_total = Eigen::Matrix3d::Zero();
    Eigen::Vector3d com_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_force_meas = Eigen::Vector3d::Zero();



};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wrench_observer>());
    rclcpp::shutdown();
    return 0;
}
