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

        

        // Model Parameter /////////////////////////////////////////////////////

        std::vector<double> Momentum_of_inertia_bar_vec = this->declare_parameter<std::vector<double>>("Momentum_of_inertia_bar", {0.01, 0.01, 0.01}); // Bar의 질량중심에서의 Bar의 관성모멘트
        std::vector<double> Center_of_mass_bar_vec = this->declare_parameter<std::vector<double>>("Center_of_mass_bar", {0.01, 0.01, 0.01}); // 드론의 질량중심으로부터 bar의 질량중심까지의거리
        double mass_bar = this->declare_parameter<double>("mass_bar", 100.0);

        std::vector<double> Momentum_of_inertia_drone_vec = this->declare_parameter<std::vector<double>>("Momentum_of_inertia_drone", {0.01, 0.01, 0.01}); // Bar의 질량중심에서의 Bar의 관성모멘트
        double mass_drone = this->declare_parameter<double>("mass_drone", 100.0);


        // TODO 1.: Ligid Body Modeling. Bar와 Drone의 물성을 잘 조합해서 강제 모델링하기.

        total_mass = mass_bar + mass_drone;

        // 중심좌표계 기준 bar와 drone의 질량중심 위치 (벡터로 설정)
        Eigen::Vector3d com_drone = Eigen::Vector3d::Zero();  // 원점 기준
        Eigen::Vector3d com_bar(Center_of_mass_bar_vec[0], Center_of_mass_bar_vec[1], Center_of_mass_bar_vec[2]);

        // 총 CoM 위치
        Total_Center_of_Mass = (mass_bar * com_bar + mass_drone * com_drone) / total_mass;

        // 병진 운동 기준 관성: 병진의 병렬축 정리 (parallel axis theorem)
        Eigen::Vector3d I_bar_vec(Center_of_mass_bar_vec[0], Center_of_mass_bar_vec[1], Center_of_mass_bar_vec[2]);
        Eigen::Vector3d drone_I_vec(Momentum_of_inertia_drone_vec[0], Momentum_of_inertia_drone_vec[1], Momentum_of_inertia_drone_vec[2]);
        Eigen::Vector3d bar_I_vec(Momentum_of_inertia_bar_vec[0], Momentum_of_inertia_bar_vec[1], Momentum_of_inertia_bar_vec[2]);

        // 평행축 정리 적용 (I + m*d^2)
        Eigen::Vector3d bar_parallel_offset = com_bar - Total_Center_of_Mass;
        Eigen::Vector3d drone_parallel_offset = com_drone - Total_Center_of_Mass;

        Total_Momentum_of_inertia = 
            bar_I_vec + mass_bar * bar_parallel_offset.cwiseProduct(bar_parallel_offset) +
            drone_I_vec + mass_drone * drone_parallel_offset.cwiseProduct(drone_parallel_offset);




        
        force_dob_fc = this->declare_parameter<double>("force_dob_fc", 100.0);
        Tau_dob_fc = this->declare_parameter<double>("Tau_dob_fc", 100.0); 
        
        //////////////////////////////////////////////////////////////////
        // QoS Setting ////////////////////////////////////////////////////        
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                          .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);




        // Publisher Group ////////////////////////////////////////////////
        external_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", qos_settings);
        thrust_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/MJ_Force", qos_settings);   // 가속도곱하기 질량이랑 힘
        global_force_meas_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/dob_checker", qos_settings);


        //SUBSCRIBER GROUP
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pen/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&wrench_observer::cf_pose_subscriber, this, std::placeholders::_1));

        cf_omega_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/omega", qos_settings,
          std::bind(&wrench_observer::cf_meas_omega_callback, this, std::placeholders::_1));

        cf_acc_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/acc", qos_settings,
          std::bind(&wrench_observer::cf_meas_acc_callback, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/vel", qos_settings,
          std::bind(&wrench_observer::cf_meas_vel_callback, this, std::placeholders::_1));


        cf_thrust_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/thrust", qos_settings,
          std::bind(&wrench_observer::cf_thrust_callback, this, std::placeholders::_1));

        cf_torque_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/torque", qos_settings,
          std::bind(&wrench_observer::cf_torque_cmd_callback, this, std::placeholders::_1));
          


        init_dob_1st_order();
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

        Update_dob_1st_order();

        Calculate_MCG_Model_Dynamics();

        data_publish();
    }


    
      void Calculate_MCG_Model_Dynamics()
      {
        //TODO. 2: 적절히 값들을 활용하여, 논문 참고해서 MCG Dynamics 혹은 Newton Euler Dynamics 계산하기. 
        //Bar와 Drone의 물성을 잘 조합해서 계산하기.
        //////////////////////////////////////
        // NOTE: 아래 변수들을 참고하기. (필요에 따라 사용)
        // global_command_Force: global 기준 quadrotor가 만들어내는 force
        // torque_cmd: body 기준 quadrotor가 만들어내는 torque
        // R_B: Body rotational matrix
        // com_drone, com_bar, Total_Center_of_Mass, Total_Momentum_of_inertia, 등등.........

        // M_matrix = ;
        // C_matrix = ;
        // G_matrix = ;

      }

      void data_publish()
      {

        geometry_msgs::msg::Wrench external_wrench_msg;
        external_wrench_msg.force.x = global_force_hat[0];
        external_wrench_msg.force.y = global_force_hat[1];
        external_wrench_msg.force.z = global_force_hat[2];
        external_wrench_msg.torque.x = Nominal_Force[0] - Filtered_Force[0];
        external_wrench_msg.torque.y = Nominal_Force[1] - Filtered_Force[1];
        external_wrench_msg.torque.z = Nominal_Force[2] - Filtered_Force[2];

        external_wrench_publisher_->publish(external_wrench_msg);


        geometry_msgs::msg::Twist global_command_Force_msg;
        global_command_Force_msg.linear.x = global_command_Force[0];
        global_command_Force_msg.linear.y = global_command_Force[1];
        global_command_Force_msg.linear.z = global_command_Force[2];
        global_command_Force_msg.angular.x = global_force_meas[0];
        global_command_Force_msg.angular.y = global_force_meas[1];
        global_command_Force_msg.angular.z = global_force_meas[2];
        thrust_publisher_->publish(global_command_Force_msg);

        geometry_msgs::msg::Twist global_force_meas_msg;
        global_force_meas_msg.linear.x = Nominal_Force_1nd[0];
        global_force_meas_msg.linear.y = Nominal_Force_1nd[1];
        global_force_meas_msg.linear.z = Nominal_Force_1nd[2];
        global_force_meas_msg.angular.x = Filtered_Force_1nd[0];
        global_force_meas_msg.angular.y = Filtered_Force_1nd[1];
        global_force_meas_msg.angular.z = Filtered_Force_1nd[2];
        global_force_meas_publisher_->publish(global_force_meas_msg);



      }


    void cf_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Pose vector: [x, y, z]
        global_xyz_meas[0] = msg->pose.position.x;
        global_xyz_meas[1] = msg->pose.position.y;
        global_xyz_meas[2] = msg->pose.position.z;

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

    void cf_meas_omega_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      body_omega_meas[0] = msg->data[0];
      body_omega_meas[1] = msg->data[1];
      body_omega_meas[2] = msg->data[2];
    }

    void cf_thrust_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 1) {
        RCLCPP_WARN(this->get_logger(), "Thrust msg->values is empty");
        return;
      }
      Thrust[2] = msg->data[0];
      global_command_Force = R_B * Thrust / 100000.0;
    }

    
    void cf_torque_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Torque msg->values.size() < 3");
        return;
      }
      torque_cmd[0] = msg->data[0];
      torque_cmd[1] = msg->data[1];
      torque_cmd[2] = msg->data[2];
    }



    void cf_meas_acc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Acc msg->values.size() < 3");
        return;
      }
      body_acc_meas[0] = msg->data[0];
      body_acc_meas[1] = msg->data[1];
      body_acc_meas[2] = msg->data[2];
      global_acc_meas = R_B * body_acc_meas;
      // global_acc_meas[2] += 9.81;
      global_force_meas = total_mass * (global_acc_meas + Eigen::Vector3d(0, 0, 9.81));
    }
      

    void cf_meas_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Acc msg->values.size() < 3");
        return;
      }
      global_xyz_vel_meas[0] = msg->data[0];
      global_xyz_vel_meas[1] = msg->data[1];
      global_xyz_vel_meas[2] = msg->data[2];
    }


  void init_dob_2nd_order()
  {
    //TO GPT: Q filter를 butterworth 2차 필터로, input을 velocity로... 
    // Pn = 1 / ms
    //------------------Force DoB P_n inv Q----------------------

    MinvQ_A_F <<  -sqrt(2) * force_dob_fc, -pow(force_dob_fc,2),
                            1.0,		                0.0;

    MinvQ_B_F <<           1.0,                    0.0;

    MinvQ_C_F <<  Total_Center_of_Mass * pow(force_dob_fc,2), 			0.0;


    //------------------Force DoB Q filter----------------------
    Q_A_F <<   -sqrt(2) * force_dob_fc,  - pow(force_dob_fc,2),
                          1.0,		               0.0;

    Q_B_F <<             1.0,                   0.0;

    Q_C_F <<             0.0,          pow(force_dob_fc,2);




    //------------------Torque DoB P_n inv Q----------------------

    MinvQ_A_Tau <<  -sqrt(2) * Tau_dob_fc, - pow(Tau_dob_fc,2),
                            1.0,		                0.0;

    MinvQ_B_Tau <<         1.0,                    0.0;

    MinvQ_C_Taux <<  Total_Momentum_of_inertia[0] * pow(Tau_dob_fc,2), 			0.0;
    MinvQ_C_Tauy <<  Total_Momentum_of_inertia[1] * pow(Tau_dob_fc,2), 			0.0;
    MinvQ_C_Tauz <<  Total_Momentum_of_inertia[2] * pow(Tau_dob_fc,2), 			0.0;

    
    //------------------Torque DoB Q filter----------------------
    Q_A_Tau <<   -sqrt(2) * Tau_dob_fc,  - pow(Tau_dob_fc,2),
                          1.0,		               0.0;

    Q_B_Tau <<           1.0,                   0.0;

    Q_C_Tau <<           0.0,            pow(Tau_dob_fc,2);
  }


  void init_dob_1st_order(){
    // 1차 Butterworth 필터 기반 DoB
    // Pn = 1/m, Pn⁻¹Q = m * (wc / (s + wc))
    double wc = force_dob_fc;

    // Pn⁻¹ * Q 필터
    MinvQ_A_F_1nd << -wc;
    MinvQ_B_F_1nd << wc;
    MinvQ_C_F_1nd << total_mass;

    // Q 필터
    Q_A_F_1nd << -wc;
    Q_B_F_1nd << wc;
    Q_C_F_1nd << 1.0;

    // 결과를 Vector3 형태로 저장
    
  }


  void Update_dob_1st_order(){
    ///////////////////////////
    // -- Force DOB 1st-order -- //
    ///////////////////////////    

    // Nominal Side (input: measured acceleration)
    state_MinvQ_dot_Fx_1nd = MinvQ_A_F_1nd(0,0) * state_MinvQ_Fx_1nd + MinvQ_B_F_1nd(0,0) * global_acc_meas[0];
    state_MinvQ_Fx_1nd += state_MinvQ_dot_Fx_1nd / control_loop_hz;
    Nominal_Force_1nd[0] = MinvQ_C_F_1nd(0,0) * state_MinvQ_Fx_1nd;

    state_MinvQ_dot_Fy_1nd = MinvQ_A_F_1nd(0,0) * state_MinvQ_Fy_1nd + MinvQ_B_F_1nd(0,0) * global_acc_meas[1];
    state_MinvQ_Fy_1nd += state_MinvQ_dot_Fy_1nd / control_loop_hz;
    Nominal_Force_1nd[1] = MinvQ_C_F_1nd(0,0) * state_MinvQ_Fy_1nd;

    state_MinvQ_dot_Fz_1nd = MinvQ_A_F_1nd(0,0) * state_MinvQ_Fz_1nd + MinvQ_B_F_1nd(0,0) * global_acc_meas[2];
    state_MinvQ_Fz_1nd += state_MinvQ_dot_Fz_1nd / control_loop_hz;
    Nominal_Force_1nd[2] = MinvQ_C_F_1nd(0,0) * state_MinvQ_Fz_1nd;

    // Filtered Side (input: command force
    state_Q_dot_Fx_1nd = Q_A_F_1nd(0,0) * state_Q_Fx_1nd + Q_B_F_1nd(0,0) * global_command_Force[0];
    state_Q_Fx_1nd += state_Q_dot_Fx_1nd / control_loop_hz;
    Filtered_Force_1nd[0] = Q_C_F_1nd(0,0) * state_Q_Fx_1nd;

    state_Q_dot_Fy_1nd = Q_A_F_1nd(0,0) * state_Q_Fy_1nd + Q_B_F_1nd(0,0) * global_command_Force[1];
    state_Q_Fy_1nd += state_Q_dot_Fy_1nd / control_loop_hz;
    Filtered_Force_1nd[1] = Q_C_F_1nd(0,0) * state_Q_Fy_1nd;

    state_Q_dot_Fz_1nd = Q_A_F_1nd(0,0) * state_Q_Fz_1nd + Q_B_F_1nd(0,0) * global_command_Force[2];
    state_Q_Fz_1nd += state_Q_dot_Fz_1nd / control_loop_hz;
    Filtered_Force_1nd[2] = Q_C_F_1nd(0,0) * state_Q_Fz_1nd - total_mass * 9.81;

    // Final estimation
    global_force_hat = Nominal_Force_1nd - Filtered_Force_1nd;
  }





  void Update_dob_2nd_order(){
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
    state_Q_dot_Taux = Q_A_Tau * state_Q_Taux + Q_B_Tau * torque_cmd[0];
    state_Q_Taux += state_Q_dot_Taux / control_loop_hz;
    // y-direction
    state_Q_dot_Tauy = Q_A_Tau * state_Q_Tauy + Q_B_Tau * torque_cmd[1];
    state_Q_Tauy += state_Q_dot_Tauy / control_loop_hz;
    // y-direction
    state_Q_dot_Tauz = Q_A_Tau * state_Q_Tauz + Q_B_Tau * torque_cmd[2];
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr global_force_meas_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_omega_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_acc_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_thrust_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_torque_cmd_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



    Eigen::Vector3d global_xyz_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_xyz_vel_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d Thrust = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_command_Force = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_cmd = Eigen::Vector3d::Zero();
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
    double total_mass = 0;
    double control_loop_hz;
    Eigen::Vector3d com_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_vel_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_vel_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_force_meas = Eigen::Vector3d::Zero();


    Eigen::Vector3d Total_Momentum_of_inertia = Eigen::Vector3d::Zero();
    Eigen::Vector3d Total_Center_of_Mass = Eigen::Vector3d::Zero();



    // === 1st-order DoB 전용 변수 (_1nd suffix) ===
    Eigen::Matrix<double, 1, 1> Q_A_F_1nd, MinvQ_A_F_1nd;
    Eigen::Matrix<double, 1, 1> Q_B_F_1nd, MinvQ_B_F_1nd;
    Eigen::Matrix<double, 1, 1> Q_C_F_1nd, MinvQ_C_F_1nd;

    double state_Q_Fx_1nd = 0.0, state_Q_Fy_1nd = 0.0, state_Q_Fz_1nd = 0.0;
    double state_MinvQ_Fx_1nd = 0.0, state_MinvQ_Fy_1nd = 0.0, state_MinvQ_Fz_1nd = 0.0;
    double state_Q_dot_Fx_1nd = 0.0, state_Q_dot_Fy_1nd = 0.0, state_Q_dot_Fz_1nd = 0.0;
    double state_MinvQ_dot_Fx_1nd = 0.0, state_MinvQ_dot_Fy_1nd = 0.0, state_MinvQ_dot_Fz_1nd = 0.0;

    Eigen::Vector3d Nominal_Force_1nd = Eigen::Vector3d::Zero();
    Eigen::Vector3d Filtered_Force_1nd = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_force_hat_1nd = Eigen::Vector3d::Zero();


    Eigen::Matrix3d M_matrix;
    Eigen::Matrix3d C_matrix;
    Eigen::Matrix3d G_matrix;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wrench_observer>());
    rclcpp::shutdown();
    return 0;
}