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
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"

using namespace std::chrono_literals;


class wrench_observer : public rclcpp::Node
{
public:
    wrench_observer() : Node("wrench_observer"),
    general_vel_filter(6, 5, 1.0 / 200),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {

        //////////////////////////////////////////////////////////////////
        // FROM YAML /////////////////////////////////////////////////////
        control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
        auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);
        inverse_kinematics_Flag = this->declare_parameter<bool>("Inverse_Kinematics", true);
        K_0 = this->declare_parameter<double>("K_0", 1.0);


        // 관성 모멘트 구성 (대각행렬 가정)
        mass = this->declare_parameter<double>("mass", 1.0);

        std::vector<double> MoI_vec = this->declare_parameter<std::vector<double>>("Momentum_of_inertia", {0.01, 0.01, 0.01});
        Momentum_of_inertia << MoI_vec[0], MoI_vec[1], MoI_vec[2];
        MoI_Matrix.setZero();        
        MoI_Matrix.diagonal() = Momentum_of_inertia;  // = Vector3d
        


        std::vector<double> End_Effector_Offset_vec = this->declare_parameter<std::vector<double>>("End_Effector_Offset", {0.01, 0.01, 0.01});
        End_Effector_Offset << End_Effector_Offset_vec[0], End_Effector_Offset_vec[1], End_Effector_Offset_vec[2];

        force_dob_fc = this->declare_parameter<double>("force_dob_fc", 100.0);
        Tau_dob_fc = this->declare_parameter<double>("Tau_dob_fc", 100.0);


        //////////////////////////////////////////////////////////////////
        // QoS Setting ////////////////////////////////////////////////////
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                          .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);




        // Publisher Group ////////////////////////////////////////////////
        external_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/pen/wrench_estimation", qos_settings);
        test_publisher_1 = this->create_publisher<geometry_msgs::msg::Twist>("/test_1", qos_settings);
        test_publisher_2 = this->create_publisher<geometry_msgs::msg::Twist>("/test_2", qos_settings);

        //SUBSCRIBER GROUP
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pen/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&wrench_observer::cf_pose_subscriber, this, std::placeholders::_1));

        cf_acc_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/acc", qos_settings,
          std::bind(&wrench_observer::cf_meas_acc_callback, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/vel", qos_settings,
          std::bind(&wrench_observer::cf_meas_vel_callback, this, std::placeholders::_1));

        cf_omega_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/omega", qos_settings,
          std::bind(&wrench_observer::cf_meas_omega_callback, this, std::placeholders::_1));

        cf_force_input_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/global_SU_Force_input", qos_settings,
          std::bind(&wrench_observer::cf_force_input_callback, this, std::placeholders::_1));

        cf_body_torque_input_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/body_torque_input", qos_settings,
          std::bind(&wrench_observer::cf_torque_input_callback, this, std::placeholders::_1));


        init_dob_2nd_order();
        start_time_ = this->now();

        timer_ = this->create_wall_timer(
          control_loop_period, std::bind(&wrench_observer::timer_callback, this));

}

    private:
    void timer_callback()
    {
        if ((this->now() - start_time_).seconds() < 5.0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for MOB stabilization...");
        momentum_p_hat = momentum_p;
            return;
        }

        Update_mob();

        data_publish();
    }


    void data_publish()
      {

        // 이게 제어기에 들어가는 토픽이야!
        geometry_msgs::msg::Wrench external_wrench_msg;
        external_wrench_msg.force.x = - ext_wrench_hat[0];
        external_wrench_msg.force.y = - ext_wrench_hat[1];
        external_wrench_msg.force.z = - ext_wrench_hat[2];
        external_wrench_msg.torque.x = - ext_wrench_hat[3];
        external_wrench_msg.torque.y = - ext_wrench_hat[4];
        external_wrench_msg.torque.z = - ext_wrench_hat[5];
        external_wrench_publisher_->publish(external_wrench_msg);

        // 0819 ma
        geometry_msgs::msg::Twist test_1_msg;
        test_1_msg.linear.x = ext_wrench_hat_raw[0];
        test_1_msg.linear.y = ext_wrench_hat_raw[1];
        test_1_msg.linear.z = ext_wrench_hat_raw[2];
        test_1_msg.angular.x = ext_wrench_hat_raw[3];
        test_1_msg.angular.y = ext_wrench_hat_raw[4];
        test_1_msg.angular.z = ext_wrench_hat_raw[5];
        test_publisher_1->publish(test_1_msg);

        // 0819 F
        geometry_msgs::msg::Twist test_2_msg;
        test_2_msg.linear.x = ext_wrench_hat[0];
        test_2_msg.linear.y = ext_wrench_hat[1];
        test_2_msg.linear.z = ext_wrench_hat[2];
        test_2_msg.angular.x = ext_wrench_hat[3];
        test_2_msg.angular.y = ext_wrench_hat[4];
        test_2_msg.angular.z = ext_wrench_hat[5];
        test_publisher_2->publish(test_2_msg);


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
      if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Omega msg->values.size() < 3");
        return;
      }
      body_omega_meas[0] = msg->data[0];
      body_omega_meas[1] = msg->data[1];
      body_omega_meas[2] = msg->data[2];
    }


    void cf_force_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      global_Force_input[0] = msg->data[0];
      global_Force_input[1] = msg->data[1];
      global_Force_input[2] = msg->data[2];

    }


    void cf_torque_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Torque msg->values.size() < 3");
        return;
      }
      body_torque_input[0] = msg->data[0];
      body_torque_input[1] = msg->data[1];
      body_torque_input[2] = msg->data[2];
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
      global_force_meas = mass * global_acc_meas;
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


  void init_dob_1st_order()
  {
    // 1차 Butterworth 필터 기반 DoB
    // Pn = 1/m, Pn⁻¹Q = m * (wc / (s + wc))
    double wc = force_dob_fc;

    // Pn⁻¹ * Q 필터
    MinvQ_A_F_1nd << -wc;
    MinvQ_B_F_1nd << wc;
    MinvQ_C_F_1nd << mass;

    // Q 필터
    Q_A_F_1nd << -wc;
    Q_B_F_1nd << wc;
    Q_C_F_1nd << 1.0;
  }


  void Update_dob_1st_order()
  {
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
    state_Q_dot_Fx_1nd = Q_A_F_1nd(0,0) * state_Q_Fx_1nd + Q_B_F_1nd(0,0) * global_Force_input[0];
    state_Q_Fx_1nd += state_Q_dot_Fx_1nd / control_loop_hz;
    Filtered_Force_1nd[0] = Q_C_F_1nd(0,0) * state_Q_Fx_1nd;

    state_Q_dot_Fy_1nd = Q_A_F_1nd(0,0) * state_Q_Fy_1nd + Q_B_F_1nd(0,0) * global_Force_input[1];
    state_Q_Fy_1nd += state_Q_dot_Fy_1nd / control_loop_hz;
    Filtered_Force_1nd[1] = Q_C_F_1nd(0,0) * state_Q_Fy_1nd;

    state_Q_dot_Fz_1nd = Q_A_F_1nd(0,0) * state_Q_Fz_1nd + Q_B_F_1nd(0,0) * global_Force_input[2];
    state_Q_Fz_1nd += state_Q_dot_Fz_1nd / control_loop_hz;
    Filtered_Force_1nd[2] = Q_C_F_1nd(0,0) * state_Q_Fz_1nd - mass;

    // Final estimation
    global_force_hat = Nominal_Force_1nd - Filtered_Force_1nd;
  }


  void Update_dob_2nd_order()
  {
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
    state_Q_dot_Fx = Q_A_F * state_Q_Fx + Q_B_F * global_Force_input[0];
    state_Q_Fx += state_Q_dot_Fx / control_loop_hz;
    // y-direction
    state_Q_dot_Fy = Q_A_F * state_Q_Fy + Q_B_F * global_Force_input[1];
    state_Q_Fy += state_Q_dot_Fy / control_loop_hz;
    // y-direction
    state_Q_dot_Fz = Q_A_F * state_Q_Fz + Q_B_F * global_Force_input[2];
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
    state_Q_dot_Taux = Q_A_Tau * state_Q_Taux + Q_B_Tau * body_torque_input[0];
    state_Q_Taux += state_Q_dot_Taux / control_loop_hz;
    // y-direction
    state_Q_dot_Tauy = Q_A_Tau * state_Q_Tauy + Q_B_Tau * body_torque_input[1];
    state_Q_Tauy += state_Q_dot_Tauy / control_loop_hz;
    // y-direction
    state_Q_dot_Tauz = Q_A_Tau * state_Q_Tauz + Q_B_Tau * body_torque_input[2];
    state_Q_Tauz += state_Q_dot_Tauz / control_loop_hz;
    // 최종정리
    Filtered_Tau[0] = Q_C_Tau * state_Q_Taux;
    Filtered_Tau[1] = Q_C_Tau * state_Q_Tauy;
    Filtered_Tau[2] = Q_C_Tau * state_Q_Tauz;

    body_tau_hat = Nominal_Tau - Filtered_Tau;


  }


  void Update_mob()
  {
    //q, u, v 정의
    state_q <<       global_xyz_meas,
                      body_rpy_meas;


    wrench_u <<      global_Force_input,
                       body_torque_input;


    general_vel <<     global_xyz_vel_meas,
                       body_omega_meas;

    general_vel_dot_raw = (general_vel - general_vel_prev) * control_loop_hz;

    general_vel_dot = general_vel_filter.apply(general_vel_dot_raw);
    general_vel_prev = general_vel;


    // MCG 정의


    MCG_M <<    mass * I, mat_zero,
                 mat_zero,   MoI_Matrix;

    MCG_C <<      vec_zero,
                body_omega_meas.cross(MoI_Matrix * body_omega_meas);

    MCG_G <<       0,
                   0,
              mass * 9.81,
                vec_zero;

    inertia_acceleration = MCG_M * general_vel_dot; // 변수명 그지같이 지어서 미안하다.. F = ma 확인하려고 하는데 거기서 나오는 ma이다...
    //MOB 구현 0819

    double dt = 1.0 / control_loop_hz;
    /////////////////////////////////////////////////////
    // // BEFORE
    momentum_p = MCG_M * general_vel;
    ext_wrench_hat_raw = K_0 * (momentum_p - momentum_p_hat);


    ext_wrench_hat_raw = ext_wrench_hat_raw.unaryExpr([](double v) {
        if (std::abs(v) <= 0.004) {
            return 0.0;
        } else {
            return v - (0.004 * ((v > 0) ? 1.0 : -1.0));
        }
    });


    momentum_p_dot_hat = wrench_u - MCG_C - MCG_G + ext_wrench_hat_raw;    
    momentum_p_hat += momentum_p_dot_hat * dt;


    MCG_dyn = MCG_M * general_vel_dot + MCG_C + MCG_G;


//////////////////////////////Wrench Conversion
    Eigen::MatrixXd wrench_conversion_matrix(6,6);    
    wrench_conversion_matrix.setZero();    

    Eigen::Matrix3d S;
    S <<     0,      -End_Effector_Offset(2),  End_Effector_Offset(1),
          End_Effector_Offset(2),     0,     -End_Effector_Offset(0),
         -End_Effector_Offset(1),  End_Effector_Offset(0),     0;    

    // Top-left: Identity (Force unchanged)
    wrench_conversion_matrix.block<3,3>(0,0) = Eigen::Matrix3d::Identity();    
    // Top-right: -S(p_BE) * R_BW
    wrench_conversion_matrix.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    // Bottom-left: Zero
    wrench_conversion_matrix.block<3,3>(3,0) = -S * R_B.transpose();    
    // Bottom-right: Identity
    wrench_conversion_matrix.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

    ext_wrench_hat = wrench_conversion_matrix * ext_wrench_hat_raw;
//////////////////////////////FINISH



    // if(true) ext_wrench_hat = wrench_conversion(ext_wrench_hat_raw);
    // else ext_wrench_hat = ext_wrench_hat_raw;
  }

  // 0819
  Eigen::VectorXd wrench_conversion(Eigen::VectorXd ext_wrench_hat_)
  {
    Eigen::VectorXd EE_ext_wrench_hat_;
    Eigen::MatrixXd wrench_conversion_matrix(6,6);    
    wrench_conversion_matrix.setZero();    

    Eigen::Matrix3d S;
    S <<     0,      -End_Effector_Offset(2),  End_Effector_Offset(1),
          End_Effector_Offset(2),     0,     -End_Effector_Offset(0),
         -End_Effector_Offset(1),  End_Effector_Offset(0),     0;    

    // Top-left: Identity (Force unchanged)
    wrench_conversion_matrix.block<3,3>(0,0) = Eigen::Matrix3d::Identity();    
    // Top-right: Zero
    wrench_conversion_matrix.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    // Bottom-left: -S(p_BE) * R_BW
    wrench_conversion_matrix.block<3,3>(3,0) = -S * R_B.transpose();    
    // Bottom-right: Identity
    wrench_conversion_matrix.block<3,3>(3,3) = Eigen::Matrix3d::Identity();


    return wrench_conversion_matrix * ext_wrench_hat_;
  }



    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr external_wrench_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr test_publisher_1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr test_publisher_2;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_acc_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_force_input_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_body_torque_input_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_omega_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



    Eigen::Vector3d global_xyz_vel_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_Force_input = Eigen::Vector3d::Zero();

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
    Eigen::Matrix3d MoI_Matrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d com_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_vel_meas = Eigen::Vector3d::Zero();

    //MJ
    //MOB 변수들
    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d global_vel_meas;
    Eigen::Vector3d global_acc_meas;
    Eigen::Vector3d global_force_meas;
    Eigen::Vector3d body_torque_input;
    Eigen::Vector3d body_omega_meas;
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d vec_zero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d mat_zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 6, 1> state_q, wrench_u, general_vel, general_vel_dot, general_vel_prev, inertia_acceleration, general_vel_dot_raw;
    Eigen::Matrix<double, 6, 1> MCG_C, MCG_G;
    Eigen::Matrix<double, 6, 6> MCG_M;
    Eigen::Matrix<double, 6, 1> momentum_p;
    Eigen::Matrix<double, 6, 1> momentum_p_prev;
    Eigen::Matrix<double, 6, 1> momentum_p_dot;
    Eigen::Matrix<double, 6, 1> momentum_p_dot_hat;
    Eigen::Matrix<double, 6, 1> momentum_p_hat;
    Eigen::Matrix<double, 6, 1> ext_wrench_dot_hat;
    Eigen::Matrix<double, 6, 1> ext_wrench_hat_prev;
    Eigen::Matrix<double, 6, 1> ext_wrench_hat;
    Eigen::Matrix<double, 6, 1> ext_wrench_hat_raw;
    Eigen::Matrix<double, 6, 1> MCG_dyn;

    Eigen::Vector3d End_Effector_Offset = Eigen::Vector3d::Zero();



    // === 1st-order DoB 전용 변수 (_1nd suffix) ===
    Eigen::Matrix<double, 1, 1> Q_A_F_1nd, MinvQ_A_F_1nd;
    Eigen::Matrix<double, 1, 1> Q_B_F_1nd, MinvQ_B_F_1nd;
    Eigen::Matrix<double, 1, 1> Q_C_F_1nd, MinvQ_C_F_1nd;

    double state_Q_Fx_1nd = 0.0, state_Q_Fy_1nd = 0.0, state_Q_Fz_1nd = 0.0;
    double state_MinvQ_Fx_1nd = 0.0, state_MinvQ_Fy_1nd = 0.0, state_MinvQ_Fz_1nd = 0.0;
    double state_Q_dot_Fx_1nd = 0.0, state_Q_dot_Fy_1nd = 0.0, state_Q_dot_Fz_1nd = 0.0;
    double state_MinvQ_dot_Fx_1nd = 0.0, state_MinvQ_dot_Fy_1nd = 0.0, state_MinvQ_dot_Fz_1nd = 0.0;
    double K_0 = 0;

    Eigen::Vector3d Nominal_Force_1nd = Eigen::Vector3d::Zero();
    Eigen::Vector3d Filtered_Force_1nd = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_force_hat_1nd = Eigen::Vector3d::Zero();
    bool inverse_kinematics_Flag = false;
    FilteredVector general_vel_filter;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wrench_observer>());
    rclcpp::shutdown();
    return 0;
}
