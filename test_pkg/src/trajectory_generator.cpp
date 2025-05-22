#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시
#include <std_msgs/msg/string.hpp>  // 다중 float64 배열 퍼블리시
#include "geometry_msgs/msg/twist.hpp"  // 다중 float64 배열 퍼블리시
#include <string> // std::string 헤더 추가
#include "std_msgs/msg/float64.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ncurses.h> // ncurses 헤더
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"

using namespace std::chrono_literals;

class trajectory_generator : public rclcpp::Node
{
public:
trajectory_generator()
  : Node("su_position_ctrler"),
  force_dot_filter(1, 3, 0.01), // LPF INIT
  normal_vector_filter(3, 3, 0.01), // LPF INIT
  count_(0)
  {

    // FROM YAML /////////////////////////////////////////////////////
    control_loop_hz = this->declare_parameter<double>("control_loop_hz", 100.0);
    auto control_loop_period = std::chrono::duration<double>(1.0 / control_loop_hz);
    numerical_calc_loop_hz = this->declare_parameter<double>("numerical_calc_loop_hz", 100.0);
    auto numerical_calc_loop_period = std::chrono::duration<double>(1.0 / numerical_calc_loop_hz);

    force_delta_cmd = this->declare_parameter<double>("force_delta_cmd", 0.003);
    std::vector<double> position_delta = this->declare_parameter<std::vector<double>>("position_delta_cmd", {0.1, 0.1, 0.1, 0.1});
    position_delta_cmd << Eigen::Vector4d(position_delta[0], position_delta[1], position_delta[2], position_delta[3]);

    force_lpf_cof = this->declare_parameter<double>("force_lpf_cof", 3);
    normal_vector_estimator_lpf_cof = this->declare_parameter<double>("normal_vector_estimator_lpf_cof", 3);


    // QOS /////////////////////////////////////////////////////
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


    // PUB /////////////////////////////////////////////////////
    global_EE_des_xyzYaw_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_des_xyzYaw", qos_settings);
    Chat_rpy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/Chat_rpy", qos_settings);
    global_EE_force_ctrl_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/global_EE_force", qos_settings);
    global_normal_hat_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/normal_hat", qos_settings);


    // SUB /////////////////////////////////////////////////////
    keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input", qos_settings,
      std::bind(&trajectory_generator::keyboard_subsciber_callback, this, std::placeholders::_1));

    cf_pose_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pen/xyzrpy", qos_settings,  // Topic name and QoS depth
      std::bind(&trajectory_generator::cf_pose_subscriber, this, std::placeholders::_1));

    force_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/ee/force_wrench", qos_settings,
      std::bind(&trajectory_generator::force_callback, this, std::placeholders::_1));

    global_EE_xyz_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pen/EE_xyzrpy_vel", qos_settings,
      std::bind(&trajectory_generator::global_EE_xyz_vel_callback, this, std::placeholders::_1));


    // Low Pass Filter Init ////////////////////////////////////////////
    force_dot_filter = FilteredVector(1, force_lpf_cof, 1 / control_loop_hz), // LPF INIT
    normal_vector_filter = FilteredVector(3, normal_vector_estimator_lpf_cof, 1 / control_loop_hz), // LPF INIT



    // Timer ///////////////////////////////////////////////////////////
    timer_ = this->create_wall_timer(
      control_loop_period, std::bind(&trajectory_generator::timer_callback, this));


  }

private:
  void timer_callback()
  {
    time_cnt ++;
    time_real = time_cnt / control_loop_hz;

    if (time_real < 5) init_hovering(); // 붕 뜨기
    else if (global_des_xyzYaw[2] < 0.05) global_des_xyzYaw[2] = -1; // sudo_land
    // external wrench observer를 여기에 하는게 나을지 fkik쪽에 넣는게 나을지 보류
    contact_check();
    normal_vector_estimation();
    Define_normal_frame();
    admittance_control();
    surface_trajectory_generation();

    data_publish();
  }

  void contact_check()
  {
    if (global_force_meas.norm() > 0.001)
    {
      contact_flag = true;
    }
    else
    {
      contact_flag = false;
    }

    if (global_force_meas.norm() > 0.001 &&
        global_EE_xyz_vel_meas.norm() > 0.06)
    {
      estimation_flag = true;
    }
    else
    {
      estimation_flag = false;
    }
  }

  void normal_vector_estimation()
  {

      if (estimation_flag)
      {
        double alpha = (global_force_meas.dot(global_EE_xyz_vel_meas)) / (global_EE_xyz_vel_meas.dot(global_EE_xyz_vel_meas));
        global_EE_force_normal_meas = normal_vector_filter.apply(global_force_meas - alpha * global_EE_xyz_vel_meas);
        // 여기까지가 Kin 방법 기반 추정기

        global_xi = global_EE_force_normal_meas - global_force_des;

        delta_normal_vector_hat = global_xi - (global_xi.dot(normal_vector_hat)) * normal_vector_hat;

        normal_vector_hat += rho * delta_normal_vector_hat / control_loop_hz;
        normal_vector_hat.normalize();

      }

  }


  void Define_normal_frame()
  {
      Eigen::Vector3d g(0, 0, -9.81); // 일반적인 중력 방향
      Chat_x = normal_vector_hat;
      Chat_y = Chat_x.cross(g);
      if (Chat_y.norm() < 1e-6) {
        // RCLCPP_WARN(this->get_logger(), "[Warning!!!] Gravity vector and normal vector are parallel!");
        return;
      }
    Chat_x.normalize();
    Chat_y.normalize();
    Chat_z = Chat_x.cross(Chat_y);
    Chat_z.normalize();

    R_Chat.col(0) = Chat_x;
    R_Chat.col(1) = Chat_y;
    R_Chat.col(2) = Chat_z;


    // RPY 각도 추출 (ZYX 순서: yaw → pitch → roll)
    Eigen::Vector3d eul_zyx = R_Chat.eulerAngles(2, 1, 0);  // yaw, pitch, roll

    Chat_rpy << eul_zyx[2], eul_zyx[1], eul_zyx[0];  // roll, pitch, yaw

  }


  void data_publish()
  {
    std_msgs::msg::Float64MultiArray global_xyz_EE_des_msg;
    global_xyz_EE_des_msg.data.push_back(global_des_xyzYaw[0]);
    global_xyz_EE_des_msg.data.push_back(global_des_xyzYaw[1]);
    global_xyz_EE_des_msg.data.push_back(global_des_xyzYaw[2]);
    global_xyz_EE_des_msg.data.push_back(global_des_xyzYaw[3]);
    global_EE_des_xyzYaw_publisher_->publish(global_xyz_EE_des_msg);

    std_msgs::msg::Float64MultiArray Chat_rpy_msg;
    Chat_rpy_msg.data.push_back(Chat_rpy[0]);
    Chat_rpy_msg.data.push_back(Chat_rpy[1]);
    Chat_rpy_msg.data.push_back(Chat_rpy[2]);
    if (contact_flag) Chat_rpy_msg.data.push_back(1);
    else Chat_rpy_msg.data.push_back(0);
    Chat_rpy_publisher_->publish(Chat_rpy_msg);

    std_msgs::msg::Float64MultiArray global_EE_force_ctrl_msg;
    global_EE_force_ctrl_msg.data.push_back(global_force_des[0]);
    global_EE_force_ctrl_msg.data.push_back(global_force_des[1]);
    global_EE_force_ctrl_msg.data.push_back(global_force_des[2]);
    global_EE_force_ctrl_publisher_->publish(global_EE_force_ctrl_msg);



    std_msgs::msg::Float64MultiArray normal_vector_hat_msg;
    normal_vector_hat_msg.data.push_back(normal_vector_hat[0]);
    normal_vector_hat_msg.data.push_back(normal_vector_hat[1]);
    normal_vector_hat_msg.data.push_back(normal_vector_hat[2]);
    if (estimation_flag) normal_vector_hat_msg.data.push_back(1);
    else normal_vector_hat_msg.data.push_back(0);
    global_normal_hat_publisher_->publish(normal_vector_hat_msg);

  }




  void init_hovering()
  {

    if (time_real < 2) global_des_xyzYaw[2] = -0.1;
    else if (time_real > 2 && time_real < 4) chat_des_vel_xyzYaw[2] = 0.3;
    else if (time_real > 4.5 && time_real < 5)
    {
      // RCLCPP_INFO(this->get_logger(), "hovering done. ready to move");
      chat_des_vel_xyzYaw[2] = 0.0;
    }
    // RCLCPP_INFO(this->get_logger(), "z position: %lf", chat_des_vel_xyzYaw[2]);
  }


  void admittance_control() // 이라 하고 force control 이라 읽는다.
  {
    chat_force_meas[0] = global_force_meas.dot(normal_vector_hat); //투영한 값.
    chat_force_error = chat_force_des - chat_force_meas;
    chat_force_error_dot_raw = (chat_force_error - chat_force_error_prev) * control_loop_hz;
    chat_force_error_dot = force_dot_filter.apply(chat_force_error_dot_raw);

    chat_des_vel_adm.head(3) = virtual_damper * chat_force_error + virtual_spring * chat_force_error_dot;
    /// 여기까지가 제어기 용이고, 아래에는 로깅을 위한 데이터 변환
    global_force_des = R_Chat * chat_force_des;

  }


  void surface_trajectory_generation()
  {
    if (contact_flag)
      {
        ////////////////////////
        // Position Generation//
        ////////////////////////
        global_des_vel_xyzYaw.head(3) = R_Chat *(chat_des_vel_xyzYaw.head(3) + chat_des_vel_adm.head(3));

        ////////////////////////
        //// Yaw Generation //// 우선은 PID 돌림.
        ////////////////////////
        global_yaw_cmd =  std::atan2(Chat_x[1], Chat_x[0]);
        double global_error_yaw = global_yaw_cmd - body_rpy_meas[2];
        double global_error_yaw_dot = (global_error_yaw - global_error_yaw_prev) * control_loop_hz;
        global_des_vel_xyzYaw[3] = 0.7 * global_error_yaw + 0.01 * global_error_yaw_dot;
      }
    else
      {
        global_des_vel_xyzYaw = chat_des_vel_xyzYaw;
      }

    global_des_xyzYaw += global_des_vel_xyzYaw / control_loop_hz;
    if (contact_flag) global_des_xyzYaw[3] = global_yaw_cmd;
  }

  void keyboard_subsciber_callback(const std_msgs::msg::String::SharedPtr msg)
  {
      // 입력된 키를 문자열로 가져옴
      std::string input = msg->data;

        if (!input.empty()) // 입력 값이 비어있지 않을 경우
        {
            char input_char = input[0]; // 문자열의 첫 번째 문자만 사용

            if (input_char == 'w') chat_des_vel_xyzYaw[0] += position_delta_cmd[0];
            else if (input_char == 's') chat_des_vel_xyzYaw[0] -= position_delta_cmd[0];
            else if (input_char == 'a') chat_des_vel_xyzYaw[1] += position_delta_cmd[1];
            else if (input_char == 'd') chat_des_vel_xyzYaw[1] -= position_delta_cmd[1];
            else if (input_char == 'e') chat_des_vel_xyzYaw[2] += position_delta_cmd[2];
            else if (input_char == 'q') chat_des_vel_xyzYaw[2] -= position_delta_cmd[2];
            else if (input_char == 'z') chat_des_vel_xyzYaw[3] += position_delta_cmd[3];
            else if (input_char == 'c') chat_des_vel_xyzYaw[3] -= position_delta_cmd[3];
            else if (input_char == 'x')
            {
              chat_des_vel_xyzYaw[0] = 0;
              chat_des_vel_xyzYaw[1] = 0;
              chat_des_vel_xyzYaw[2] = 0;
              chat_des_vel_xyzYaw[3] = 0;
            }
            else if (input_char == 'p')
            {
              chat_des_vel_xyzYaw[2] = -1;
            }
            else if (input_char == 'j') chat_force_des[0] += force_delta_cmd;
            else if (input_char == 'k') chat_force_des[0] -= force_delta_cmd;
          }

    }

    void cf_pose_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      global_xyz_meas[0] = msg->data[0];
      global_xyz_meas[1] = msg->data[1];
      global_xyz_meas[2] = msg->data[2];

      body_rpy_meas[0] = msg->data[3];
      body_rpy_meas[1] = msg->data[4];
      body_rpy_meas[2] = msg->data[5];

      //body rpy meas 0 1 2 를 기반으로 R_B 생성
      Eigen::Matrix3d Rz, Ry, Rx;

      Rz << cos(body_rpy_meas[2]), -sin(body_rpy_meas[2]), 0,
            sin(body_rpy_meas[2]),  cos(body_rpy_meas[2]), 0,
                 0  ,       0  , 1;

      Ry << cos(body_rpy_meas[1]), 0, sin(body_rpy_meas[1]),
                   0 , 1,     0,
           -sin(body_rpy_meas[1]), 0, cos(body_rpy_meas[1]);

      Rx << 1,      0       ,       0,
            0, cos(body_rpy_meas[0]), -sin(body_rpy_meas[0]),
            0, sin(body_rpy_meas[0]),  cos(body_rpy_meas[0]);

      R_B = Rz * Ry * Rx;
    }

    void global_EE_xyz_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_EE_xyz_vel_meas[0] = msg->data[0];
      global_EE_xyz_vel_meas[1] = msg->data[1];
      global_EE_xyz_vel_meas[2] = msg->data[2];

      body_omega_meas[0] = msg->data[3];
      body_omega_meas[1] = msg->data[4];
      body_omega_meas[2] = msg->data[5];
    }

    void force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg){
      global_force_meas[0] = msg->force.x;
      global_force_meas[1] = msg->force.y;
      global_force_meas[2] = msg->force.z;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_des_xyzYaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Chat_rpy_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_force_ctrl_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_normal_hat_publisher_;


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_vel_subscriber_;



  size_t count_;
  Eigen::VectorXd global_des_xyzYaw = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd chat_des_vel_xyzYaw = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd global_des_vel_xyzYaw = Eigen::VectorXd::Zero(4);


  Eigen::VectorXd chat_des_vel_adm = Eigen::VectorXd::Zero(4);
  Eigen::Vector3d global_xyz_meas;
  Eigen::Vector3d body_rpy_meas;
  Eigen::Vector3d global_rpy_meas;
  Eigen::Vector3d body_omega_meas;
  Eigen::Matrix3d R_B;
  Eigen::Matrix3d R_Chat;
  Eigen::Matrix3d R_C;

  Eigen::Vector3d chat_force_des = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_force_des = Eigen::Vector3d::Zero();
  Eigen::Vector3d chat_force_meas = Eigen::Vector3d::Zero();
  Eigen::Vector3d chat_force_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d chat_force_error_prev = Eigen::Vector3d::Zero();
  Eigen::Vector3d chat_force_error_dot = Eigen::Vector3d::Zero();
  Eigen::Vector3d chat_force_error_dot_raw = Eigen::Vector3d::Zero();

  Eigen::Vector3d Chat_x;
  Eigen::Vector3d Chat_y;
  Eigen::Vector3d Chat_z;


  Eigen::Vector3d body_force_meas;
  Eigen::Vector3d global_force_meas;


  Eigen::Vector3d normal_vector_hat = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_normal_vector_hat = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_vector_hat_normalized = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_EE_force_meas = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_EE_force_normal_meas = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_xi = Eigen::Vector3d::Zero();


  Eigen::Vector3d global_EE_xyz_vel_meas;

  double time_cnt;
  double time_real;
  double rho = 100;
  double global_yaw_cmd;

  double global_error_yaw_prev;


  FilteredVector force_dot_filter;
  FilteredVector normal_vector_filter;

  bool contact_flag = false;
  bool estimation_flag = false;

  Eigen::Matrix3d virtual_damper = (Eigen::Vector3d(5, 0, 0)).asDiagonal();
  Eigen::Matrix3d virtual_spring = (Eigen::Vector3d(0.1, 0, 0)).asDiagonal();

  Eigen::Vector3d Chat_rpy;


  double control_loop_hz = 0;
  double numerical_calc_loop_hz = 0;
  Eigen::Vector4d position_delta_cmd = Eigen::Vector4d::Zero();
  double force_delta_cmd = 0;
  double force_lpf_cof = 0;
  double normal_vector_estimator_lpf_cof = 0;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trajectory_generator>());
  rclcpp::shutdown();
  return 0;
}
