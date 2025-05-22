#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;

class DataLoggerNode : public rclcpp::Node
{
public:
  DataLoggerNode() : Node("data_logging")
  {
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", qos_settings,
      std::bind(&DataLoggerNode::cf_pose_callback, this, std::placeholders::_1));

    global_xyz_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pen/EE_des_xyzYaw", qos_settings,
      std::bind(&DataLoggerNode::global_xyz_cmd_callback, this, std::placeholders::_1));

    force_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/ee/force_wrench", qos_settings,
      std::bind(&DataLoggerNode::force_callback, this, std::placeholders::_1));

    logging_data_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging", qos_settings);

    data_logging.resize(17);  // pose(7) + EE_des_xyzYaw(4) + force(6)

    start_time_ = this->now();
    create_csv_file();

    timer_ = this->create_wall_timer(5ms, std::bind(&DataLoggerNode::publish_callback, this));
  }

  ~DataLoggerNode()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV 저장 종료");
    }
  }

private:
  void cf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    data_logging(0) = msg->pose.position.x;
    data_logging(1) = msg->pose.position.y;
    data_logging(2) = msg->pose.position.z;
    data_logging(3) = msg->pose.orientation.x;
    data_logging(4) = msg->pose.orientation.y;
    data_logging(5) = msg->pose.orientation.z;
    data_logging(6) = msg->pose.orientation.w;
  }

  void global_xyz_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    data_logging(7) = msg->data[0];
    data_logging(8) = msg->data[1];
    data_logging(9) = msg->data[2];
    data_logging(10) = msg->data[3];
  }

  void force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    data_logging(11) = msg->force.x;
    data_logging(12) = msg->force.y;
    data_logging(13) = msg->force.z;
    data_logging(14) = msg->torque.x;
    data_logging(15) = msg->torque.y;
    data_logging(16) = msg->torque.z;
  }

  void publish_callback()
  {
    std_msgs::msg::Float64MultiArray msg;
    for (int i = 0; i < data_logging.size(); ++i) {
      msg.data.push_back(data_logging(i));
    }
    logging_data_publisher_->publish(msg);

    double t = (this->now() - start_time_).seconds();

    if (csv_file_.is_open()) {
      csv_file_ << std::fixed << std::setprecision(6) << t;
      for (int i = 0; i < data_logging.size(); ++i) {
        csv_file_ << "," << data_logging(i);
      }
      csv_file_ << "\n";
    }
  }

  void create_csv_file()
  {
    std::string dir = std::string(std::getenv("HOME")) + "/sitl_ws/src/test_pkg/bag/";
    std::string filename = "log_" + get_current_time_string() + ".csv";
    std::string full_path = dir + filename;

    csv_file_.open(full_path);

    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "CSV 파일 생성 실패: %s", full_path.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "CSV 저장 시작: %s", full_path.c_str());
      csv_file_ << "time,x,y,z,qx,qy,qz,qw,cmd_x,cmd_y,cmd_z,cmd_yaw,"
                   "force_x,force_y,force_z,torque_x,torque_y,torque_z\n";
    }
  }

  std::string get_current_time_string()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    std::tm *tm_ptr = std::localtime(&now_t);

    std::ostringstream oss;
    oss << std::put_time(tm_ptr, "%m%d_%H%M%S");
    return oss.str();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr logging_data_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::VectorXd data_logging;
  rclcpp::Time start_time_;
  std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
