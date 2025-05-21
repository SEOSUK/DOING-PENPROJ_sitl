#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class DataDecryptorCSV : public rclcpp::Node
{
public:
  DataDecryptorCSV()
  : Node("data_decryptor_csv"), current_index_(0)
  {
    std::string bag_name = this->declare_parameter<std::string>("bag_name", "log_default.csv");

    std::string filepath = std::string(std::getenv("HOME")) +
                           "/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/" + bag_name;

    if (!load_csv(filepath)) {
      RCLCPP_ERROR(this->get_logger(), "CSV 파일을 불러오지 못했습니다.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "CSV 데이터 %lu 개 로드됨", csv_data_.size());

    cf_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cf2/pose", 10);
    cmd_xyz_yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_des_xyzYaw", 10);
    force_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", 10);

    double period_sec = (csv_data_.size() >= 2) ?
                        (csv_data_[1][0] - csv_data_[0][0]) : 0.005;

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&DataDecryptorCSV::publish_next_row, this));
  }

private:
  bool load_csv(const std::string &filepath)
  {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::string line;
    std::getline(file, line);  // skip header

    while (std::getline(file, line)) {
      std::istringstream ss(line);
      std::string token;
      std::vector<double> row;

      while (std::getline(ss, token, ',')) {
        try {
          row.push_back(std::stod(token));
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "변환 실패한 값: %s", token.c_str());
          return false;
        }
      }

      if (row.size() == 18) {
        csv_data_.push_back(row);
      }
    }

    return !csv_data_.empty();
  }

  void publish_next_row()
  {
    if (current_index_ >= csv_data_.size()) {
      RCLCPP_INFO(this->get_logger(), "CSV 데이터 재생 완료.");
      rclcpp::shutdown();
      return;
    }

    const auto &row = csv_data_[current_index_++];

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = row[1];
    pose_msg.pose.position.y = row[2];
    pose_msg.pose.position.z = row[3];
    pose_msg.pose.orientation.x = row[4];
    pose_msg.pose.orientation.y = row[5];
    pose_msg.pose.orientation.z = row[6];
    pose_msg.pose.orientation.w = row[7];
    cf_pose_publisher_->publish(pose_msg);

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = {row[8], row[9], row[10], row[11]};
    cmd_xyz_yaw_publisher_->publish(cmd_msg);

    geometry_msgs::msg::Wrench wrench_msg;
    wrench_msg.force.x = row[12];
    wrench_msg.force.y = row[13];
    wrench_msg.force.z = row[14];
    wrench_msg.torque.x = row[15];
    wrench_msg.torque.y = row[16];
    wrench_msg.torque.z = row[17];
    force_wrench_publisher_->publish(wrench_msg);
  }

  std::vector<std::vector<double>> csv_data_;
  size_t current_index_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_xyz_yaw_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_wrench_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataDecryptorCSV>());
  rclcpp::shutdown();
  return 0;
}
