#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class CsvPublisher : public rclcpp::Node
{
public:
  CsvPublisher()
  : Node("csv_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/data_logging_msg", 10);

    // 시작 시간 (초) 설정
    start_time_sec_ = 2.5;  // 예: 2.5초 이후부터 재생
    load_csv();
    publish_loop();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  std::vector<std::pair<uint64_t, std::vector<float>>> data_vector_;
  double start_time_sec_;

  void load_csv()
  {
    std::string filepath = std::string(std::getenv("HOME")) +
                           "/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/" +
                           "rosbag2_2025_05_14-01_19_05.csv";

    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at: %s", filepath.c_str());
      return;
    }

    std::string line;
    std::getline(file, line);  // skip header

    uint64_t start_time_ns = static_cast<uint64_t>(start_time_sec_ * 1e9);
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<float> row_data;
      uint64_t timestamp = 0;
      int col_index = 0;

      while (std::getline(ss, cell, ',')) {
        if (col_index == 0) {
          timestamp = std::stoull(cell);
        } else if (col_index >= 2) {
          row_data.push_back(static_cast<float>(std::stod(cell)));
        }
        ++col_index;
      }

      if (timestamp >= start_time_ns) {
        data_vector_.emplace_back(timestamp, row_data);
      }
    }

    if (data_vector_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No data found after %.3f seconds", start_time_sec_);
    }
  }

  void publish_loop()
  {
    std::thread([this]() {
      for (size_t i = 0; i < data_vector_.size(); ++i) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = data_vector_[i].second;
        publisher_->publish(msg);

        if (i + 1 < data_vector_.size()) {
          uint64_t t_curr = data_vector_[i].first;
          uint64_t t_next = data_vector_[i + 1].first;
          uint64_t delta_ns = t_next - t_curr;

          if (delta_ns > 0 && delta_ns < 1e9) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(delta_ns));
          }
        }
      }
    }).detach();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvPublisher>());
  rclcpp::shutdown();
  return 0;
}
