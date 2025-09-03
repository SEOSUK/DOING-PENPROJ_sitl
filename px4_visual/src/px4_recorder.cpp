#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <csignal>

namespace fs = std::filesystem;

class Px4Recorder : public rclcpp::Node {
public:
  Px4Recorder()
  : Node("px4_recorder")
  {
    topic_name_ = declare_parameter<std::string>("topic_name", "/data_logging_msg");
    base_dir_   = expand_user(declare_parameter<std::string>("base_dir", "~/bag"));

    try { fs::create_directories(base_dir_); }
    catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "폴더 생성 실패 [%s]: %s", base_dir_.c_str(), e.what());
      rclcpp::shutdown(); return;
    }

    filepath_ = (fs::path(base_dir_) / make_timestamped_filename()).string();
    file_.open(filepath_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      RCLCPP_FATAL(get_logger(), "파일 열기 실패: %s", filepath_.c_str());
      rclcpp::shutdown(); return;
    }

    // 큰 버퍼로 I/O 효율 개선 (약 4MB)
    static std::vector<char> io_buffer(4 * 1024 * 1024);
    file_.rdbuf()->pubsetbuf(io_buffer.data(), static_cast<std::streamsize>(io_buffer.size()));

    // 헤더: MATLAB 스크립트와 정렬
    file_ << "t_nsec_from_start,dt_nsec,rcl_stamp_sec,rcl_stamp_nsec,data...\n";
    file_.flush();

    // 시작 기준 시각 (ROS time)
    start_time_ = now_ros();
    last_time_  = start_time_;

    using std::placeholders::_1;
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic_name_, rclcpp::SensorDataQoS(),
      std::bind(&Px4Recorder::callback, this, _1));

    RCLCPP_INFO(get_logger(), "토픽 [%s] 로그를 %s 에 저장합니다.", topic_name_.c_str(), filepath_.c_str());
  }

  ~Px4Recorder() override {
    close_file();
  }

private:
  rclcpp::Time now_ros() { return this->get_clock()->now(); }

  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // ROS Clock 기반의 현재 시각
    const rclcpp::Time t_now = now_ros();

    // 상대 시각(ns) 및 샘플 간격(ns)
    const uint64_t t_from_start_ns = static_cast<uint64_t>((t_now - start_time_).nanoseconds());
    const uint64_t dt_ns           = static_cast<uint64_t>((t_now - last_time_).nanoseconds());
    last_time_ = t_now;

    // 한 줄 구성
    // 1:t_from_start_ns, 2:dt_ns, 3:rcl_stamp_sec, 4:rcl_stamp_nsec, 이후 data[]
    file_ << t_from_start_ns << "," << dt_ns << ","
          << t_now.seconds() << "," << (t_now.nanoseconds() % 1000000000ULL);

    for (float v : msg->data) file_ << "," << v;
    file_ << "\n";

    // 주기적으로 flush (전원 차단 대비)
    if (++line_count_ % 100 == 0) file_.flush();
  }

  static std::string expand_user(const std::string &path) {
    if (!path.empty() && path[0] == '~') {
      const char *home = std::getenv("HOME");
      if (home) return std::string(home) + path.substr(1);
    }
    return path;
  }

  static std::string make_timestamped_filename() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << "px4_log_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    return oss.str();
  }

  void close_file() {
    if (file_.is_open()) {
      file_.flush();
      file_.close();
      RCLCPP_INFO(get_logger(), "파일 저장 종료: %s", filepath_.c_str());
    }
  }

  std::string topic_name_;
  std::string base_dir_;
  std::string filepath_;
  std::ofstream file_;
  rclcpp::Time start_time_, last_time_;
  size_t line_count_{0};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4Recorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
