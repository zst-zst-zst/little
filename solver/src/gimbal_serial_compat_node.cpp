#include <rclcpp/rclcpp.hpp>

#include <interfaces/msg/gimbal_cmd.hpp>
#include <interfaces/msg/serial_receive_data.hpp>

#include <array>
#include <chrono>

#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"

class GimbalSerialCompatNode : public rclcpp::Node {
public:
  GimbalSerialCompatNode() : Node("gimbal_serial_compat") {
    device_ = declare_parameter("port", std::string("/dev/ttyUSB0"));
    baud_ = declare_parameter("baud", 115200);
    rx_topic_ = declare_parameter("rx_topic", std::string("serial/receive"));
    tx_topic_ = declare_parameter("tx_topic", std::string("solver/gimbal_cmd"));
    yaw_sign_ = static_cast<float>(declare_parameter("yaw_sign", 1.0));
    pitch_sign_ = static_cast<float>(declare_parameter("pitch_sign", 1.0));

    rx_pub_ = create_publisher<interfaces::msg::SerialReceiveData>(rx_topic_, 50);
    tx_sub_ = create_subscription<interfaces::msg::GimbalCmd>(
      tx_topic_, 50,
      std::bind(&GimbalSerialCompatNode::onCmd, this, std::placeholders::_1));

    io_timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&GimbalSerialCompatNode::onIo, this));

    if (!serial_.open(device_, baud_)) {
      RCLCPP_WARN(get_logger(), "serial open failed at startup: %s @ %d", device_.c_str(), baud_);
    } else {
      RCLCPP_INFO(get_logger(), "serial opened: %s @ %d", device_.c_str(), baud_);
    }
  }

private:
  void onCmd(const interfaces::msg::GimbalCmd::SharedPtr msg) {
    if (!msg) {
      return;
    }
    if (!ensureSerialOpen()) {
      return;
    }

    gimbal_serial::GimbalCommand cmd;
    cmd.yaw = yaw_sign_ * static_cast<float>(msg->yaw);
    cmd.pitch = pitch_sign_ * static_cast<float>(msg->pitch);
    cmd.yaw_rate = yaw_sign_ * static_cast<float>(msg->yaw_diff);
    cmd.pitch_rate = pitch_sign_ * static_cast<float>(msg->pitch_diff);
    cmd.timestamp = static_cast<uint32_t>(this->get_clock()->now().nanoseconds() / 1000000ULL);

    uint8_t tx[gimbal_serial::kTxFrameSize] = {0};
    gimbal_serial::packGimbalCommand(cmd, tx);
    (void)serial_.write(tx, static_cast<int>(gimbal_serial::kTxFrameSize));
  }

  void onIo() {
    if (!ensureSerialOpen()) {
      return;
    }

    std::array<uint8_t, 128> buf{};
    int n = serial_.read(buf.data(), static_cast<int>(buf.size()), 0);
    if (n < 0) {
      serial_.close();
      return;
    }
    if (n == 0) {
      return;
    }

    gimbal_serial::GimbalState st;
    while (parser_.push(buf.data(), static_cast<size_t>(n), &st)) {
      interfaces::msg::SerialReceiveData out;
      out.header.stamp = now();
      out.mode = 0;
      out.bullet_speed = 0.0;
      out.enemy_air_support_active = false;
      out.roll = 0.0;
      out.yaw = st.yaw;
      out.pitch = st.pitch;
      rx_pub_->publish(out);
      n = 0;
    }
  }

  bool ensureSerialOpen() {
    if (serial_.isOpen()) {
      return true;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (has_last_reopen_try_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_reopen_try_tp_).count() < 500) {
      return false;
    }
    has_last_reopen_try_ = true;
    last_reopen_try_tp_ = now_tp;

    if (!serial_.reopen()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "serial reopen failed: %s @ %d", device_.c_str(), baud_);
      return false;
    }
    RCLCPP_INFO(get_logger(), "serial reopened: %s @ %d", device_.c_str(), baud_);
    return true;
  }

  std::string device_;
  int baud_{115200};
  std::string rx_topic_;
  std::string tx_topic_;
  float yaw_sign_{1.0f};
  float pitch_sign_{1.0f};
  bool has_last_reopen_try_{false};
  std::chrono::steady_clock::time_point last_reopen_try_tp_{};

  gimbal_serial::SerialPort serial_;
  gimbal_serial::FrameParser parser_;

  rclcpp::Publisher<interfaces::msg::SerialReceiveData>::SharedPtr rx_pub_;
  rclcpp::Subscription<interfaces::msg::GimbalCmd>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalSerialCompatNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
