#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <interfaces/msg/serial_receive_data.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {
uint8_t crc8Poly07(const uint8_t *data, size_t n) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      const bool msb = (crc & 0x80U) != 0U;
      crc = static_cast<uint8_t>(crc << 1U);
      if (msb) {
        crc ^= 0x07U;
      }
    }
  }
  return crc;
}

speed_t baudToTermios(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}
}  // namespace

class DjiCSerialBridgeNode : public rclcpp::Node {
public:
  DjiCSerialBridgeNode() : Node("dji_c_serial_bridge") {
    port_ = declare_parameter("port", std::string("/dev/ttyUSB0"));
    baud_ = declare_parameter("baud", 115200);
    tx_topic_ = declare_parameter("tx_topic", std::string("serial/tx_packet"));
    rx_topic_ = declare_parameter("rx_topic", std::string("serial/receive"));

    rx_pub_ = create_publisher<interfaces::msg::SerialReceiveData>(rx_topic_, 20);
    tx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      tx_topic_, 50,
      std::bind(&DjiCSerialBridgeNode::onTxPacket, this, std::placeholders::_1));

    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&DjiCSerialBridgeNode::onIoTimer, this));

    openPort();
  }

  ~DjiCSerialBridgeNode() override {
    closePort();
  }

private:
  void openPort() {
    if (fd_ >= 0) {
      return;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "open serial failed: %s, err=%s", port_.c_str(), std::strerror(errno));
      return;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      closePort();
      return;
    }

    cfmakeraw(&tio);
    const speed_t spd = baudToTermios(baud_);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      closePort();
      return;
    }

    RCLCPP_INFO(get_logger(), "serial opened: %s @ %d", port_.c_str(), baud_);
  }

  void closePort() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void onTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (fd_ < 0 || !msg) {
      return;
    }
    if (msg->data.empty()) {
      return;
    }

    const auto *ptr = reinterpret_cast<const uint8_t *>(msg->data.data());
    const ssize_t n = static_cast<ssize_t>(msg->data.size());
    const ssize_t wr = ::write(fd_, ptr, static_cast<size_t>(n));
    if (wr < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "serial write failed: %s", std::strerror(errno));
    }
  }

  void onIoTimer() {
    if (fd_ < 0) {
      openPort();
      return;
    }

    std::array<uint8_t, 512> tmp{};
    const ssize_t n = ::read(fd_, tmp.data(), tmp.size());
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "serial read failed: %s", std::strerror(errno));
        closePort();
      }
      return;
    }
    if (n == 0) {
      return;
    }

    rx_buf_.insert(rx_buf_.end(), tmp.begin(), tmp.begin() + n);
    parseFrames();
  }

  void parseFrames() {
    constexpr size_t kFrameSize = 24;
    while (rx_buf_.size() >= kFrameSize) {
      auto it = std::find(rx_buf_.begin(), rx_buf_.end(), 0xA5);
      if (it == rx_buf_.end()) {
        rx_buf_.clear();
        return;
      }

      const size_t start = static_cast<size_t>(it - rx_buf_.begin());
      if (rx_buf_.size() - start < kFrameSize) {
        if (start > 0) {
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(start));
        }
        return;
      }

      const uint8_t *f = rx_buf_.data() + start;
      if (f[23] != 0x5A || f[1] != 0x01 || f[2] != 0x12) {
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(start + 1));
        continue;
      }

      const uint8_t crc = crc8Poly07(f, 22);
      if (crc != f[22]) {
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(start + 1));
        continue;
      }

      float yaw = 0.0F;
      float pitch = 0.0F;
      float yaw_rate = 0.0F;
      float pitch_rate = 0.0F;
      std::memcpy(&yaw, f + 4, sizeof(float));
      std::memcpy(&pitch, f + 8, sizeof(float));
      std::memcpy(&yaw_rate, f + 12, sizeof(float));
      std::memcpy(&pitch_rate, f + 16, sizeof(float));

      const uint8_t flags = f[20];
      const uint8_t mode = f[21];

      interfaces::msg::SerialReceiveData msg;
      msg.header.stamp = now();
      msg.mode = mode;
      msg.bullet_speed = 0.0F;
      msg.enemy_air_support_active = (flags & 0x80U) != 0U;
      msg.roll = 0.0F;
      msg.yaw = yaw;
      msg.pitch = pitch;
      (void)yaw_rate;
      (void)pitch_rate;
      rx_pub_->publish(msg);

      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(start + kFrameSize));
    }
  }

  std::string port_;
  int baud_{115200};
  std::string tx_topic_;
  std::string rx_topic_;

  int fd_{-1};
  std::vector<uint8_t> rx_buf_;

  rclcpp::Publisher<interfaces::msg::SerialReceiveData>::SharedPtr rx_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DjiCSerialBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
