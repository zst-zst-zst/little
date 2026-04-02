#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/dnn.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "interfaces/msg/debug_laser.hpp"
#include "interfaces/msg/gimbal_cmd.hpp"
#include "interfaces/msg/serial_receive_data.hpp"
#include "lt_control/controller.h"

extern "C" {
void *fyt_trt_create(const char *engine_path, float conf, float nms);
void fyt_trt_destroy(void *ptr);
int fyt_trt_infer(
  void *ptr,
  const unsigned char *bgr,
  int width,
  int height,
  float *out_boxes,
  int max_det);
}

namespace {
struct LaserState {
  double progress{0.0};
  int lock_count{0};
  bool lock_active{false};
  double lock_remaining{0.0};
  int stage{1};
};

struct Detection {
  cv::Rect box;
  int class_id{0};
  float score{0.0F};
};

class LaserRule {
public:
  LaserRule() = default;

  LaserState update(double dt, bool valid_hit, bool laser_on) {
    dt = std::clamp(dt, 0.0, 0.2);

    if (lock_remaining_ > 0.0) {
      lock_remaining_ = std::max(0.0, lock_remaining_ - dt);
      if (lock_remaining_ > 0.0) {
        progress_ = 0.0;
        continuous_illum_ = 0.0;
        judge_count_ = 0;
        return snapshot();
      }
    }

    if (!laser_on || lock_count_ >= 3) {
      progress_ = 0.0;
      continuous_illum_ = 0.0;
      judge_count_ = 0;
      return snapshot();
    }

    if (!valid_hit) {
      progress_ = std::max(0.0, progress_ - 0.5 * dt);
      continuous_illum_ = 0.0;
      judge_count_ = 0;
      return snapshot();
    }

    continuous_illum_ += dt;
    while (continuous_illum_ >= 0.1) {
      continuous_illum_ -= 0.1;
      judge_count_ += 1;
      progress_ += static_cast<double>(judge_count_);
    }

    const double p0 = lock_count_ <= 0 ? 50.0 : 100.0;
    if (progress_ >= p0) {
      lock_count_ += 1;
      lock_remaining_ = 45.0;
      progress_ = 0.0;
      continuous_illum_ = 0.0;
      judge_count_ = 0;
    }

    return snapshot();
  }

private:
  LaserState snapshot() const {
    LaserState s;
    s.progress = progress_;
    s.lock_count = lock_count_;
    s.lock_active = lock_remaining_ > 0.0;
    s.lock_remaining = lock_remaining_;
    s.stage = lock_count_ <= 0 ? 1 : (lock_count_ == 1 ? 2 : 3);
    return s;
  }

  double progress_{0.0};
  double continuous_illum_{0.0};
  int judge_count_{0};
  int lock_count_{0};
  double lock_remaining_{0.0};
};
}  // namespace

class SolverNode : public rclcpp::Node {
public:
  SolverNode() : Node("solver") {
    use_gui_ = declare_parameter("use_gui", true);
    tracker_type_ = declare_parameter("tracker_type", std::string("CSRT"));
    yaw_gain_ = declare_parameter("yaw_gain", 1.0);
    pitch_gain_ = declare_parameter("pitch_gain", 1.0);
    yaw_sign_ = declare_parameter("yaw_sign", 1.0);
    pitch_sign_ = declare_parameter("pitch_sign", -1.0);
    hit_rx_ = declare_parameter("hit_window_ratio_x", 0.08);
    hit_ry_ = declare_parameter("hit_window_ratio_y", 0.08);
    auto_reselect_ = declare_parameter("auto_reselect", true);
    use_serial_feedback_ = declare_parameter("use_serial_feedback", false);
    init_yaw_deg_ = declare_parameter("init_yaw_deg", 0.0);
    init_pitch_deg_ = declare_parameter("init_pitch_deg", 0.0);
    show_window_ = declare_parameter("publish_debug_window", true);

    color_filter_enabled_ = declare_parameter("color_filter.enabled", true);
    self_color_ = declare_parameter("color_filter.self_color", std::string("red"));
    enemy_color_manual_ = declare_parameter("color_filter.enemy_color", std::string("auto"));
    min_enemy_color_ratio_ = declare_parameter("color_filter.min_enemy_color_ratio", 0.02);

    distance_gate_enabled_ = declare_parameter("distance_gate.enabled", true);
    min_distance_m_ = declare_parameter("distance_gate.min_m", 10.0);
    max_distance_m_ = declare_parameter("distance_gate.max_m", 25.0);
    target_height_m_ = declare_parameter("distance_gate.target_height_m", 0.12);
    preferred_distance_m_ = declare_parameter("distance_gate.preferred_m", 5.0);
    preferred_distance_sigma_m_ = declare_parameter("distance_gate.preferred_sigma_m", 1.0);

    laser_comp_enabled_ = declare_parameter("laser_comp.enabled", true);
    laser_offset_right_m_ = declare_parameter("laser_comp.offset_right_m", 0.0);
    laser_offset_up_m_ = declare_parameter("laser_comp.offset_up_m", 0.0);

    detector_mode_ = declare_parameter("detector.mode", std::string("tracker"));
    detector_backend_ = declare_parameter("detector.backend", std::string("opencv"));
    yolo_model_path_ = declare_parameter("detector.yolo.model_path", std::string(""));
    yolo_engine_path_ = declare_parameter("detector.yolo.engine_path", std::string("/home/zst/FYT/model/yolo.engine"));
    yolo_input_w_ = declare_parameter("detector.yolo.input_w", 640);
    yolo_input_h_ = declare_parameter("detector.yolo.input_h", 640);
    yolo_conf_thres_ = declare_parameter("detector.yolo.conf_thres", 0.35);
    yolo_nms_thres_ = declare_parameter("detector.yolo.nms_thres", 0.45);
    yolo_layout_ = declare_parameter("detector.yolo.layout", std::string("auto"));
    yolo_target_class_id_ = declare_parameter("detector.yolo.target_class_id", -1);
    yolo_max_det_ = declare_parameter("detector.yolo.max_det", 20);
    yolo_max_targets_for_valid_ = declare_parameter("detector.yolo.max_targets_for_valid", 12);
    const auto allowed_ids = declare_parameter("detector.yolo.allowed_class_ids", std::vector<int64_t>{0, 1});
    yolo_allowed_class_ids_.clear();
    yolo_allowed_class_ids_.reserve(allowed_ids.size());
    for (const auto id : allowed_ids) {
      yolo_allowed_class_ids_.push_back(static_cast<int>(id));
    }
    yolo_detect_interval_ = declare_parameter("detector.yolo.detect_interval", 3);
    yolo_single_target_mode_ = declare_parameter("detector.yolo.single_target_mode", true);
    yolo_class_color_gate_enabled_ = declare_parameter("detector.yolo.class_color_gate.enabled", true);
    yolo_red_class_id_ = declare_parameter("detector.yolo.class_color_gate.red_class_id", 0);
    yolo_blue_class_id_ = declare_parameter("detector.yolo.class_color_gate.blue_class_id", 1);
    yolo_class_lock_enabled_ = declare_parameter("detector.yolo.class_lock.enabled", true);
    yolo_class_switch_confirm_frames_ =
      declare_parameter("detector.yolo.class_lock.switch_confirm_frames", 8);

    yolo_min_area_px_ = declare_parameter("detector.selector.min_area_px", 120.0);
    yolo_max_area_ratio_ = declare_parameter("detector.selector.max_area_ratio", 0.75);
    yolo_w_conf_ = declare_parameter("detector.selector.w_conf", 0.55);
    yolo_w_iou_ = declare_parameter("detector.selector.w_iou", 0.20);
    yolo_w_center_ = declare_parameter("detector.selector.w_center", 0.20);
    yolo_w_size_ = declare_parameter("detector.selector.w_size", 0.05);
    yolo_w_distance_ = declare_parameter("detector.selector.w_distance", 0.0);

    prediction_enabled_ = declare_parameter("prediction.enabled", true);
    lead_time_s_ = declare_parameter("prediction.lead_time_s", 0.08);
    max_lead_px_ = declare_parameter("prediction.max_lead_px", 80.0);
    center_smooth_alpha_ = declare_parameter("prediction.center_smooth_alpha", 0.55);
    deadzone_px_ = declare_parameter("prediction.deadzone_px", 1.5);
    vel_jump_thres_px_s_ = declare_parameter("prediction.vel_jump_thres_px_s", 250.0);
    confidence_decay_ = declare_parameter("prediction.confidence_decay", 0.25);
    confidence_recover_ = declare_parameter("prediction.confidence_recover", 0.08);
    min_prediction_confidence_ = declare_parameter("prediction.min_confidence", 0.2);

    lost_hold_enabled_ = declare_parameter("lost_hold.enabled", true);
    lost_hold_s_ = declare_parameter("lost_hold.hold_s", 0.2);

    score_stability_window_s_ = declare_parameter("score.stability_window_s", 0.12);

    serial_bridge_enabled_ = declare_parameter("serial_bridge.enabled", false);
    serial_bridge_topic_ = declare_parameter("serial_bridge.tx_topic", std::string("serial/tx_packet"));
    serial_bridge_protocol_ = declare_parameter("serial_bridge.protocol", std::string("dji_c_v1"));

    line_motion_enabled_ = declare_parameter("line_motion.enabled", true);
    line_axis_ = declare_parameter("line_motion.axis", std::string("x"));
    line_center_alpha_ = declare_parameter("line_motion.center_alpha", 0.15);

    rotate_image_180_ = declare_parameter("image_transform.rotate_180", false);

    control_backend_ = declare_parameter("control.backend", std::string("solver"));
    std::transform(
      control_backend_.begin(), control_backend_.end(), control_backend_.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    use_lt_controller_ = (control_backend_ == "lasertracking" || control_backend_ == "lt");

    lt_cfg_.kp = declare_parameter("control.kp", 1.2);
    lt_cfg_.deadband_px = declare_parameter("control.deadband_px", 1.0);
    lt_cfg_.max_angle_rate = declare_parameter("control.max_angle_rate", 180.0);
    lt_cfg_.lowpass_alpha = declare_parameter("control.lowpass_alpha", 0.45);
    lt_cfg_.yaw_sign = declare_parameter("control.yaw_sign", -1.0);
    lt_cfg_.pitch_sign = declare_parameter("control.pitch_sign", -1.0);
    lt_cfg_.ctrl_dt_nominal_ms = declare_parameter("control.ctrl_dt_nominal_ms", 10.0);
    lt_cfg_.ctrl_dt_min_ms = declare_parameter("control.ctrl_dt_min_ms", 1.0);
    lt_cfg_.ctrl_dt_max_ms = declare_parameter("control.ctrl_dt_max_ms", 100.0);
    lt_cfg_.use_velocity_ff = declare_parameter("control.use_velocity_ff", true);
    lt_cfg_.ff_alpha = declare_parameter("control.ff_alpha", 0.25);
    lt_cfg_.ff_rate_max = declare_parameter("control.ff_rate_max", 150.0);
    lt_cfg_.ff_dt_max_ms = declare_parameter("control.ff_dt_max_ms", 80.0);
    lt_cfg_.use_damping = declare_parameter("control.use_damping", true);
    lt_cfg_.damping_source = declare_parameter("control.damping_source", std::string("meas"));
    lt_cfg_.damping_kd = declare_parameter("control.damping_kd", 0.02);
    lt_cfg_.damping_dt_max_ms = declare_parameter("control.damping_dt_max_ms", 120.0);
    lt_cfg_.scan_enable = declare_parameter("control.scan_enable", true);
    lt_cfg_.scan_radius_deg = declare_parameter("control.scan_radius_deg", 12.0);
    lt_cfg_.scan_rate_hz = declare_parameter("control.scan_rate_hz", 0.2);
    lt_cfg_.scan_pattern = declare_parameter("control.scan_pattern", std::string("spiral"));
    lt_cfg_.scan_spacing_deg = declare_parameter("control.scan_spacing_deg", 6.05);
    lt_cfg_.scan_speed_deg_s = declare_parameter("control.scan_speed_deg_s", 25.0);
    lt_cfg_.scan_r_max_deg = declare_parameter("control.scan_r_max_deg", 15.0);
    lt_cfg_.scan_spiral_return = declare_parameter("control.scan_spiral_return", false);
    lt_cfg_.scan_k_yaw = declare_parameter("control.scan_k_yaw", 2.5);
    lt_cfg_.scan_k_pitch = declare_parameter("control.scan_k_pitch", 0.75);
    lt_cfg_.scan_enter_delay_ms = declare_parameter("control.scan_enter_delay_ms", 180.0);
    lt_cfg_.scan_reacq_confirm_frames = declare_parameter("control.scan_reacq_confirm_frames", 2);
    lt_cfg_.startup_check_frames = declare_parameter("control.startup_check_frames", 5);
    lt_cfg_.startup_home_pitch = declare_parameter("control.startup_home_pitch", 0.0);
    lt_cfg_.startup_home_yaw = declare_parameter("control.startup_home_yaw", 0.0);
    lt_cfg_.startup_prep_ms = declare_parameter("control.startup_prep_ms", 1000);
    lt_cfg_.startup_hold_ms = declare_parameter("control.startup_hold_ms", 200);
    lt_cfg_.startup_home_ms = declare_parameter("control.startup_home_ms", 600);
    lt_cfg_.startup_validate_ms = declare_parameter("control.startup_validate_ms", 200);
    lt_cfg_.startup_min_state_frames = declare_parameter("control.startup_min_state_frames", 1);
    lt_cfg_.startup_require_home = declare_parameter("control.startup_require_home", true);
    lt_cfg_.startup_home_tol_deg = declare_parameter("control.startup_home_tol_deg", 1.0);
    lt_cfg_.startup_home_max_extra_ms = declare_parameter("control.startup_home_max_extra_ms", 1000);
    lt_cfg_.startup_validate_first = declare_parameter("control.startup_validate_first", true);
    lt_cfg_.startup_allow_early_exit = declare_parameter("control.startup_allow_early_exit", true);

    lt_boresight_.u_l = declare_parameter("control.boresight.u_l", -1.0);
    lt_boresight_.v_l = declare_parameter("control.boresight.v_l", -1.0);

    if (use_lt_controller_) {
      lt_controller_ = std::make_unique<lt_control::Controller>(lt_cfg_);
      RCLCPP_INFO(get_logger(), "control.backend=lasertracking enabled");
    }

    std::transform(
      detector_backend_.begin(), detector_backend_.end(), detector_backend_.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    if (detector_mode_ == "yolo" || detector_mode_ == "hybrid") {
      if (detector_backend_ == "tensorrt" || detector_backend_ == "trt") {
        if (yolo_engine_path_.empty()) {
          RCLCPP_ERROR(get_logger(), "detector.backend=tensorrt but detector.yolo.engine_path is empty");
        } else {
          if (!std::filesystem::exists(yolo_engine_path_)) {
            if (yolo_model_path_.empty()) {
              RCLCPP_ERROR(get_logger(), "TensorRT engine missing and detector.yolo.model_path is empty");
            } else {
              const std::string cmd =
                "/usr/src/tensorrt/bin/trtexec --onnx=" + yolo_model_path_ +
                " --saveEngine=" + yolo_engine_path_ +
                " --fp16";
              RCLCPP_WARN(get_logger(), "TensorRT engine not found, building: %s", yolo_engine_path_.c_str());
              const int ret = std::system(cmd.c_str());
              if (ret != 0) {
                RCLCPP_ERROR(get_logger(), "Build TensorRT engine failed, trtexec return=%d", ret);
              }
            }
          }

          if (std::filesystem::exists(yolo_engine_path_)) {
            const float trt_decode_conf = std::min(static_cast<float>(yolo_conf_thres_), 0.01F);
            trt_handle_ = fyt_trt_create(
              yolo_engine_path_.c_str(),
              trt_decode_conf,
              static_cast<float>(yolo_nms_thres_));
            if (trt_handle_ != nullptr) {
              yolo_ready_ = true;
              RCLCPP_INFO(
                get_logger(),
                "TensorRT engine loaded: %s (decode_conf=%.4f, post_conf=%.4f)",
                yolo_engine_path_.c_str(),
                trt_decode_conf,
                static_cast<float>(yolo_conf_thres_));
            } else {
              RCLCPP_ERROR(get_logger(), "Load TensorRT engine failed: %s", yolo_engine_path_.c_str());
            }
          }
        }
      } else {
        if (yolo_model_path_.empty()) {
          RCLCPP_ERROR(get_logger(), "detector.mode=yolo but detector.yolo.model_path is empty");
        } else {
          try {
            yolo_net_ = cv::dnn::readNetFromONNX(yolo_model_path_);
            if (yolo_net_.empty()) {
              RCLCPP_ERROR(get_logger(), "Failed to load YOLO model: %s", yolo_model_path_.c_str());
            } else {
              yolo_ready_ = true;
              RCLCPP_INFO(get_logger(), "YOLO model loaded: %s", yolo_model_path_.c_str());
              RCLCPP_INFO(get_logger(), "OpenCV version: %s", CV_VERSION);
            }
          } catch (const cv::Exception &e) {
            RCLCPP_ERROR(get_logger(), "Load YOLO model failed: %s", e.what());
          }
        }
      }
    }

    yaw_deg_ = init_yaw_deg_;
    pitch_deg_ = init_pitch_deg_;

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&SolverNode::imageCallback, this, std::placeholders::_1));

    cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      std::bind(&SolverNode::cameraInfoCallback, this, std::placeholders::_1));

    serial_sub_ = create_subscription<interfaces::msg::SerialReceiveData>(
      "serial/receive", rclcpp::SensorDataQoS(),
      std::bind(&SolverNode::serialCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal", 10);
    dbg_pub_ = create_publisher<interfaces::msg::DebugLaser>("armor_solver/laser_debug", 10);
    serial_tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(serial_bridge_topic_, 10);

    last_time_ = now();
  }

  ~SolverNode() override {
    if (trt_handle_ != nullptr) {
      fyt_trt_destroy(trt_handle_);
      trt_handle_ = nullptr;
    }
  }

private:
  cv::Ptr<cv::Tracker> createTracker() {
    if (tracker_type_ == "KCF") {
      return cv::TrackerKCF::create();
    }
    return cv::TrackerCSRT::create();
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    if (rotate_image_180_) {
      if (msg->width > 0) {
        cx_ = static_cast<double>(msg->width - 1U) - msg->k[2];
      }
      if (msg->height > 0) {
        cy_ = static_cast<double>(msg->height - 1U) - msg->k[5];
      }
    }
    has_cam_info_ = true;
  }

  void serialCallback(const interfaces::msg::SerialReceiveData::SharedPtr msg) {
    enemy_color_from_mode_ = msg->mode == 1 ? "blue" : "red";

    if (!use_serial_feedback_) {
      return;
    }
    yaw_deg_ = msg->yaw;
    pitch_deg_ = msg->pitch;
    yaw_rate_deg_s_ = 0.0;
    pitch_rate_deg_s_ = 0.0;
  }

  interfaces::msg::GimbalCmd buildLtCmd(const rclcpp::Time &stamp, bool valid, double u, double v) {
    interfaces::msg::GimbalCmd cmd;
    if (!use_lt_controller_ || !lt_controller_ || !has_cam_info_) {
      return cmd;
    }

    lt_control::TargetMeasurement meas;
    meas.valid = valid;
    meas.timestamp = static_cast<int64_t>(stamp.nanoseconds() / 1000000LL);
    meas.uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    meas.confidence = 1.0F;

    lt_control::CameraModel cam;
    cam.fx = fx_;
    cam.fy = fy_;
    cam.cx = cx_;
    cam.cy = cy_;

    lt_control::Boresight bs = lt_boresight_;
    if (bs.u_l <= 0.0 || bs.v_l <= 0.0) {
      bs.u_l = cx_;
      bs.v_l = cy_;
    }

    lt_control::GimbalState st;
    st.yaw = static_cast<float>(yaw_deg_);
    st.pitch = static_cast<float>(pitch_deg_);
    st.yaw_rate = static_cast<float>(yaw_rate_deg_s_);
    st.pitch_rate = static_cast<float>(pitch_rate_deg_s_);
    st.timestamp = static_cast<uint32_t>(stamp.nanoseconds() / 1000000ULL);

    const lt_control::GimbalCommand out = lt_controller_->update(meas, cam, bs, st);
    cmd.header.stamp = stamp;
    cmd.yaw = out.yaw;
    cmd.pitch = out.pitch;
    cmd.yaw_diff = static_cast<double>(out.yaw) - yaw_deg_;
    cmd.pitch_diff = static_cast<double>(out.pitch) - pitch_deg_;
    cmd.distance = -1.0;
    cmd.laser_power_on = true;
    cmd.laser_advice = true;
    cmd.fire_advice = true;
    cmd.laser_hit_valid = false;
    cmd.aimed_progress = 0.0;
    cmd.lock_count = 0;
    cmd.lock_active = false;
    cmd.lock_remaining = 0.0;
    cmd.progress_stage = 1;
    return cmd;
  }

  bool inHitWindow(const cv::Rect &r, int w, int h) const {
    const double u = static_cast<double>(r.x) + static_cast<double>(r.width) * 0.5;
    const double v = static_cast<double>(r.y) + static_cast<double>(r.height) * 0.5;
    const double dx = std::abs(u - 0.5 * static_cast<double>(w)) / std::max(1.0, static_cast<double>(w));
    const double dy = std::abs(v - 0.5 * static_cast<double>(h)) / std::max(1.0, static_cast<double>(h));
    return dx <= hit_rx_ && dy <= hit_ry_;
  }

  std::string currentEnemyColor() const {
    if (enemy_color_manual_ == "red" || enemy_color_manual_ == "blue") {
      return enemy_color_manual_;
    }
    if (self_color_ == "red") {
      return "blue";
    }
    if (self_color_ == "blue") {
      return "red";
    }
    return enemy_color_from_mode_;
  }

  double estimateDistanceM(const cv::Rect &r) const {
    if (r.height <= 1 || fy_ <= 1.0 || target_height_m_ <= 0.0) {
      return -1.0;
    }
    return (fy_ * target_height_m_) / static_cast<double>(r.height);
  }

  bool passColorGate(const cv::Mat &bgr, const cv::Rect &r) const {
    if (!color_filter_enabled_) {
      return true;
    }

    const cv::Rect rr = r & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (rr.width <= 1 || rr.height <= 1) {
      return false;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr(rr), hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1, red2, red_mask, blue_mask;
    cv::inRange(hsv, cv::Scalar(0, 90, 90), cv::Scalar(12, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(160, 90, 90), cv::Scalar(180, 255, 255), red2);
    cv::bitwise_or(red1, red2, red_mask);
    cv::inRange(hsv, cv::Scalar(90, 90, 90), cv::Scalar(140, 255, 255), blue_mask);

    const double area = static_cast<double>(rr.area());
    if (area <= 1.0) {
      return false;
    }

    const double red_ratio = static_cast<double>(cv::countNonZero(red_mask)) / area;
    const double blue_ratio = static_cast<double>(cv::countNonZero(blue_mask)) / area;
    const std::string enemy = currentEnemyColor();

    if (enemy == "blue") {
      return blue_ratio >= min_enemy_color_ratio_ && blue_ratio >= red_ratio;
    }
    return red_ratio >= min_enemy_color_ratio_ && red_ratio >= blue_ratio;
  }

  int stabilizeClassId(int detected_class_id) {
    if (!yolo_class_lock_enabled_ || detected_class_id < 0) {
      return detected_class_id;
    }

    if (locked_class_id_ < 0) {
      locked_class_id_ = detected_class_id;
      pending_class_id_ = -1;
      pending_class_count_ = 0;
      return locked_class_id_;
    }

    if (detected_class_id == locked_class_id_) {
      pending_class_id_ = -1;
      pending_class_count_ = 0;
      return locked_class_id_;
    }

    if (pending_class_id_ != detected_class_id) {
      pending_class_id_ = detected_class_id;
      pending_class_count_ = 1;
      return locked_class_id_;
    }

    pending_class_count_++;
    if (pending_class_count_ >= std::max(1, yolo_class_switch_confirm_frames_)) {
      locked_class_id_ = pending_class_id_;
      pending_class_id_ = -1;
      pending_class_count_ = 0;
    }
    return locked_class_id_;
  }

  std::pair<double, double> laserCompDeg(double distance_m) const {
    if (!laser_comp_enabled_ || distance_m <= 0.1) {
      return {0.0, 0.0};
    }
    const double yaw_comp = std::atan2(laser_offset_right_m_, distance_m) * 180.0 / M_PI;
    const double pitch_comp = -std::atan2(laser_offset_up_m_, distance_m) * 180.0 / M_PI;
    return {yaw_comp, pitch_comp};
  }

  double rectIou(const cv::Rect &a, const cv::Rect &b) const {
    const cv::Rect inter = a & b;
    if (inter.width <= 0 || inter.height <= 0) {
      return 0.0;
    }
    const double inter_area = static_cast<double>(inter.area());
    const double union_area = static_cast<double>(a.area() + b.area()) - inter_area;
    if (union_area <= 1e-6) {
      return 0.0;
    }
    return inter_area / union_area;
  }

  bool selectBestDetection(const std::vector<Detection> &dets, const cv::Mat &bgr, Detection &best) const {
    if (dets.empty()) {
      return false;
    }

    const double frame_area = static_cast<double>(bgr.cols) * static_cast<double>(bgr.rows);
    if (frame_area <= 1.0) {
      return false;
    }

    double aim_u = 0.5 * static_cast<double>(bgr.cols);
    double aim_v = 0.5 * static_cast<double>(bgr.rows);
    if (has_motion_state_) {
      const auto p = predictedCenter();
      aim_u = p.first;
      aim_v = p.second;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &d : dets) {
      const double area = static_cast<double>(d.box.area());
      if (area < yolo_min_area_px_) {
        continue;
      }
      if ((area / frame_area) > yolo_max_area_ratio_) {
        continue;
      }

      if (yolo_class_color_gate_enabled_) {
        const std::string enemy = currentEnemyColor();
        if (enemy == "red" && yolo_red_class_id_ >= 0 && d.class_id != yolo_red_class_id_) {
          continue;
        }
        if (enemy == "blue" && yolo_blue_class_id_ >= 0 && d.class_id != yolo_blue_class_id_) {
          continue;
        }
      }

      if (!passColorGate(bgr, d.box)) {
        continue;
      }

      const double dist_m = estimateDistanceM(d.box);
      if (distance_gate_enabled_ && (dist_m < min_distance_m_ || dist_m > max_distance_m_)) {
        continue;
      }
      double distance_n = 0.5;
      if (dist_m > 0.0 && preferred_distance_sigma_m_ > 1e-3) {
        const double z = (dist_m - preferred_distance_m_) / preferred_distance_sigma_m_;
        distance_n = std::exp(-0.5 * z * z);
      }

      const double conf_n = std::clamp(static_cast<double>(d.score), 0.0, 1.0);

      double iou_n = 0.0;
      double size_n = 0.5;
      if (has_last_target_ && roi_.width > 1 && roi_.height > 1) {
        iou_n = rectIou(d.box, roi_);
        const double ref_area = static_cast<double>(roi_.area());
        if (ref_area > 1.0) {
          const double ratio = area / ref_area;
          const double log_ratio = std::abs(std::log(std::max(1e-6, ratio)));
          size_n = std::clamp(1.0 - log_ratio, 0.0, 1.0);
        }
      }

      const double u = static_cast<double>(d.box.x) + 0.5 * static_cast<double>(d.box.width);
      const double v = static_cast<double>(d.box.y) + 0.5 * static_cast<double>(d.box.height);
      const double du = u - aim_u;
      const double dv = v - aim_v;
      const double dist_px = std::hypot(du, dv);
      const double diag = std::hypot(static_cast<double>(bgr.cols), static_cast<double>(bgr.rows));
      const double center_n = std::clamp(1.0 - dist_px / std::max(1.0, diag), 0.0, 1.0);

      const double score =
        yolo_w_conf_ * conf_n +
        yolo_w_iou_ * iou_n +
        yolo_w_center_ * center_n +
        yolo_w_size_ * size_n +
        yolo_w_distance_ * distance_n;

      if (score > best_score) {
        best_score = score;
        best = d;
        found = true;
      }
    }

    return found;
  }

  bool detectWithYolo(const cv::Mat &bgr, Detection &best) {
    if (!yolo_ready_) {
      return false;
    }

    if ((detector_backend_ == "tensorrt" || detector_backend_ == "trt") && trt_handle_ != nullptr) {
      std::vector<float> raw(static_cast<size_t>(std::max(1, yolo_max_det_)) * 6U, 0.0F);
      const int count = fyt_trt_infer(
        trt_handle_,
        bgr.data,
        bgr.cols,
        bgr.rows,
        raw.data(),
        std::max(1, yolo_max_det_));
      if (count <= 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TRT infer raw_count=0");
        return false;
      }
      if (yolo_max_targets_for_valid_ > 0 && count > yolo_max_targets_for_valid_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TRT dropped by max_targets_for_valid: raw_count=%d, limit=%d",
          count, yolo_max_targets_for_valid_);
        return false;
      }

      std::vector<Detection> dets;
      dets.reserve(static_cast<size_t>(count));
      int pass_conf_and_class = 0;
      for (int i = 0; i < count; ++i) {
        const float *p = &raw[static_cast<size_t>(i) * 6U];
        const int x1 = static_cast<int>(std::round(p[0]));
        const int y1 = static_cast<int>(std::round(p[1]));
        const int x2 = static_cast<int>(std::round(p[2]));
        const int y2 = static_cast<int>(std::round(p[3]));
        const int w = x2 - x1;
        const int h = y2 - y1;
        if (w <= 1 || h <= 1) {
          continue;
        }

        cv::Rect box(x1, y1, w, h);
        box &= cv::Rect(0, 0, bgr.cols, bgr.rows);
        if (box.width <= 1 || box.height <= 1) {
          continue;
        }

        const float conf = p[4];
        if (conf < static_cast<float>(yolo_conf_thres_)) {
          continue;
        }

        const int cls = static_cast<int>(std::round(p[5]));
        if (!yolo_allowed_class_ids_.empty() &&
            std::find(yolo_allowed_class_ids_.begin(), yolo_allowed_class_ids_.end(), cls) ==
              yolo_allowed_class_ids_.end()) {
          continue;
        }
        pass_conf_and_class++;
        dets.push_back(Detection{box, cls, conf});
      }

      if (dets.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TRT filtered to zero: raw_count=%d pass_conf_class=%d conf_thres=%.3f",
          count, pass_conf_and_class, yolo_conf_thres_);
        return false;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TRT counts: raw=%d after_conf_class=%zu",
        count, dets.size());
      return selectBestDetection(dets, bgr, best);
    }

    if (yolo_net_.empty()) {
      return false;
    }

    cv::Mat out;
    try {
      const cv::Mat blob = cv::dnn::blobFromImage(
        bgr,
        1.0 / 255.0,
        cv::Size(yolo_input_w_, yolo_input_h_),
        cv::Scalar(),
        true,
        false);

      yolo_net_.setInput(blob);
      out = yolo_net_.forward();
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "YOLO forward failed (OpenCV %s). If this keeps happening, re-export ONNX with opset=11 simplify=False. err=%s",
        CV_VERSION, e.what());
      return false;
    }
    if (out.empty()) {
      return false;
    }

    std::vector<Detection> dets;
    decodeYoloOutput(out, bgr.size(), dets);
    if (dets.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "OpenCV decode produced 0 detections");
      return false;
    }
    if (yolo_max_targets_for_valid_ > 0 && static_cast<int>(dets.size()) > yolo_max_targets_for_valid_) {
      return false;
    }

    if (!yolo_allowed_class_ids_.empty()) {
      std::vector<Detection> filtered;
      filtered.reserve(dets.size());
      for (const auto &d : dets) {
        if (std::find(yolo_allowed_class_ids_.begin(), yolo_allowed_class_ids_.end(), d.class_id) !=
            yolo_allowed_class_ids_.end()) {
          filtered.push_back(d);
        }
      }
      dets.swap(filtered);
      if (dets.empty()) {
        return false;
      }
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(dets.size());
    scores.reserve(dets.size());
    for (const auto &d : dets) {
      boxes.push_back(d.box);
      scores.push_back(d.score);
    }

    std::vector<int> keep;
    try {
      cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(yolo_conf_thres_),
                        static_cast<float>(yolo_nms_thres_), keep, 1.0F, yolo_max_det_);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "YOLO NMS failed (OpenCV %s): %s", CV_VERSION, e.what());
      return false;
    }
    if (keep.empty()) {
      return false;
    }

    std::vector<Detection> kept_dets;
    kept_dets.reserve(keep.size());
    for (int idx : keep) {
      kept_dets.push_back(dets[idx]);
    }

    return selectBestDetection(kept_dets, bgr, best);
  }

  void publishSerialPacket(const interfaces::msg::GimbalCmd &cmd) {
    if (!serial_bridge_enabled_ || !serial_tx_pub_) {
      return;
    }

    std_msgs::msg::UInt8MultiArray packet;
    auto crc8 = [](const uint8_t *data, size_t n) -> uint8_t {
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
    };

    if (serial_bridge_protocol_ == "legacy19") {
      std::array<uint8_t, 19> frame{};
      frame[0] = 0xA5;
      frame[1] = 0x5A;
      frame[2] = 0x01;

      uint8_t flags = 0;
      if (cmd.laser_power_on) {
        flags |= 0x01;
      }
      if (cmd.laser_hit_valid) {
        flags |= 0x02;
      }
      frame[3] = flags;

      const float yaw = static_cast<float>(cmd.yaw);
      const float pitch = static_cast<float>(cmd.pitch);
      const float distance = static_cast<float>(cmd.distance);
      std::memcpy(&frame[4], &yaw, sizeof(float));
      std::memcpy(&frame[8], &pitch, sizeof(float));
      std::memcpy(&frame[12], &distance, sizeof(float));

      uint8_t checksum = 0;
      for (int i = 0; i < 16; ++i) {
        checksum ^= frame[static_cast<size_t>(i)];
      }
      frame[16] = checksum;
      frame[17] = 0x0D;
      frame[18] = 0x0A;
      packet.data.assign(frame.begin(), frame.end());
    } else {
      std::array<uint8_t, 24> frame{};
      frame[0] = 0xA5;
      frame[1] = 0x01;   // protocol version
      frame[2] = 0x12;   // payload length (bytes from seq to flags)
      frame[3] = serial_tx_seq_++;

      const float yaw = static_cast<float>(cmd.yaw);
      const float pitch = static_cast<float>(cmd.pitch);
      const float yaw_rate = static_cast<float>(cmd.yaw_diff);
      const float pitch_rate = static_cast<float>(cmd.pitch_diff);

      std::memcpy(&frame[4], &yaw, sizeof(float));
      std::memcpy(&frame[8], &pitch, sizeof(float));
      std::memcpy(&frame[12], &yaw_rate, sizeof(float));
      std::memcpy(&frame[16], &pitch_rate, sizeof(float));

      uint8_t flags = 0;
      if (cmd.laser_power_on) {
        flags |= 0x01;
      }
      if (cmd.laser_hit_valid) {
        flags |= 0x02;
      }
      if (cmd.lock_active) {
        flags |= 0x04;
      }
      if (cmd.fire_advice) {
        flags |= 0x08;
      }
      frame[20] = flags;
      frame[21] = 0x00;
      frame[22] = crc8(frame.data(), 22);
      frame[23] = 0x5A;
      packet.data.assign(frame.begin(), frame.end());
    }

    serial_tx_pub_->publish(packet);
  }

  void decodeYoloOutput(const cv::Mat &out,
                        const cv::Size &image_size,
                        std::vector<Detection> &dets) const {
    cv::Mat det = out;
    if (det.dims == 3) {
      const int d0 = det.size[0];
      const int d1 = det.size[1];
      const int d2 = det.size[2];
      if (d0 == 1) {
        if (d2 > d1) {
          cv::Mat m(d1, d2, CV_32F, const_cast<float *>(det.ptr<float>()));
          cv::transpose(m, det);
        } else {
          det = cv::Mat(d1, d2, CV_32F, const_cast<float *>(det.ptr<float>()));
        }
      } else {
        return;
      }
    }

    if (det.dims != 2 || det.cols < 5) {
      return;
    }

    const float sx = static_cast<float>(image_size.width) / static_cast<float>(yolo_input_w_);
    const float sy = static_cast<float>(image_size.height) / static_cast<float>(yolo_input_h_);

    for (int i = 0; i < det.rows; ++i) {
      const float *p = det.ptr<float>(i);
      const float cx = p[0];
      const float cy = p[1];
      const float w = p[2];
      const float h = p[3];
      if (w <= 1.0F || h <= 1.0F) {
        continue;
      }

      int best_cls = -1;
      float best_cls_score = 0.0F;
      float obj = 1.0F;
      int cls_begin = 5;

      std::string layout = yolo_layout_;
      std::transform(layout.begin(), layout.end(), layout.begin(),
                     [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

      if (layout == "v8") {
        cls_begin = 4;
        obj = 1.0F;
      } else if (layout == "v5") {
        cls_begin = 5;
        obj = det.cols >= 6 ? p[4] : 1.0F;
      } else {
        // auto: single-class v8 output is usually [cx, cy, w, h, cls0]
        if (det.cols == 5) {
          cls_begin = 4;
          obj = 1.0F;
        } else {
          cls_begin = 5;
          obj = p[4];
        }
      }

      if (cls_begin >= det.cols) {
        continue;
      }

      for (int c = cls_begin; c < det.cols; ++c) {
        if (p[c] > best_cls_score) {
          best_cls_score = p[c];
          best_cls = c - cls_begin;
        }
      }
      if (best_cls < 0) {
        continue;
      }

      const float conf = obj * best_cls_score;
      if (conf < static_cast<float>(yolo_conf_thres_)) {
        continue;
      }
      const int effective_target_class =
        yolo_single_target_mode_ ? (yolo_target_class_id_ >= 0 ? yolo_target_class_id_ : 0) : yolo_target_class_id_;
      if (effective_target_class >= 0 && best_cls != effective_target_class) {
        continue;
      }

      const int x = static_cast<int>((cx - 0.5F * w) * sx);
      const int y = static_cast<int>((cy - 0.5F * h) * sy);
      const int ww = static_cast<int>(w * sx);
      const int hh = static_cast<int>(h * sy);
      cv::Rect box(x, y, ww, hh);
      box &= cv::Rect(0, 0, image_size.width, image_size.height);
      if (box.width <= 1 || box.height <= 1) {
        continue;
      }

      dets.push_back(Detection{box, best_cls, conf});
    }
  }

  void publishIdle(const rclcpp::Time &stamp) {
    if (use_lt_controller_ && lt_controller_) {
      auto cmd = buildLtCmd(stamp, false, cx_, cy_);
      cmd_pub_->publish(cmd);
      publishSerialPacket(cmd);

      interfaces::msg::DebugLaser dbg;
      dbg.header.stamp = stamp;
      dbg.laser_power_on = true;
      dbg.laser_hit_valid = false;
      dbg.target_tracking = false;
      dbg.aimed_progress = 0.0;
      dbg.lock_count = 0;
      dbg.lock_active = false;
      dbg.lock_remaining = 0.0;
      dbg.progress_stage = 1;
      dbg.target_distance = -1.0;
      dbg.target_id = "lt_idle";
      dbg_pub_->publish(dbg);
      return;
    }

    interfaces::msg::GimbalCmd cmd;
    cmd.header.stamp = stamp;
    cmd.yaw = yaw_deg_;
    cmd.pitch = pitch_deg_;
    cmd.yaw_diff = 0.0;
    cmd.pitch_diff = 0.0;
    cmd.distance = -1.0;
    cmd.laser_power_on = true;
    cmd.laser_advice = true;
    cmd.fire_advice = true;
    const auto s = rule_.update((stamp - last_time_).seconds(), false, true);
    cmd.laser_hit_valid = false;
    cmd.aimed_progress = s.progress;
    cmd.lock_count = static_cast<uint8_t>(s.lock_count);
    cmd.lock_active = s.lock_active;
    cmd.lock_remaining = s.lock_remaining;
    cmd.progress_stage = static_cast<uint8_t>(s.stage);
    cmd_pub_->publish(cmd);
    publishSerialPacket(cmd);

    interfaces::msg::DebugLaser dbg;
    dbg.header.stamp = stamp;
    dbg.laser_power_on = true;
    dbg.laser_hit_valid = false;
    dbg.target_tracking = false;
    dbg.aimed_progress = s.progress;
    dbg.lock_count = static_cast<uint8_t>(s.lock_count);
    dbg.lock_active = s.lock_active;
    dbg.lock_remaining = s.lock_remaining;
    dbg.progress_stage = static_cast<uint8_t>(s.stage);
    dbg.target_distance = -1.0;
    dbg.target_id = "manual_roi";
    dbg_pub_->publish(dbg);

    has_motion_state_ = false;
    line_center_initialized_ = false;
  }

  void updateMotionState(double u, double v, double dt_s) {
    dt_s = std::clamp(dt_s, 0.001, 0.2);

    if (line_motion_enabled_) {
      if (!line_center_initialized_) {
        line_center_u_ = u;
        line_center_v_ = v;
        line_center_initialized_ = true;
      } else {
        line_center_u_ = (1.0 - line_center_alpha_) * line_center_u_ + line_center_alpha_ * u;
        line_center_v_ = (1.0 - line_center_alpha_) * line_center_v_ + line_center_alpha_ * v;
      }

      // 目标沿单轴移动时，抑制垂直于该轴的抖动。
      if (line_axis_ == "x" || line_axis_ == "X") {
        v = line_center_v_;
      } else if (line_axis_ == "y" || line_axis_ == "Y") {
        u = line_center_u_;
      }
    }

    if (!has_motion_state_) {
      center_u_ = u;
      center_v_ = v;
      vel_u_ = 0.0;
      vel_v_ = 0.0;
      has_motion_state_ = true;
      return;
    }

    const double prev_u = center_u_;
    const double prev_v = center_v_;
    center_u_ = (1.0 - center_smooth_alpha_) * center_u_ + center_smooth_alpha_ * u;
    center_v_ = (1.0 - center_smooth_alpha_) * center_v_ + center_smooth_alpha_ * v;

    const double raw_vu = (center_u_ - prev_u) / dt_s;
    const double raw_vv = (center_v_ - prev_v) / dt_s;

    const double dv_jump = std::hypot(raw_vu - vel_u_, raw_vv - vel_v_);
    if (dv_jump > vel_jump_thres_px_s_) {
      prediction_confidence_ = std::max(min_prediction_confidence_, prediction_confidence_ - confidence_decay_);
    } else {
      prediction_confidence_ = std::min(1.0, prediction_confidence_ + confidence_recover_);
    }

    vel_u_ = 0.65 * vel_u_ + 0.35 * raw_vu;
    vel_v_ = 0.65 * vel_v_ + 0.35 * raw_vv;
  }

  std::pair<double, double> predictedCenter() const {
    if (!prediction_enabled_ || !has_motion_state_) {
      return {center_u_, center_v_};
    }

    const double eff_lead_s = lead_time_s_ * prediction_confidence_;
    double du = vel_u_ * eff_lead_s;
    double dv = vel_v_ * eff_lead_s;
    const double mag = std::hypot(du, dv);
    if (mag > max_lead_px_ && mag > 1e-6) {
      const double s = max_lead_px_ / mag;
      du *= s;
      dv *= s;
    }

    if (std::abs(du) < deadzone_px_) {
      du = 0.0;
    }
    if (std::abs(dv) < deadzone_px_) {
      dv = 0.0;
    }

    return {center_u_ + du, center_v_ + dv};
  }

  bool tryVisualHold(const cv::Mat &bgr, const rclcpp::Time &stamp, double &aim_u, double &aim_v) {
    if (!lost_hold_enabled_ || !has_last_target_) {
      return false;
    }
    if ((stamp - last_target_stamp_).seconds() > lost_hold_s_) {
      return false;
    }
    if (roi_.width <= 1 || roi_.height <= 1) {
      return false;
    }

    cv::Rect held = roi_;
    if (has_motion_state_) {
      const auto pred = predictedCenter();
      const double cur_u = static_cast<double>(held.x) + 0.5 * static_cast<double>(held.width);
      const double cur_v = static_cast<double>(held.y) + 0.5 * static_cast<double>(held.height);
      held.x += static_cast<int>(std::lround(pred.first - cur_u));
      held.y += static_cast<int>(std::lround(pred.second - cur_v));
    }

    held &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (held.width <= 1 || held.height <= 1) {
      return false;
    }
    roi_ = held;

    if (has_motion_state_) {
      const auto pred = predictedCenter();
      aim_u = pred.first;
      aim_v = pred.second;
    } else {
      aim_u = static_cast<double>(roi_.x) + 0.5 * static_cast<double>(roi_.width);
      aim_v = static_cast<double>(roi_.y) + 0.5 * static_cast<double>(roi_.height);
    }
    return true;
  }

  bool publishPredictedHold(const rclcpp::Time &stamp) {
    if (use_lt_controller_ && lt_controller_) {
      const auto cmd = buildLtCmd(stamp, false, center_u_, center_v_);
      cmd_pub_->publish(cmd);
      publishSerialPacket(cmd);
      return true;
    }

    if (!lost_hold_enabled_ || !has_last_target_ || !has_motion_state_) {
      return false;
    }
    if ((stamp - last_target_stamp_).seconds() > lost_hold_s_) {
      return false;
    }

    const auto [aim_u, aim_v] = predictedCenter();
    const double yaw_err_deg = std::atan2(aim_u - cx_, fx_) * 180.0 / M_PI;
    const double pitch_err_deg = std::atan2(aim_v - cy_, fy_) * 180.0 / M_PI;
    const auto [yaw_comp_deg, pitch_comp_deg] = laserCompDeg(last_distance_m_);

    const double yaw_diff = yaw_sign_ * yaw_gain_ * yaw_err_deg + yaw_comp_deg;
    const double pitch_diff = pitch_sign_ * pitch_gain_ * pitch_err_deg + pitch_comp_deg;
    const auto s = rule_.update((stamp - last_time_).seconds(), false, true);

    interfaces::msg::GimbalCmd cmd;
    cmd.header.stamp = stamp;
    cmd.yaw = yaw_deg_ + yaw_diff;
    cmd.pitch = pitch_deg_ + pitch_diff;
    cmd.yaw_diff = yaw_diff;
    cmd.pitch_diff = pitch_diff;
    cmd.distance = last_distance_m_ > 0.0 ? last_distance_m_ : -1.0;
    cmd.laser_power_on = true;
    cmd.laser_advice = true;
    cmd.fire_advice = true;
    cmd.laser_hit_valid = false;
    cmd.aimed_progress = s.progress;
    cmd.lock_count = static_cast<uint8_t>(s.lock_count);
    cmd.lock_active = s.lock_active;
    cmd.lock_remaining = s.lock_remaining;
    cmd.progress_stage = static_cast<uint8_t>(s.stage);
    cmd_pub_->publish(cmd);
    publishSerialPacket(cmd);

    interfaces::msg::DebugLaser dbg;
    dbg.header.stamp = stamp;
    dbg.laser_power_on = true;
    dbg.laser_hit_valid = false;
    dbg.target_tracking = true;
    dbg.aimed_progress = s.progress;
    dbg.lock_count = static_cast<uint8_t>(s.lock_count);
    dbg.lock_active = s.lock_active;
    dbg.lock_remaining = s.lock_remaining;
    dbg.progress_stage = static_cast<uint8_t>(s.stage);
    dbg.target_distance = cmd.distance;
    dbg.target_id = "predict_hold";
    dbg_pub_->publish(dbg);

    stable_hit_time_s_ = 0.0;
    return true;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const rclcpp::Time stamp(msg->header.stamp);

    if (!has_cam_info_) {
      return;
    }

    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (frame.empty()) {
      return;
    }

    cv::Mat bgr;
    if (frame.channels() == 3) {
      if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
      } else {
        bgr = frame;
      }
    } else {
      cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    }

    if (rotate_image_180_) {
      cv::rotate(bgr, bgr, cv::ROTATE_180);
    }

    auto renderWindow = [&](bool draw_overlay, double aim_u, double aim_v) {
      if (!show_window_) {
        return;
      }

      if (draw_overlay && roi_.width > 1 && roi_.height > 1) {
        cv::Scalar box_color(0, 255, 0);
        if (last_det_class_id_ == yolo_red_class_id_) {
          box_color = cv::Scalar(0, 0, 255);
        } else if (last_det_class_id_ == yolo_blue_class_id_) {
          box_color = cv::Scalar(255, 0, 0);
        }

        cv::rectangle(bgr, roi_, box_color, 2);
        if (last_det_class_id_ >= 0) {
          const std::string label =
            "cls=" + std::to_string(last_det_class_id_) + " conf=" + std::to_string(last_det_conf_).substr(0, 4);
          cv::putText(
            bgr,
            label,
            cv::Point(roi_.x, std::max(0, roi_.y - 8)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            box_color,
            2);
        }
        cv::drawMarker(
          bgr,
          cv::Point(static_cast<int>(aim_u), static_cast<int>(aim_v)),
          cv::Scalar(0, 0, 255),
          cv::MARKER_TILTED_CROSS,
          18,
          2);
      }

      cv::drawMarker(bgr, cv::Point(bgr.cols / 2, bgr.rows / 2), cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 24, 2);
      cv::imshow("simple_tracker", bgr);
      const int key = cv::waitKey(1);
      if (key == 'r' || key == 'R') {
        tracker_initialized_ = false;
      } else if (key == 'b' || key == 'B') {
        enemy_color_manual_ = "blue";
      } else if (key == 'e' || key == 'E') {
        enemy_color_manual_ = "red";
      } else if (key == 'a' || key == 'A') {
        enemy_color_manual_ = "auto";
      }
    };

    if (detector_mode_ == "yolo") {
      Detection det;
      if (!detectWithYolo(bgr, det)) {
        double hold_u = 0.0;
        double hold_v = 0.0;
        const bool visual_hold = tryVisualHold(bgr, stamp, hold_u, hold_v);
        if (!visual_hold) {
          last_det_class_id_ = -1;
          last_det_conf_ = 0.0F;
        } else {
          last_det_conf_ = std::max(0.0F, last_det_conf_ * 0.92F);
        }
        renderWindow(visual_hold, hold_u, hold_v);
        if (!publishPredictedHold(stamp)) {
          publishIdle(stamp);
        }
        last_time_ = stamp;
        frame_count_++;
        return;
      }
      roi_ = det.box;
      last_det_class_id_ = stabilizeClassId(det.class_id);
      last_det_conf_ = det.score;
      tracker_initialized_ = false;
    } else if (detector_mode_ == "hybrid") {
      bool got_target = false;

      if (!tracker_initialized_ || (frame_count_ % std::max(1, yolo_detect_interval_) == 0)) {
        Detection det;
        if (detectWithYolo(bgr, det)) {
          roi_ = det.box;
          last_det_class_id_ = stabilizeClassId(det.class_id);
          last_det_conf_ = det.score;
          tracker_ = createTracker();
          tracker_->init(bgr, roi_);
          tracker_initialized_ = true;
          got_target = true;
        }
      }

      if (!got_target) {
        if (!tracker_initialized_ || !tracker_) {
          last_det_class_id_ = -1;
          last_det_conf_ = 0.0F;
          renderWindow(false, 0.0, 0.0);
          if (!publishPredictedHold(stamp)) {
            publishIdle(stamp);
          }
          last_time_ = stamp;
          frame_count_++;
          return;
        }

        cv::Rect tracked = roi_;
        const bool ok = tracker_->update(bgr, tracked);
        if (!ok || tracked.width <= 1 || tracked.height <= 1) {
          tracker_initialized_ = false;
          last_det_class_id_ = -1;
          last_det_conf_ = 0.0F;
          renderWindow(false, 0.0, 0.0);
          if (!publishPredictedHold(stamp)) {
            publishIdle(stamp);
          }
          last_time_ = stamp;
          frame_count_++;
          return;
        }
        tracked &= cv::Rect(0, 0, bgr.cols, bgr.rows);
        roi_ = tracked;
      }
    } else {
      if (!tracker_initialized_) {
        if (!use_gui_) {
          renderWindow(false, 0.0, 0.0);
          if (!publishPredictedHold(stamp)) {
            publishIdle(stamp);
          }
          last_time_ = stamp;
          frame_count_++;
          return;
        }
        const cv::Rect sel = cv::selectROI("simple_tracker", bgr, false, false);
        if (sel.width <= 1 || sel.height <= 1) {
          renderWindow(false, 0.0, 0.0);
          if (!publishPredictedHold(stamp)) {
            publishIdle(stamp);
          }
          last_time_ = stamp;
          frame_count_++;
          return;
        }
        roi_ = sel;
        tracker_ = createTracker();
        tracker_->init(bgr, roi_);
        tracker_initialized_ = true;
      }

      cv::Rect tracked = roi_;
      const bool ok = tracker_->update(bgr, tracked);
      if (!ok || tracked.width <= 1 || tracked.height <= 1) {
        tracker_initialized_ = false;
        last_det_class_id_ = -1;
        last_det_conf_ = 0.0F;
        if (!auto_reselect_) {
          renderWindow(false, 0.0, 0.0);
          if (!publishPredictedHold(stamp)) {
            publishIdle(stamp);
          }
          last_time_ = stamp;
          frame_count_++;
          return;
        }
        renderWindow(false, 0.0, 0.0);
        if (!publishPredictedHold(stamp)) {
          publishIdle(stamp);
        }
        last_time_ = stamp;
        frame_count_++;
        return;
      }

      tracked &= cv::Rect(0, 0, bgr.cols, bgr.rows);
      roi_ = tracked;
    }

    // 瞄准点采用框中心：即(top+bottom)/2与(left+right)/2。
    const double u = static_cast<double>(roi_.x) + static_cast<double>(roi_.width) * 0.5;
    const double v = static_cast<double>(roi_.y) + static_cast<double>(roi_.height) * 0.5;

    const double dt = std::clamp((stamp - last_time_).seconds(), 0.001, 0.2);
    updateMotionState(u, v, dt);
    const auto [aim_u, aim_v] = predictedCenter();
    has_last_target_ = true;
    last_target_stamp_ = stamp;

    const double estimated_distance_m = estimateDistanceM(roi_);
    double cmd_yaw = yaw_deg_;
    double cmd_pitch = pitch_deg_;
    double yaw_diff = 0.0;
    double pitch_diff = 0.0;

    if (use_lt_controller_ && lt_controller_) {
      auto lt_cmd = buildLtCmd(stamp, true, aim_u, aim_v);
      cmd_yaw = lt_cmd.yaw;
      cmd_pitch = lt_cmd.pitch;
      yaw_diff = lt_cmd.yaw_diff;
      pitch_diff = lt_cmd.pitch_diff;
    } else {
      const double yaw_err_deg = std::atan2(aim_u - cx_, fx_) * 180.0 / M_PI;
      const double pitch_err_deg = std::atan2(aim_v - cy_, fy_) * 180.0 / M_PI;
      const auto [yaw_comp_deg, pitch_comp_deg] = laserCompDeg(estimated_distance_m);
      yaw_diff = yaw_sign_ * yaw_gain_ * yaw_err_deg + yaw_comp_deg;
      pitch_diff = pitch_sign_ * pitch_gain_ * pitch_err_deg + pitch_comp_deg;
      cmd_yaw = yaw_deg_ + yaw_diff;
      cmd_pitch = pitch_deg_ + pitch_diff;
    }

    const bool color_ok = passColorGate(bgr, roi_);
    const bool distance_ok =
      !distance_gate_enabled_ ||
      (estimated_distance_m >= min_distance_m_ && estimated_distance_m <= max_distance_m_);
    const bool raw_hit_valid = inHitWindow(roi_, bgr.cols, bgr.rows) && color_ok && distance_ok;
    stable_hit_time_s_ = raw_hit_valid ? (stable_hit_time_s_ + dt) : 0.0;
    const bool hit_valid = raw_hit_valid && stable_hit_time_s_ >= score_stability_window_s_;
    const auto s = rule_.update((stamp - last_time_).seconds(), hit_valid, true);

    if (estimated_distance_m > 0.0) {
      last_distance_m_ = estimated_distance_m;
    }

    interfaces::msg::GimbalCmd cmd;
    cmd.header.stamp = msg->header.stamp;
    cmd.yaw = cmd_yaw;
    cmd.pitch = cmd_pitch;
    cmd.yaw_diff = yaw_diff;
    cmd.pitch_diff = pitch_diff;
    cmd.distance = estimated_distance_m > 0.0 ? estimated_distance_m : -1.0;
    cmd.laser_power_on = true;
    cmd.laser_advice = true;
    cmd.fire_advice = true;
    cmd.laser_hit_valid = hit_valid;
    cmd.aimed_progress = s.progress;
    cmd.lock_count = static_cast<uint8_t>(s.lock_count);
    cmd.lock_active = s.lock_active;
    cmd.lock_remaining = s.lock_remaining;
    cmd.progress_stage = static_cast<uint8_t>(s.stage);
    cmd_pub_->publish(cmd);
    publishSerialPacket(cmd);

    interfaces::msg::DebugLaser dbg;
    dbg.header.stamp = msg->header.stamp;
    dbg.laser_power_on = true;
    dbg.laser_hit_valid = hit_valid;
    dbg.target_tracking = true;
    dbg.aimed_progress = s.progress;
    dbg.lock_count = static_cast<uint8_t>(s.lock_count);
    dbg.lock_active = s.lock_active;
    dbg.lock_remaining = s.lock_remaining;
    dbg.progress_stage = static_cast<uint8_t>(s.stage);
    dbg.target_distance = cmd.distance;
    dbg.target_id = "manual_roi";
    dbg_pub_->publish(dbg);

    renderWindow(true, aim_u, aim_v);

    frame_count_++;
    last_time_ = stamp;
  }

  bool use_gui_{true};
  std::string tracker_type_{"CSRT"};
  double yaw_gain_{1.0};
  double pitch_gain_{1.0};
  double yaw_sign_{1.0};
  double pitch_sign_{-1.0};
  double hit_rx_{0.08};
  double hit_ry_{0.08};
  bool auto_reselect_{true};
  bool use_serial_feedback_{false};
  bool show_window_{true};

  bool color_filter_enabled_{true};
  std::string self_color_{"red"};
  std::string enemy_color_manual_{"auto"};
  std::string enemy_color_from_mode_{"blue"};
  double min_enemy_color_ratio_{0.02};

  bool distance_gate_enabled_{true};
  double min_distance_m_{10.0};
  double max_distance_m_{25.0};
  double target_height_m_{0.12};
  double preferred_distance_m_{5.0};
  double preferred_distance_sigma_m_{1.0};

  bool laser_comp_enabled_{true};
  double laser_offset_right_m_{0.0};
  double laser_offset_up_m_{0.0};

  std::string detector_mode_{"tracker"};
  std::string detector_backend_{"opencv"};
  std::string yolo_model_path_;
  std::string yolo_engine_path_;
  int yolo_input_w_{640};
  int yolo_input_h_{640};
  double yolo_conf_thres_{0.35};
  double yolo_nms_thres_{0.45};
  std::string yolo_layout_{"auto"};
  int yolo_target_class_id_{-1};
  int yolo_max_det_{20};
  int yolo_max_targets_for_valid_{12};
  std::vector<int> yolo_allowed_class_ids_;
  int yolo_detect_interval_{3};
  bool yolo_single_target_mode_{true};
  bool yolo_class_color_gate_enabled_{true};
  int yolo_red_class_id_{0};
  int yolo_blue_class_id_{1};
  bool yolo_class_lock_enabled_{true};
  int yolo_class_switch_confirm_frames_{8};
  double yolo_min_area_px_{120.0};
  double yolo_max_area_ratio_{0.75};
  double yolo_w_conf_{0.55};
  double yolo_w_iou_{0.20};
  double yolo_w_center_{0.20};
  double yolo_w_size_{0.05};
  double yolo_w_distance_{0.0};
  bool yolo_ready_{false};
  cv::dnn::Net yolo_net_;
  void *trt_handle_{nullptr};

  bool serial_bridge_enabled_{false};
  std::string serial_bridge_topic_{"serial/tx_packet"};
  std::string serial_bridge_protocol_{"dji_c_v1"};
  uint8_t serial_tx_seq_{0};

  bool prediction_enabled_{true};
  double lead_time_s_{0.08};
  double max_lead_px_{80.0};
  double center_smooth_alpha_{0.55};
  double deadzone_px_{1.5};
  double vel_jump_thres_px_s_{250.0};
  double confidence_decay_{0.25};
  double confidence_recover_{0.08};
  double min_prediction_confidence_{0.2};
  double prediction_confidence_{1.0};

  bool lost_hold_enabled_{true};
  double lost_hold_s_{0.2};
  bool has_last_target_{false};
  rclcpp::Time last_target_stamp_;
  double last_distance_m_{-1.0};

  double score_stability_window_s_{0.12};
  double stable_hit_time_s_{0.0};

  bool line_motion_enabled_{true};
  std::string line_axis_{"x"};
  double line_center_alpha_{0.15};
  bool rotate_image_180_{false};

  bool has_motion_state_{false};
  bool line_center_initialized_{false};
  double center_u_{0.0};
  double center_v_{0.0};
  double vel_u_{0.0};
  double vel_v_{0.0};
  double line_center_u_{0.0};
  double line_center_v_{0.0};
  int64_t frame_count_{0};

  std::string control_backend_{"solver"};
  bool use_lt_controller_{false};
  lt_control::ControlConfig lt_cfg_;
  lt_control::Boresight lt_boresight_;
  std::unique_ptr<lt_control::Controller> lt_controller_;
  double yaw_rate_deg_s_{0.0};
  double pitch_rate_deg_s_{0.0};

  double init_yaw_deg_{0.0};
  double init_pitch_deg_{0.0};
  double yaw_deg_{0.0};
  double pitch_deg_{0.0};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool has_cam_info_{false};

  bool tracker_initialized_{false};
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect roi_;
  int last_det_class_id_{-1};
  float last_det_conf_{0.0F};
  int locked_class_id_{-1};
  int pending_class_id_{-1};
  int pending_class_count_{0};
  LaserRule rule_;

  rclcpp::Time last_time_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
  rclcpp::Subscription<interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;

  rclcpp::Publisher<interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<interfaces::msg::DebugLaser>::SharedPtr dbg_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_tx_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
