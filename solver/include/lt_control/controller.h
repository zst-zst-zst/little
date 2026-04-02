#ifndef LT_CONTROL_CONTROLLER_H
#define LT_CONTROL_CONTROLLER_H

#include <chrono>
#include <string>

#include "lt_control/types.h"

namespace lt_control {

struct ControlConfig {
  double kp = 1.0;
  double deadband_px = 2.0;
  double max_angle_rate = 57.2958;
  double lowpass_alpha = 0.3;
  double yaw_sign = -1.0;
  double pitch_sign = 1.0;
  double ctrl_dt_nominal_ms = 10.0;
  double ctrl_dt_min_ms = 1.0;
  double ctrl_dt_max_ms = 100.0;
  bool use_velocity_ff = false;
  double ff_alpha = 0.3;
  double ff_rate_max = 114.592;
  double ff_dt_max_ms = 80.0;
  bool use_damping = false;
  std::string damping_source = "meas";
  double damping_kd = 0.0;
  double damping_dt_max_ms = 120.0;
  bool scan_enable = false;
  double scan_radius_deg = 2.0;
  double scan_rate_hz = 0.2;
  std::string scan_pattern = "circle";
  double scan_spacing_deg = 6.0;
  double scan_speed_deg_s = 12.0;
  double scan_r_max_deg = 20.0;
  bool scan_spiral_return = false;
  double scan_k_yaw = 1.5;
  double scan_k_pitch = 1.0;
  double scan_enter_delay_ms = 0.0;
  int scan_reacq_confirm_frames = 0;
  int startup_check_frames = 2;
  double startup_home_pitch = 0.0;
  double startup_home_yaw = 0.0;
  int startup_prep_ms = 0;
  int startup_hold_ms = 200;
  int startup_home_ms = 600;
  int startup_validate_ms = 200;
  int startup_min_state_frames = 1;
  bool startup_require_home = false;
  double startup_home_tol_deg = 1.0;
  int startup_home_max_extra_ms = 1000;
  bool startup_validate_first = false;
  bool startup_allow_early_exit = false;
};

class Controller {
public:
  explicit Controller(ControlConfig cfg = {});

  GimbalCommand update(const TargetMeasurement &meas,
                       const CameraModel &cam,
                       const Boresight &bs,
                       const GimbalState &state);

private:
  ControlConfig cfg_;
  double last_pitch_ = 0.0;
  double last_yaw_ = 0.0;
  bool has_last_ = false;
  bool has_last_update_steady_ = false;
  std::chrono::steady_clock::time_point last_update_tp_{};
  bool has_last_meas_ = false;
  int64_t last_meas_ts_ = 0;
  cv::Point2f last_uv_{};
  double last_ff_pitch_rate_ = 0.0;
  double last_ff_yaw_rate_ = 0.0;
  bool scanning_ = false;
  double scan_phase_ = 0.0;
  double scan_phase_offset_ = 0.0;
  double scan_center_pitch_ = 0.0;
  double scan_center_yaw_ = 0.0;
  int scan_dir_ = 1;
  bool lost_active_ = false;
  int64_t lost_start_ts_ = 0;
  int reacq_count_ = 0;
  int startup_frames_ = 0;
  bool startup_has_meas_ = false;
  bool startup_prep_started_ = false;
  bool startup_prep_done_ = false;
  std::chrono::steady_clock::time_point startup_prep_tp_{};
  uint32_t last_state_ts_ = 0;
  int startup_state_frames_ = 0;
  bool startup_home_extend_ = false;
  std::chrono::steady_clock::time_point startup_home_extend_tp_{};
  bool in_deadband_ = false;
};

}  // namespace lt_control

#endif  // LT_CONTROL_CONTROLLER_H
