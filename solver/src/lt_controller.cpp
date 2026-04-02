#include "lt_control/controller.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>

namespace lt_control {

namespace {
double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kMinScanDs = 1e-3;
constexpr double kDeadbandHystPx = 1.0;

std::string toLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}
}  // namespace

Controller::Controller(ControlConfig cfg) : cfg_(std::move(cfg)) {}

GimbalCommand Controller::update(const TargetMeasurement &meas,
                                 const CameraModel &cam,
                                 const Boresight &bs,
                                 const GimbalState &state) {
  GimbalCommand cmd;
  cmd.timestamp = static_cast<uint32_t>(meas.timestamp);
  cmd.pitch = state.pitch;
  cmd.yaw = state.yaw;
  cmd.pitch_rate = 0.0f;
  cmd.yaw_rate = 0.0f;

  const auto steady_now = std::chrono::steady_clock::now();
  double dt_s = cfg_.ctrl_dt_nominal_ms / 1000.0;
  if (has_last_update_steady_) {
    double dt_ms = std::chrono::duration<double, std::milli>(steady_now - last_update_tp_).count();
    dt_ms = clamp(dt_ms, cfg_.ctrl_dt_min_ms, cfg_.ctrl_dt_max_ms);
    dt_s = dt_ms / 1000.0;
  }
  last_update_tp_ = steady_now;
  has_last_update_steady_ = true;

  if (!meas.valid || cam.fx <= 0.0 || cam.fy <= 0.0) {
    in_deadband_ = false;
    if (!lost_active_) {
      lost_active_ = true;
      lost_start_ts_ = meas.timestamp;
      reacq_count_ = 0;
    }

    double base_pitch = has_last_ ? last_pitch_ : state.pitch;
    double base_yaw = has_last_ ? last_yaw_ : state.yaw;
    double pitch_cmd = base_pitch;
    double yaw_cmd = base_yaw;

    const double lost_ms = static_cast<double>(meas.timestamp - lost_start_ts_);
    const bool enter_scan = !cfg_.scan_enable
                              ? false
                              : (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);
    if (enter_scan) {
      if (!scanning_) {
        double offset = 0.0;
        if (has_last_meas_) {
          double du = last_uv_.x - bs.u_l;
          double dv = last_uv_.y - bs.v_l;
          double u = cfg_.yaw_sign * du;
          double v = cfg_.pitch_sign * dv;
          if (std::abs(u) > 1e-6 || std::abs(v) > 1e-6) {
            const std::string pattern = toLower(cfg_.scan_pattern);
            if (pattern == "spiral") {
              double kx = std::max(1e-3, cfg_.scan_k_yaw);
              double ky = std::max(1e-3, cfg_.scan_k_pitch);
              u /= kx;
              v /= ky;
            }
            offset = std::atan2(v, u);
          }
        }
        scan_center_pitch_ = base_pitch;
        scan_center_yaw_ = base_yaw;
        scan_phase_ = 0.0;
        scan_phase_offset_ = offset;
        scan_dir_ = 1;
        scanning_ = true;
      }

      const std::string pattern = toLower(cfg_.scan_pattern);
      if (pattern == "spiral") {
        double theta = scan_phase_;
        const double spacing = std::max(1e-3, cfg_.scan_spacing_deg);
        const double a = spacing / (2.0 * M_PI);
        double r = a * theta;
        const double r_max = std::max(spacing, cfg_.scan_r_max_deg);
        const double kx = std::max(1e-3, cfg_.scan_k_yaw);
        const double ky = std::max(1e-3, cfg_.scan_k_pitch);
        const double psi = theta + scan_phase_offset_;

        if (cfg_.scan_spiral_return) {
          if (scan_dir_ > 0 && r >= r_max) {
            scan_dir_ = -1;
          } else if (scan_dir_ < 0 && theta <= 0.0) {
            scan_dir_ = 1;
            theta = 0.0;
            r = 0.0;
          }
        }

        double dx_dtheta = 0.0;
        double dy_dtheta = 0.0;
        if (r < r_max) {
          dx_dtheta = kx * (a * std::cos(psi) - r * std::sin(psi));
          dy_dtheta = ky * (a * std::sin(psi) + r * std::cos(psi));
        } else {
          r = r_max;
          dx_dtheta = -kx * r * std::sin(psi);
          dy_dtheta = ky * r * std::cos(psi);
        }
        double ds_dtheta = std::sqrt(dx_dtheta * dx_dtheta + dy_dtheta * dy_dtheta);
        ds_dtheta = std::max(ds_dtheta, kMinScanDs);
        const double v = std::max(1e-3, cfg_.scan_speed_deg_s);
        const double step = (v / ds_dtheta) * dt_s;
        if (cfg_.scan_spiral_return) {
          theta += static_cast<double>(scan_dir_) * step;
          if (theta < 0.0) {
            theta = 0.0;
          }
        } else {
          theta += step;
        }

        if (theta > 1000.0 * M_PI) {
          theta = std::fmod(theta, 2.0 * M_PI);
        }
        scan_phase_ = theta;
        r = a * theta;
        const double r_eval = std::min(r, r_max);
        pitch_cmd = scan_center_pitch_ + ky * r_eval * std::sin(theta + scan_phase_offset_);
        yaw_cmd = scan_center_yaw_ + kx * r_eval * std::cos(theta + scan_phase_offset_);
      } else {
        scan_phase_ += 2.0 * M_PI * cfg_.scan_rate_hz * dt_s;
        if (scan_phase_ > 2.0 * M_PI) {
          scan_phase_ = std::fmod(scan_phase_, 2.0 * M_PI);
        }
        pitch_cmd = scan_center_pitch_ +
                    cfg_.scan_radius_deg * std::sin(scan_phase_ + scan_phase_offset_);
        yaw_cmd = scan_center_yaw_ +
                  cfg_.scan_radius_deg * std::cos(scan_phase_ + scan_phase_offset_);
      }
    }

    double max_step = cfg_.max_angle_rate * dt_s;
    pitch_cmd = clamp(pitch_cmd, base_pitch - max_step, base_pitch + max_step);
    yaw_cmd = clamp(yaw_cmd, base_yaw - max_step, base_yaw + max_step);

    cmd.pitch = static_cast<float>(pitch_cmd);
    cmd.yaw = static_cast<float>(yaw_cmd);

    last_pitch_ = pitch_cmd;
    last_yaw_ = yaw_cmd;
    has_last_ = true;
    return cmd;
  }

  if (lost_active_) {
    reacq_count_++;
    const int need = std::max(1, cfg_.scan_reacq_confirm_frames);
    if (reacq_count_ >= need) {
      scanning_ = false;
      lost_active_ = false;
      reacq_count_ = 0;
    }
  }

  double du = meas.uv.x - bs.u_l;
  double dv = meas.uv.y - bs.v_l;
  const double abs_du = std::abs(du);
  const double abs_dv = std::abs(dv);
  const double deadband_enter = std::max(0.0, cfg_.deadband_px);
  const double deadband_exit = deadband_enter + kDeadbandHystPx;

  if (in_deadband_) {
    if (abs_du > deadband_exit || abs_dv > deadband_exit) {
      in_deadband_ = false;
    }
  } else if (abs_du < deadband_enter && abs_dv < deadband_enter) {
    in_deadband_ = true;
  }

  if (in_deadband_) {
    if (has_last_) {
      cmd.pitch = static_cast<float>(last_pitch_);
      cmd.yaw = static_cast<float>(last_yaw_);
    }
    last_uv_ = meas.uv;
    last_meas_ts_ = meas.timestamp;
    has_last_meas_ = true;
    return cmd;
  }

  double dyaw_rad = cfg_.yaw_sign * std::atan(du / cam.fx);
  double dpitch_rad = cfg_.pitch_sign * std::atan(dv / cam.fy);
  double dyaw = dyaw_rad * kRadToDeg;
  double dpitch = dpitch_rad * kRadToDeg;

  double yaw_cmd = state.yaw + cfg_.kp * dyaw;
  double pitch_cmd = state.pitch + cfg_.kp * dpitch;

  if (cfg_.use_damping && cfg_.damping_kd > 0.0) {
    double yaw_rate_fb = 0.0;
    double pitch_rate_fb = 0.0;
    bool has_rate = false;
    const std::string src = toLower(cfg_.damping_source);

    if (src == "gimbal") {
      yaw_rate_fb = state.yaw_rate;
      pitch_rate_fb = state.pitch_rate;
      has_rate = true;
    } else if (has_last_meas_) {
      double dt_ms = static_cast<double>(meas.timestamp - last_meas_ts_);
      if (dt_ms > 0.0 && dt_ms <= cfg_.damping_dt_max_ms) {
        double dt = dt_ms / 1000.0;
        double u_dot = (meas.uv.x - last_uv_.x) / dt;
        double v_dot = (meas.uv.y - last_uv_.y) / dt;
        yaw_rate_fb = cfg_.yaw_sign * (u_dot / cam.fx) * kRadToDeg;
        pitch_rate_fb = cfg_.pitch_sign * (v_dot / cam.fy) * kRadToDeg;
        has_rate = true;
      }
    }

    if (has_rate) {
      yaw_cmd -= cfg_.damping_kd * yaw_rate_fb;
      pitch_cmd -= cfg_.damping_kd * pitch_rate_fb;
    }
  }

  if (cfg_.use_velocity_ff && has_last_meas_) {
    double dt_ms = static_cast<double>(meas.timestamp - last_meas_ts_);
    if (dt_ms > 0.0 && dt_ms <= cfg_.ff_dt_max_ms) {
      double dt = dt_ms / 1000.0;
      double u_dot = (meas.uv.x - last_uv_.x) / dt;
      double v_dot = (meas.uv.y - last_uv_.y) / dt;
      double yaw_rate_raw = cfg_.yaw_sign * (u_dot / cam.fx) * kRadToDeg;
      double pitch_rate_raw = cfg_.pitch_sign * (v_dot / cam.fy) * kRadToDeg;

      double yaw_rate_ff =
        cfg_.ff_alpha * yaw_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_yaw_rate_;
      double pitch_rate_ff =
        cfg_.ff_alpha * pitch_rate_raw + (1.0 - cfg_.ff_alpha) * last_ff_pitch_rate_;

      yaw_rate_ff = clamp(yaw_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);
      pitch_rate_ff = clamp(pitch_rate_ff, -cfg_.ff_rate_max, cfg_.ff_rate_max);

      last_ff_yaw_rate_ = yaw_rate_ff;
      last_ff_pitch_rate_ = pitch_rate_ff;
      cmd.yaw_rate = static_cast<float>(yaw_rate_ff);
      cmd.pitch_rate = static_cast<float>(pitch_rate_ff);
    } else {
      last_ff_yaw_rate_ = 0.0;
      last_ff_pitch_rate_ = 0.0;
    }
  } else {
    last_ff_yaw_rate_ = 0.0;
    last_ff_pitch_rate_ = 0.0;
  }

  if (has_last_) {
    yaw_cmd = cfg_.lowpass_alpha * yaw_cmd + (1.0 - cfg_.lowpass_alpha) * last_yaw_;
    pitch_cmd = cfg_.lowpass_alpha * pitch_cmd + (1.0 - cfg_.lowpass_alpha) * last_pitch_;
  }

  double max_step = cfg_.max_angle_rate * dt_s;
  yaw_cmd = clamp(yaw_cmd, state.yaw - max_step, state.yaw + max_step);
  pitch_cmd = clamp(pitch_cmd, state.pitch - max_step, state.pitch + max_step);

  cmd.pitch = static_cast<float>(pitch_cmd);
  cmd.yaw = static_cast<float>(yaw_cmd);

  last_pitch_ = pitch_cmd;
  last_yaw_ = yaw_cmd;
  has_last_ = true;
  last_uv_ = meas.uv;
  last_meas_ts_ = meas.timestamp;
  has_last_meas_ = true;
  return cmd;
}

}  // namespace lt_control
