#!/usr/bin/env bash
set -euo pipefail

NODE_NAME="/camera_driver"
GAIN_DEFAULT="2.0"

print_usage() {
  cat <<'EOF'
Usage:
  exposure_presets.sh list
  exposure_presets.sh apply <index> [gain]
  exposure_presets.sh sweep <start> <end> <step> [gain] [seconds]
  exposure_presets.sh interactive [index] [gain]

Examples:
  exposure_presets.sh list
  exposure_presets.sh apply 6
  exposure_presets.sh apply 9 1.5
  exposure_presets.sh sweep 120 2400 120 2.0 1.5
  exposure_presets.sh interactive 6 2.0

Notes:
  - This script only writes runtime ROS params to /camera_driver.
  - Keep auto_exposure=0 and auto_gain=0 for stable manual tuning.
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/../config/camera_driver_params.yaml"

PRESETS=(
  80
  120
  160
  220
  300
  420
  600
  850
  1200
  1700
  2400
  3200
  4200
  5600
  7200
)

set_manual_mode() {
  ros2 param set "$NODE_NAME" auto_exposure 0 >/dev/null
  ros2 param set "$NODE_NAME" auto_gain 0 >/dev/null
}

apply_exposure() {
  local exp="$1"
  local gain="$2"
  set_manual_mode
  ros2 param set "$NODE_NAME" exposure_time "$exp" >/dev/null
  ros2 param set "$NODE_NAME" gain "$gain" >/dev/null
  echo "Applied exposure_time=$exp gain=$gain"
}

save_to_config() {
  local exp="$1"
  local gain="$2"
  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Config file not found: $CONFIG_FILE"
    return 1
  fi

  sed -i -E "s/^([[:space:]]*auto_exposure:).*/\1 0/" "$CONFIG_FILE"
  sed -i -E "s/^([[:space:]]*auto_gain:).*/\1 0/" "$CONFIG_FILE"
  sed -i -E "s/^([[:space:]]*exposure_time:).*/\1 ${exp}/" "$CONFIG_FILE"
  sed -i -E "s/^([[:space:]]*gain:).*/\1 ${gain}/" "$CONFIG_FILE"
  echo "Saved to config: exposure_time=$exp gain=$gain"
}

interactive_mode() {
  local idx="$1"
  local gain="$2"

  if (( idx < 1 || idx > ${#PRESETS[@]} )); then
    echo "Index out of range. Run: exposure_presets.sh list"
    exit 1
  fi

  local cur_idx="$idx"
  local cur_exp="${PRESETS[$((cur_idx - 1))]}"
  apply_exposure "$cur_exp" "$gain"

  echo ""
  echo "Interactive tuning mode"
  echo "  n: next preset"
  echo "  p: previous preset"
  echo "  g: gain +0.5"
  echo "  G: gain -0.5"
  echo "  s: save current exposure/gain to config"
  echo "  q: quit"
  echo ""

  while true; do
    echo -n "[idx=$cur_idx exp=$cur_exp gain=$gain] > "
    IFS= read -r -s -n 1 key
    echo "$key"

    case "$key" in
      n)
        if (( cur_idx < ${#PRESETS[@]} )); then
          cur_idx=$((cur_idx + 1))
        fi
        cur_exp="${PRESETS[$((cur_idx - 1))]}"
        apply_exposure "$cur_exp" "$gain"
        ;;
      p)
        if (( cur_idx > 1 )); then
          cur_idx=$((cur_idx - 1))
        fi
        cur_exp="${PRESETS[$((cur_idx - 1))]}"
        apply_exposure "$cur_exp" "$gain"
        ;;
      g)
        gain=$(awk -v g="$gain" 'BEGIN{printf "%.1f", g + 0.5}')
        apply_exposure "$cur_exp" "$gain"
        ;;
      G)
        gain=$(awk -v g="$gain" 'BEGIN{v=g-0.5; if(v<0) v=0; printf "%.1f", v}')
        apply_exposure "$cur_exp" "$gain"
        ;;
      s)
        save_to_config "$cur_exp" "$gain"
        ;;
      q)
        echo "Exit interactive mode"
        break
        ;;
      *)
        echo "Unknown key: $key"
        ;;
    esac
  done
}

if [[ $# -lt 1 ]]; then
  print_usage
  exit 1
fi

cmd="$1"
shift

case "$cmd" in
  list)
    echo "Preset indices and exposure_time values:"
    for i in "${!PRESETS[@]}"; do
      idx=$((i + 1))
      echo "  $idx) ${PRESETS[$i]}"
    done
    ;;

  apply)
    if [[ $# -lt 1 || $# -gt 2 ]]; then
      print_usage
      exit 1
    fi
    idx="$1"
    gain="${2:-$GAIN_DEFAULT}"

    if ! [[ "$idx" =~ ^[0-9]+$ ]]; then
      echo "Index must be a positive integer"
      exit 1
    fi
    if (( idx < 1 || idx > ${#PRESETS[@]} )); then
      echo "Index out of range. Run: exposure_presets.sh list"
      exit 1
    fi

    exp="${PRESETS[$((idx - 1))]}"
    apply_exposure "$exp" "$gain"
    ;;

  sweep)
    if [[ $# -lt 3 || $# -gt 5 ]]; then
      print_usage
      exit 1
    fi
    start="$1"
    end="$2"
    step="$3"
    gain="${4:-$GAIN_DEFAULT}"
    hold_s="${5:-1.2}"

    if ! [[ "$start" =~ ^[0-9]+$ && "$end" =~ ^[0-9]+$ && "$step" =~ ^[0-9]+$ ]]; then
      echo "start/end/step must be integers"
      exit 1
    fi
    if (( step <= 0 )); then
      echo "step must be > 0"
      exit 1
    fi

    echo "Sweeping exposure_time from $start to $end step $step, gain=$gain, hold=${hold_s}s"

    if (( start <= end )); then
      exp="$start"
      while (( exp <= end )); do
        apply_exposure "$exp" "$gain"
        sleep "$hold_s"
        exp=$((exp + step))
      done
    else
      exp="$start"
      while (( exp >= end )); do
        apply_exposure "$exp" "$gain"
        sleep "$hold_s"
        exp=$((exp - step))
      done
    fi
    ;;

  interactive)
    if [[ $# -gt 2 ]]; then
      print_usage
      exit 1
    fi
    idx="${1:-6}"
    gain="${2:-$GAIN_DEFAULT}"
    if ! [[ "$idx" =~ ^[0-9]+$ ]]; then
      echo "Index must be a positive integer"
      exit 1
    fi
    interactive_mode "$idx" "$gain"
    ;;

  *)
    print_usage
    exit 1
    ;;
esac
