#!/usr/bin/env bash
# bag + candump 동시 녹화.
#   record.sh <out_dir> [--ros] [--if can0,can1] [--topics "t1 t2 ..."]
# Ctrl+C 한 번으로 모두 종료하고 meta.yaml 에 시각·CAN 통계를 기록한다.
set -u
OUT=""; ROS=0; IFS_LIST="can0,can1"
TOPICS="/joint_states /left_joint_trajectory_controller/controller_state /right_joint_trajectory_controller/controller_state /tf"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros) ROS=1; shift;;
    --if) IFS_LIST="$2"; shift 2;;
    --topics) TOPICS="$2"; shift 2;;
    -h|--help) sed -n 2,5p "$0"; exit 0;;
    *) OUT="$1"; shift;;
  esac
done
[[ -z "$OUT" ]] && { echo "usage: record.sh <out_dir> [--ros] [--if can0,can1]"; exit 1; }
mkdir -p "$OUT"
META="$OUT/meta.yaml"
PIDS=()

can_stats() {  # name -> yaml line
  local ifc=$1
  ip -s -d link show "$ifc" | awk -v ifc="$ifc" '
    /berr-counter/ {for(i=1;i<=NF;i++){if($i=="tx")btx=$(i+1); if($i=="rx")brx=$(i+1)}; gsub(/[()]/,"",btx); gsub(/[()]/,"",brx)}
    /^[ \t]*re-started/ {getline; bus_err=$2; bus_off=$6}
    /RX:/ {getline; rx_pkts=$2; rx_err=$3; rx_drop=$4}
    /TX:/ {getline; tx_pkts=$2; tx_err=$3; tx_drop=$4}
    END {printf "    %s: {rx_pkts: %s, rx_err: %s, rx_drop: %s, tx_pkts: %s, tx_err: %s, tx_drop: %s, bus_errors: %s, bus_off: %s, berr_tx: %s, berr_rx: %s}\n", ifc, rx_pkts, rx_err, rx_drop, tx_pkts, tx_err, tx_drop, bus_err, bus_off, btx, brx}'
}

{
  echo "record:"
  echo "  start_utc: $(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
  echo "  start_epoch: $(date +%s.%N)"
  echo "  host: $(hostname)"
  echo "  ros_bag: $ROS"
  echo "  can_ifs: [${IFS_LIST}]"
  echo "  can_stats_start:"
  for ifc in ${IFS_LIST//,/ }; do can_stats "$ifc"; done
} > "$META"

# candump: -L = 로그 포맷을 stdout 으로 ((epoch) if id#data), 커널 rx 타임스탬프 사용
for ifc in ${IFS_LIST//,/ }; do
  candump -L "$ifc" > "$OUT/$ifc.log" &
  PIDS+=($!)
done

if [[ $ROS -eq 1 ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash; source "$HOME/openarm_ws/install/setup.bash"
  # shellcheck disable=SC2086
  ros2 bag record -s mcap -o "$OUT/bag" $TOPICS > "$OUT/bag_record.log" 2>&1 &
  PIDS+=($!)
fi

echo "[record] → $OUT  (pids: ${PIDS[*]})  Ctrl+C 로 종료"
cleanup() {
  echo; echo "[record] stopping..."
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null; done
  {
    echo "  end_utc: $(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
    echo "  end_epoch: $(date +%s.%N)"
    echo "  can_stats_end:"
    for ifc in ${IFS_LIST//,/ }; do can_stats "$ifc"; done
    for ifc in ${IFS_LIST//,/ }; do echo "  frames_$ifc: $(wc -l < "$OUT/$ifc.log")"; done
  } >> "$META"
  echo "[record] done. meta → $META"
  exit 0
}
trap cleanup INT TERM
wait
