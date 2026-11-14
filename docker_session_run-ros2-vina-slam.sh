#!/bin/bash
# Run VINA-SLAM on a rosbag (ROS 2 bag directory or ROS 1 .bag), then convert
# its output to an HDMapping session.
#
# VINA-SLAM reads the bag itself (no ros2 bag play): the LiDAR and IMU topics of
# its parameter file, as fast as it processes them. With saving on, it writes
#   <output_dir>/vina-slam-results/trajectory.txt   one TUM row per frame
#                                                   (scan-end time, bag time)
#   <output_dir>/vina-slam-results/pcd/NNNNNN.pcd   each frame's points in its
#                                                   own body frame
# and exits when the bag is done. The converter places every frame with the
# pose of its row. VINA-SLAM's clock is left untouched (no use_sim_time).

IMAGE_NAME='vina-slam_humble'
TMUX_SESSION='ros2_vina_slam'

DATASET_CONTAINER_PATH='/ros2_ws/dataset/input.bag'
DATASET_ROS2_PATH='/tmp/dataset_ros2'
BAG_OUTPUT_CONTAINER='/ros2_ws/recordings'
CONFIG_MOUNT_CONTAINER='/ros2_ws/config/vina_slam.yaml'
CONFIG_DIR_CONTAINER='/ros2_ws/install/vina_slam/share/vina_slam/config'

# The folder VINA-SLAM saves into, inside the output directory. VINA-SLAM
# refuses to start if it exists, so each run removes the previous one.
RESULTS_NAME="vina-slam-results"
HDMAPPING_OUT_NAME="output_hdmapping"

# VINA-SLAM parameter file: a file on the host, or the name of one shipped with
# VINA-SLAM in its config/ folder. Default: mid360_bench.yaml, VINA-SLAM's
# mid360.yaml (/livox/lidar CustomMsg + /livox/imu) with the local BA on.
CONFIG_FILE="${CONFIG_FILE:-mid360_bench.yaml}"

# RViz on by default — the live view of how VINA-SLAM tracks the dataset.
USE_RVIZ="${USE_RVIZ:-1}"

# Force Mesa software rendering by default so RViz renders even when the host
# GPU driver is not exposed to the container (the libGL "nvidia-drm" / amdgpu case).
LIBGL_SW="${LIBGL_SW:-1}"
if [[ "$LIBGL_SW" == "1" ]]; then LIBGL_ENV="1"; else LIBGL_ENV=""; fi

usage() {
  echo "Usage:"
  echo "  $0 <ros2_bag_dir | input.bag> <output_dir>"
  echo
  echo "  ros2_bag_dir : ROS 2 bag directory — read directly, no conversion"
  echo "                 (this is what the HDMapping orchestration passes)"
  echo "  input.bag    : ROS 1 bag file — its LiDAR and IMU topics are converted to ROS 2 first"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  CONFIG_FILE - VINA-SLAM parameter file: a host *.yaml, or the name of one in"
  echo "                VINA-SLAM's config/ folder    (default: mid360_bench.yaml)"
  echo "  USE_RVIZ    - 1/0, launch RViz live view    (default: 1)"
  exit 1
}

echo "=== VINA-SLAM rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file (or ROS 2 bag directory)")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input does not exist: $DATASET_HOST_PATH"
  exit 1
fi

# Parameter file: a host file is mounted into the container; a plain name is
# a file shipped with VINA-SLAM (checked inside the container).
CONFIG_MOUNT=()
if [[ -f "$CONFIG_FILE" ]]; then
  CONFIG_FILE=$(realpath "$CONFIG_FILE")
  CONFIG_MOUNT=(-v "$CONFIG_FILE":"$CONFIG_MOUNT_CONTAINER":ro)
  CONFIG_CONTAINER="$CONFIG_MOUNT_CONTAINER"
  CONFIG_DESC="$CONFIG_FILE (host)"
elif [[ "$CONFIG_FILE" != */* ]]; then
  CONFIG_CONTAINER="$CONFIG_DIR_CONTAINER/$CONFIG_FILE"
  CONFIG_DESC="$CONFIG_FILE (VINA-SLAM config/)"
else
  echo "Error: CONFIG_FILE does not exist: $CONFIG_FILE"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

# Orchestration convention: a directory input is an already-converted ROS 2 bag
# (read directly); a file input is a ROS 1 .bag (converted first).
if [[ -d "$DATASET_HOST_PATH" ]]; then
  INPUT_IS_DIR=1
else
  INPUT_IS_DIR=0
fi

echo "Input           : $DATASET_HOST_PATH ($([[ $INPUT_IS_DIR == 1 ]] && echo 'ROS 2 bag directory' || echo 'ROS 1 bag file, converted to ROS 2'))"
echo "Output dir      : $BAG_OUTPUT_HOST"
echo "Parameter file  : $CONFIG_DESC"

xhost +local:docker >/dev/null

# ── Phase 1: run VINA-SLAM (it reads the bag and saves its own output) ───────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -e LIBGL_ALWAYS_SOFTWARE="$LIBGL_ENV" \
  -e USE_RVIZ="$USE_RVIZ" \
  -e CONFIG_CONTAINER="$CONFIG_CONTAINER" \
  -e INPUT_IS_DIR="$INPUT_IS_DIR" \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "${CONFIG_MOUNT[@]}" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash

    INPUT_TOPICS=$(python3 /ros2_ws/scripts/vina_slam_preflight.py --topics "$CONFIG_CONTAINER") || exit 42
    export INPUT_TOPICS

    # ── Convert ROS 1 bag to ROS 2 format if needed (LiDAR and IMU only) ─────
    if [[ "$INPUT_IS_DIR" == "1" ]]; then
      echo "[convert] Input is already a ROS 2 bag directory — skipping conversion."
      ROS2_BAG="'"$DATASET_CONTAINER_PATH"'"
    else
      echo "[convert] Converting $INPUT_TOPICS of the ROS 1 bag to ROS 2 format..."
      rm -rf "'"$DATASET_ROS2_PATH"'"
      INCLUDE_ARGS=""
      for t in $INPUT_TOPICS; do INCLUDE_ARGS="$INCLUDE_ARGS --include-topic $t"; done
      rosbags-convert "'"$DATASET_CONTAINER_PATH"'" --dst "'"$DATASET_ROS2_PATH"'" $INCLUDE_ARGS || {
        echo "[convert] ERROR: rosbags-convert failed"; exit 1; }
      ROS2_BAG="'"$DATASET_ROS2_PATH"'"
    fi

    export ROS2_BAG
    echo "[convert] ROS 2 bag ready at: $ROS2_BAG"
    ls -la $ROS2_BAG/

    # ── Preflight: the LiDAR and IMU topics must be in the bag, otherwise ────
    # VINA-SLAM processes nothing and the run saves no output.
    if ! python3 /ros2_ws/scripts/vina_slam_preflight.py "$CONFIG_CONTAINER" "$ROS2_BAG"; then
      echo "[preflight] input check failed, nothing is run"
      exit 42
    fi

    # Diagnostic logs, next to the results. Exported so the shells of the
    # tmux panes inherit it.
    export LOG_DIR='"$BAG_OUTPUT_CONTAINER"'/logs
    export RESULTS_DIR='"$BAG_OUTPUT_CONTAINER/$RESULTS_NAME"'
    mkdir -p "$LOG_DIR"
    rm -f "$LOG_DIR"/*.log
    echo "[setup] diagnostic logs -> $LOG_DIR"

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: VINA-SLAM (reads the bag, saves, exits) ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
rm -rf "$RESULTS_DIR"
echo "[vina_slam] launching vina_slam_bench.launch.py (config=$CONFIG_CONTAINER, bag=$ROS2_BAG) ..."
stdbuf -oL -eL ros2 launch vina_slam vina_slam_bench.launch.py \
  config_file:=$CONFIG_CONTAINER bag_path:=$ROS2_BAG \
  save_path:='"$BAG_OUTPUT_CONTAINER"'/ bagname:='"$RESULTS_NAME"' 2>&1 | tee "$LOG_DIR"/vina_slam.log
tmux wait-for -S VINA_DONE
echo "[vina_slam] done"
'\'' C-m

    # ---------- PANE 1: RViz ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
if [[ "$USE_RVIZ" == "1" ]]; then
  echo "[rviz] launching RViz2..."
  rviz2 -d /ros2_ws/install/vina_slam/share/vina_slam/rviz_cfg/vina_slam_bench.rviz
else
  echo "[rviz] disabled (USE_RVIZ=0)"
fi
'\'' C-m

    # ---------- PANE 2: diagnostics ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 20
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
(
echo "=== ROS 2 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
ros2 topic list
echo ""
for t in /map_scan /map_path; do
  echo "--- hz $t ---"
  timeout 5 ros2 topic hz $t 2>&1 | tail -2
done
echo ""
echo "--- saved so far ---"
echo "trajectory rows: $(wc -l 2>/dev/null < "$RESULTS_DIR"/trajectory.txt || echo 0)"
echo "frame PCDs     : $(ls "$RESULTS_DIR"/pcd 2>/dev/null | wc -l)"
) 2>&1 | tee "$LOG_DIR"/diag.log
echo ""
echo "[diag] done — you can type ROS 2 commands here, e.g.:"
echo "  ros2 topic hz /map_scan"
'\'' C-m

    # ---------- Control window (window 1) ----------
    # This is the window the user attaches to; the noisy panes are on window 0.
    # It waits for VINA-SLAM to finish the bag and exit, then tears the whole
    # session down.
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[control] waiting for VINA-SLAM to read the whole bag and exit..."
echo "(logs -> $LOG_DIR/{vina_slam,diag}.log)"
tmux wait-for VINA_DONE
echo "[control] VINA-SLAM finished — shutting down"

echo ""
echo "--- vina_slam.log: BagReader and last 5 WARN / ERROR lines ---"
grep -E "BagReader\]|finished cleanly" "$LOG_DIR"/vina_slam.log 2>/dev/null | tail -3
grep -E "\[WARN\]|\[ERROR\]" "$LOG_DIR"/vina_slam.log 2>/dev/null | tail -5
echo ""
echo "trajectory rows: $(wc -l 2>/dev/null < "$RESULTS_DIR"/trajectory.txt || echo 0)"
echo "frame PCDs     : $(ls "$RESULTS_DIR"/pcd 2>/dev/null | wc -l)"
echo ""

# Graceful stop of RViz, then force-kill by process name.
# Pane layout: 0=VINA-SLAM, 1=rviz, 2=diag
echo "[control] stopping RViz..."
tmux send-keys -t '"$TMUX_SESSION"':0.1 C-c
sleep 3
pkill -9 vina_slam 2>/dev/null || true
pkill -9 rviz2     2>/dev/null || true
sleep 1

echo "[control] terminating tmux"
tmux kill-server
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# The preflight check failed: nothing was saved, and converting would pick up
# stale results from an earlier run.
if [[ $? -eq 42 ]]; then
  echo "=== ABORTED: input check failed, no conversion ==="
  exit 1
fi

if [[ ! -s "$BAG_OUTPUT_HOST/$RESULTS_NAME/trajectory.txt" ]]; then
  echo "=== ABORTED: VINA-SLAM saved no trajectory ($BAG_OUTPUT_HOST/$RESULTS_NAME/trajectory.txt);"
  echo "    see $BAG_OUTPUT_HOST/logs/vina_slam.log ==="
  exit 1
fi

# ── Phase 2: convert VINA-SLAM's output to an HDMapping session ──────────────
echo "=== Converting VINA-SLAM output to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 run vina_slam_to_hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RESULTS_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-vina-slam\"
  "

echo "=== DONE ==="
