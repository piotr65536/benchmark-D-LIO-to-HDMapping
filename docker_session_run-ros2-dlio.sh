#!/bin/bash

IMAGE_NAME='d-lio_humble'
TMUX_SESSION='ros2_D-LIO'

DATASET_CONTAINER_PATH='/ros2_ws/dataset/input.bag'
DATASET_ROS2_PATH='/tmp/dataset_ros2'
BAG_OUTPUT_CONTAINER='/ros2_ws/recordings'

RECORDED_BAG_NAME="recorded-D-LIO"
HDMAPPING_OUT_NAME="output_hdmapping"

LIDAR_TOPIC=/livox/pointcloud
IMU_TOPIC=/livox/imu
HZ_CLOUD=10.0
HZ_IMU=200.0
LIDAR_TYPE=livox
CALIBRATION_TIME=1.0

usage() {
  echo "Usage:"
  echo "  $0 <input.bag> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  LIDAR_TOPIC       - input PointCloud2 topic   (default: /livox/pointcloud)"
  echo "  IMU_TOPIC         - input IMU topic            (default: /livox/imu)"
  echo "  HZ_CLOUD          - LiDAR frequency in Hz      (default: 10.0)"
  echo "  HZ_IMU            - IMU frequency in Hz         (default: 200.0)"
  echo "  LIDAR_TYPE        - LiDAR type (ouster/hesai)   (default: ouster)"
  echo "  CALIBRATION_TIME  - static calibration time (s) (default: 1.0)"
  exit 1
}

echo "=== D-LIO rosbag pipeline ==="

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
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -f "$DATASET_HOST_PATH" ]]; then
  echo "Error: BAG file does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

echo "Input bag   : $DATASET_HOST_PATH"
echo "Output dir  : $BAG_OUTPUT_HOST"
echo "LiDAR topic : $LIDAR_TOPIC"
echo "IMU topic   : $IMU_TOPIC"
echo "LiDAR Hz    : $HZ_CLOUD"
echo "IMU Hz      : $HZ_IMU"
echo "LiDAR type  : $LIDAR_TYPE"

xhost +local:docker >/dev/null

# ── Phase 1: run D-LIO + record output topics ────────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash

    # ── Convert ROS 1 bag to ROS 2 format if needed ──
    if [[ "'"$DATASET_CONTAINER_PATH"'" == *.bag ]]; then
      echo "[convert] Converting ROS 1 bag to ROS 2 format..."
      echo "[convert] Input: '"$DATASET_CONTAINER_PATH"'"
      ls -la "'"$DATASET_CONTAINER_PATH"'" || { echo "[convert] ERROR: input bag not found!"; exit 1; }
      rm -rf "'"$DATASET_ROS2_PATH"'"
      rosbags-convert "'"$DATASET_CONTAINER_PATH"'" --dst "'"$DATASET_ROS2_PATH"'"
      if [[ ! -d "'"$DATASET_ROS2_PATH"'" ]]; then
        echo "[convert] ERROR: rosbags-convert failed! Output not created."
        exit 1
      fi
      ROS2_BAG="'"$DATASET_ROS2_PATH"'"
    else
      ROS2_BAG="'"$DATASET_CONTAINER_PATH"'"
    fi

    export ROS2_BAG
    echo "[convert] ROS 2 bag ready at: $ROS2_BAG"
    ls -la $ROS2_BAG/

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: D-LIO node + static TF ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Publish identity static transforms: common sensor frames -> base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link --ros-args -p use_sim_time:=true &
echo "[tf] Static transforms published (livox_frame, imu_link -> base_link)"

sleep 3
ros2 run dlio dlio_node --ros-args \
  -p in_cloud:='"$LIDAR_TOPIC"' \
  -p in_imu:='"$IMU_TOPIC"' \
  -p hz_cloud:='"$HZ_CLOUD"' \
  -p hz_imu:='"$HZ_IMU"' \
  -p lidar_type:='"$LIDAR_TYPE"' \
  -p calibration_time:='"$CALIBRATION_TIME"' \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p use_sim_time:=true \
  -p gyr_dev:=0.00117396706572 \
  -p gyr_rw_dev:=2.66e-07 \
  -p acc_dev:=0.0115432018302 \
  -p acc_rw_dev:=0.0000333 \
  -p keyframe_dist:=1.0 \
  -p keyframe_rot:=25.0 \
  -p tdfGridSizeX_low:=-100.0 \
  -p tdfGridSizeX_high:=100.0 \
  -p tdfGridSizeY_low:=-100.0 \
  -p tdfGridSizeY_high:=100.0 \
  -p tdfGridSizeZ_low:=-30.0 \
  -p tdfGridSizeZ_high:=30.0 \
  -p solver_max_iter:=100 \
  -p solver_max_threads:=12 \
  -p min_range:=1.0 \
  -p max_range:=100.0 \
  -p leaf_size:=-1.0 \
  -p robust_kernel_scale:=5.0 \
  -p maxCells:=100000 \
  -p timestamp_mode:=START_OF_SCAN
'\'' C-m

    # ---------- PANE 1: RViz visualization ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[rviz] launching RViz2..."
rviz2 -d /ros2_ws/install/dlio/share/dlio/launch/default.rviz --ros-args -p use_sim_time:=true
'\'' C-m

    # ---------- PANE 2: ros2 bag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
rm -rf '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] start"
ros2 bag record /odometry_pose /cloud -o '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANE 2: ros2 bag play ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[play] start"
ros2 bag play $ROS2_BAG --clock; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANE 3: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 8
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "=== ROS 2 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
ros2 topic list
echo ""
echo "--- Checking input topic: '"$LIDAR_TOPIC"' ---"
timeout 5 ros2 topic hz '"$LIDAR_TOPIC"' 2>&1 &
echo ""
echo "--- Checking input IMU: '"$IMU_TOPIC"' ---"
timeout 5 ros2 topic hz '"$IMU_TOPIC"' 2>&1 &
echo ""
echo "--- Checking D-LIO output: /odometry_pose ---"
timeout 5 ros2 topic hz /odometry_pose 2>&1 &
echo ""
echo "--- Checking D-LIO output: /cloud ---"
timeout 5 ros2 topic hz /cloud 2>&1 &
wait
echo ""
echo "=== If input topics show messages but D-LIO output shows nothing ==="
echo "=== check sensor parameters (lidar_type, hz_cloud, hz_imu).      ==="
echo ""
echo "--- Node list ---"
ros2 node list
echo ""
echo "[diag] done — you can type ROS 2 commands here, e.g.:"
echo "  ros2 topic list"
echo "  ros2 topic echo /odometry_pose"
echo "  ros2 topic hz /cloud"
'\'' C-m

    # ---------- Control window ----------
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
echo "[control] waiting for play end"
tmux wait-for BAG_DONE
echo "[control] bag playback finished — shutting down"

# Give D-LIO a moment to process remaining queued scans
sleep 3

# Graceful stop: Ctrl+C to each pane
# Pane layout: 0=D-LIO+TF, 1=rviz, 2=recorder, 3=play, 4=diag
echo "[control] sending Ctrl+C to all panes..."
tmux send-keys -t '"$TMUX_SESSION"':0.2 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.1 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.0 C-c
sleep 3

# Force-kill by process name (NOT -f which matches cmdline text of this shell!)
echo "[control] force-killing remaining processes..."
pkill -9 dlio_node 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
pkill -9 static_transform_publisher 2>/dev/null || true
sleep 1

echo "[control] terminating tmux"
tmux kill-server
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 run dlio-to-hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-D-LIO\"
  "

echo "=== DONE ==="
