#!/bin/bash

# EPIC Exploration Algorithm Runner for Host PC (Pre-deployment Testing)
# This script sets up the environment and runs the core exploration algorithm on host PC
#
# Usage:
#   ./run_host_pc.sh [map_name] [run_sampler]
#
# Parameters:
#   map_name: garage|factory|cave (default: garage)
#   run_sampler: true|false (default: true)
#
# Examples:
#   ./run_host_pc.sh garage true
#   ./run_host_pc.sh factory false
#
# Note: LiDAR configuration should be set via launch files (host_pc_simulation.launch)

echo "🚁 Starting EPIC Exploration Algorithm on Host PC (Testing Mode)..."

# Host PC IPs (for testing)
HOST_PC_IP="127.0.0.1"         # Local testing
TARGET_BOARD_IP="127.0.0.1"    # Same as host for testing

# Get parameters from arguments
MAP_NAME=${1:-garage}
RUN_SAMPLER=${2:-true}         # Set to "true" to run trajectory sampler

# ========================================
# 🔧 TOPIC CONFIGURATION
# Change these topic names as needed
# ========================================

# ODOM_TOPIC="/lio_sam/mapping/odometry"
# CLOUD_TOPIC="/lio_sam/mapping/cloud_registered"

ODOM_TOPIC="/quad_0/lidar_slam/odom"
CLOUD_TOPIC="/quad0_pcl_render_node/cloud"

# Set paths for host PC (current workspace)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
HOST_WS="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"
EPIC_DIR="$HOST_WS/src/EPIC/src/global_planner"
CONFIG_FILE="$EPIC_DIR/exploration_manager/config/host_pc_$MAP_NAME.yaml"
TSP_DIR="$EPIC_DIR/utils/lkh_tsp_solver/resource"

# Debug TSP directory
echo "🔍 Checking TSP resource directory..."
echo "TSP_DIR: $TSP_DIR"
if [ -d "$TSP_DIR" ]; then
    echo "✅ Directory exists"
    echo "Contents:"
    ls -la "$TSP_DIR"
else
    echo "❌ Directory not found!"
    echo "Creating directory..."
    mkdir -p "$TSP_DIR"
fi

echo "🌐 Host PC IP (Testing): $HOST_PC_IP"
echo "🌐 Target Board IP (Testing): $TARGET_BOARD_IP"
echo "🗺️  Map: $MAP_NAME"
echo "📁 Config file: $CONFIG_FILE"
echo "🔬 Trajectory Sampler: $RUN_SAMPLER"
echo "📡 Odometry Topic: $ODOM_TOPIC"
echo "☁️  Pointcloud Topic: $CLOUD_TOPIC"
echo "📂 Workspace: $HOST_WS"
echo ""
echo "💡 Note: LiDAR configuration is handled by simulation launch file"

# Set ROS Master URI to point to local host
export ROS_MASTER_URI=http://$HOST_PC_IP:11311
export ROS_IP=$HOST_PC_IP
export ROS_HOSTNAME=$HOST_PC_IP

echo "🔗 ROS_MASTER_URI: $ROS_MASTER_URI"
echo "🔗 ROS_IP: $ROS_IP"
echo "🔗 ROS_HOSTNAME: $ROS_HOSTNAME"

# Source ROS
echo "📦 Sourcing ROS..."
source /opt/ros/noetic/setup.bash

# Source workspace (if devel/setup.bash exists)
if [ -f "$HOST_WS/devel/setup.bash" ]; then
    echo "📦 Sourcing workspace setup..."
    source "$HOST_WS/devel/setup.bash"
else
    echo "⚠️  Warning: workspace devel/setup.bash not found. Please build the workspace first."
fi

# Set up environment for host PC binaries
export LD_LIBRARY_PATH=$HOST_WS/devel/lib:$LD_LIBRARY_PATH
export PATH=$HOST_WS/devel/lib/epic_planner:$HOST_WS/devel/lib/plan_manage:$HOST_WS/devel/lib/traj_utils:$PATH
export ROS_PACKAGE_PATH=$HOST_WS/src:$ROS_PACKAGE_PATH

# Load default ROS parameters
echo "📝 Loading base ROS parameters..."
rosparam load $CONFIG_FILE /exploration_node

# Set custom topic names BEFORE starting nodes
echo "📝 Setting custom topic parameters..."
rosparam set /exploration_node/odometry_topic "$ODOM_TOPIC"
rosparam set /exploration_node/cloud_topic "$CLOUD_TOPIC"

echo "   ✅ Topic parameters set:"
echo "   - Odometry topic: $ODOM_TOPIC"
echo "   - Pointcloud topic: $CLOUD_TOPIC"

# Sync LiDAR parameters between rendering and exploration nodes
echo "📝 Syncing LiDAR parameters..."

# Check if required LiDAR parameters exist
RENDER_NODE="/quad0_pcl_render_node"
REQUIRED_PARAMS=(
    "is_360lidar"
    "vertical_fov"
    "sensing_horizon"
    "lidar_pitch"
    "yaw_fov"
)

MISSING_PARAMS=()
for param in "${REQUIRED_PARAMS[@]}"; do
    if ! rosparam get $RENDER_NODE/$param > /dev/null 2>&1; then
        MISSING_PARAMS+=("$RENDER_NODE/$param")
    fi
done

if [ ${#MISSING_PARAMS[@]} -ne 0 ]; then
    echo "❌ Error: Required LiDAR parameters not found:"
    for param in "${MISSING_PARAMS[@]}"; do
        echo "   - $param"
    done
    echo "   Please make sure simulation is running and parameters are set"
    exit 1
fi

# Read parameters from rendering node
IS_360_LIDAR=$(rosparam get $RENDER_NODE/is_360lidar)
VERTICAL_FOV=$(rosparam get $RENDER_NODE/vertical_fov)
SENSING_HORIZON=$(rosparam get $RENDER_NODE/sensing_horizon)
LIDAR_PITCH=$(rosparam get $RENDER_NODE/lidar_pitch)
YAW_FOV=$(rosparam get $RENDER_NODE/yaw_fov)

# Calculate FOV values based on LiDAR configuration
if [ "$IS_360_LIDAR" = "1" ]; then
    FOV_UP="90.0"
    FOV_DOWN="-90.0"
    FOV_VP_UP="90.0"
    FOV_VP_DOWN="-90.0"
    echo "   Using 360° LiDAR configuration"
else
    # For standard LiDAR, split vertical FOV equally
    FOV_UP=$(echo "$VERTICAL_FOV / 2" | bc -l)
    FOV_DOWN=$(echo "-1 * $VERTICAL_FOV / 2" | bc -l)
    # Add small margin for viewpoint FOV
    VP_MARGIN="2.0"
    FOV_VP_UP=$(echo "$FOV_UP - $VP_MARGIN" | bc -l)
    FOV_VP_DOWN=$(echo "$FOV_DOWN + $VP_MARGIN" | bc -l)
    echo "   Using standard LiDAR configuration:"
    echo "   - Vertical FOV: ${VERTICAL_FOV}°"
    echo "   - Yaw FOV: ${YAW_FOV}°"
fi

# Set exploration algorithm parameters
rosparam set /exploration_node/lidar_perception/max_ray_length "$SENSING_HORIZON"
rosparam set /exploration_node/lidar_perception/fov_up "$FOV_UP"
rosparam set /exploration_node/lidar_perception/fov_down "$FOV_DOWN"
rosparam set /exploration_node/lidar_perception/fov_viewpoint_up "$FOV_VP_UP"
rosparam set /exploration_node/lidar_perception/fov_viewpoint_down "$FOV_VP_DOWN"
rosparam set /exploration_node/lidar_perception/lidar_pitch "$LIDAR_PITCH"

echo "   ✅ LiDAR parameters synced:"
echo "   - Max ray length: ${SENSING_HORIZON}m"
echo "   - FOV up: ${FOV_UP}°"
echo "   - FOV down: ${FOV_DOWN}°"
echo "   - Viewpoint FOV up: ${FOV_VP_UP}°"
echo "   - Viewpoint FOV down: ${FOV_VP_DOWN}°"
echo "   - LiDAR pitch: ${LIDAR_PITCH}°"

# Check if required executables exist
REQUIRED_NODES=("exploration_node" "traj_server" "fast_planner_node")
for node in "${REQUIRED_NODES[@]}"; do
    if ! which $node &> /dev/null; then
        echo "❌ Error: $node not found. Please ensure the workspace is built correctly."
        echo "   Expected locations:"
        echo "   - exploration_node: $HOST_WS/devel/lib/epic_planner/"
        echo "   - traj_server: $HOST_WS/devel/lib/plan_manage/"
        echo "   - fast_planner_node: $HOST_WS/devel/lib/plan_manage/"
        echo ""
        echo "💡 To build workspace:"
        echo "   cd $HOST_WS"
        echo "   catkin_make"
        exit 1
    fi
done

# Check for trajectory sampler if requested
if [ "$RUN_SAMPLER" = "true" ]; then
    if [ -f "$HOST_WS/devel/lib/epic_planner/custom_trajectory_sampler" ]; then
        echo "✅ Trajectory sampler found"
    else
        echo "❌ Error: custom_trajectory_sampler not found. Please build it first."
        exit 1
    fi
fi

echo "✅ All required nodes found"

# Check if required topics are available (from simulation)
echo "🔍 Checking for required input topics..."
echo "   This may take up to 10 seconds..."

# Input topics from simulator (use custom topic names)
REQUIRED_TOPICS=(
    "$ODOM_TOPIC"
    "$CLOUD_TOPIC"
)

# Output topics to monitor
OUTPUT_TOPICS=(
    "/planning/pos_cmd"
    "/planning/bspline"
    "/exploration_finish"
)

# Monitor input topics
for topic in "${REQUIRED_TOPICS[@]}"; do
    timeout 10 rostopic echo $topic -n 1 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ Topic $topic is available"
    else
        echo "⚠️  Warning: Topic $topic not available yet"
        echo "   Make sure the simulation is running"
        echo "   You can start simulation separately"
    fi
done

# Start monitoring output topics in background
echo "📊 Starting output topic monitoring..."
for topic in "${OUTPUT_TOPICS[@]}"; do
    (rostopic echo $topic > /dev/null 2>&1 &)
    echo "   - Monitoring $topic"
done

echo "🚀 Starting EPIC exploration algorithm..."
echo "   Map: $MAP_NAME"
echo "   Config: $CONFIG_FILE"
echo "   Odometry: $ODOM_TOPIC"
echo "   Pointcloud: $CLOUD_TOPIC"
echo ""
echo "📊 Monitor the following topics for output:"
echo "   - /planning/pos_cmd (position commands)"
echo "   - /planning/bspline (trajectory)"
echo "   - /exploration_finish (completion status)"
echo ""
echo "🛑 Press Ctrl+C to stop"
echo ""

# Run the nodes directly (parameters already set via rosparam)
echo "🚀 Starting exploration_node..."
cd $TSP_DIR && \
$HOST_WS/devel/lib/epic_planner/exploration_node \
    __name:=exploration_node &

EXPLORATION_PID=$!

echo "🚀 Starting traj_server..."
$HOST_WS/devel/lib/plan_manage/traj_server \
    __name:=traj_server \
    _config_file:=$CONFIG_FILE \
    /position_cmd:=/planning/pos_cmd  &

TRAJ_SERVER_PID=$!

echo "🚀 Starting fast_planner_node..."
$HOST_WS/devel/lib/plan_manage/fast_planner_node \
    __name:=fast_planner_node \
    _config_file:=$CONFIG_FILE  &

FAST_PLANNER_PID=$!

# Start trajectory sampler if requested
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "🚀 Starting custom_trajectory_sampler..."
    $HOST_WS/devel/lib/epic_planner/custom_trajectory_sampler \
        __name:=trajectory_sampler \
        _sample_interval:=0.2 \
        _total_duration:=10.0 \
        _auto_duration:=true  &
    
    SAMPLER_PID=$!
    echo "✅ Trajectory sampler started (PID: $SAMPLER_PID)"
    echo "📤 Output topics:"
    echo "   - /planning/trajectory_discrete"
    echo "   - /planning/trajectory_waypoints"
fi

# Wait for interrupt
trap "echo '🛑 Stopping EPIC nodes...'; kill $EXPLORATION_PID $TRAJ_SERVER_PID $FAST_PLANNER_PID $SAMPLER_PID 2>/dev/null; exit 0" INT

echo "✅ All nodes started. Waiting..."
wait

echo "🏁 EPIC exploration algorithm stopped" 