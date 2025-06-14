#!/usr/bin/env bash
set -e
MY_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash

# convert Xacro → URDF
xacro "$MY_DIR/urdf/irb1200.xacro" > /tmp/irb1200.urdf

echo "🧹  Cleaning up stray Gazebo instances…"
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
sleep 1              # let the sockets close

# --- at top of run.sh, before starting Gazebo ---
echo "🧹  Cleaning up stray Gazebo instances…"
pkill -9 gzserver gzclient 2>/dev/null || true



# ---------- Gazebo (headless fallback) ----------
# Try full GUI first; if it crashes, fall back to headless
echo "🚀  Starting Gazebo ..."
ros2 launch gazebo_ros gazebo.launch.py \
        verbose:=true 2>/tmp/gz.log &
GZ_PID=$!

sleep 5
if ! ps -p $GZ_PID &>/dev/null; then
  echo "⚠️  Gazebo GUI crashed – retrying in headless mode."
  ros2 launch gazebo_ros gazebo.launch.py \
          verbose:=true gui:=false headless:=true &
  GZ_PID=$!
  sleep 3
fi

# ---------- Robot state publisher ----------
ros2 run robot_state_publisher robot_state_publisher \
        /tmp/irb1200.urdf --ros-args -p use_sim_time:=true &
RSP_PID=$!

# ---------- Spawn robot ----------
ros2 run gazebo_ros spawn_entity.py \
        -entity irb1200 -file /tmp/irb1200.urdf

# ---------- Controllers ----------
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active joint_trajectory_controller \
        --param-file "$MY_DIR/config/irb1200_controllers.yaml"

echo -e "\n✅ IRB1200 stick robot is live!  Ctrl-C to quit."
wait $GZ_PID $RSP_PID
