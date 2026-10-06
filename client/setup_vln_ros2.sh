#!/usr/bin/env bash
# ============================================================================
# 一次把跑 VLN ROS 2 桥接所需的环境准备好。
#
#   用法（必须 source，不能 ./ 执行）：
#       source ~/vln/setup_vln_ros2.sh
#       $VLN_PY vln_ros2_bridge.py --instruction "go to the red chair"
#
# 做了什么：
#   1. 依次 source ROS 2 Jazzy → 宇树 cyclonedds → lighting_ws
#   2. 设 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp（与 G1 同域，必须）
#   3. 兜底设 CYCLONEDDS_URI 到 enP2p1s0（宇树内网口）
#   4. 导出 VLN_PY = 装了 websockets/Pillow/rclpy 依赖的那个 venv python
#
# 为什么要用 $VLN_PY 而不是 python3：
#   本机默认 python3 是 miniconda 的 3.14，而 ROS 2 Jazzy 与 pyrealsense2
#   都是按 python3.12 编的 —— 用 3.14 会 import 失败。
# ============================================================================

VLN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export VLN_DIR

# ---- 1. ROS 2 Jazzy ----
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  echo "❌ 找不到 /opt/ros/jazzy/setup.bash" >&2
fi

# ---- 2. lighting_ws（g1_nav_bridge / realsense2_camera / 各消息包，只用 install/）----
LIGHTING_WS=/opt/G1/lighting_ws/install/setup.bash
if [ -f "$LIGHTING_WS" ]; then
  # shellcheck disable=SC1091
  source "$LIGHTING_WS"
else
  echo "❌ 找不到 $LIGHTING_WS，g1_nav_bridge 和 realsense2_camera 会不可用" >&2
fi

# ---- 3. DDS：和导航栈同一份配置（cyclonedds_g1.xml，已调好大图像流）----
# cmdvel_to_sport 要通过 CycloneDDS 的 /api/sport/request 和 G1 本体说话，
# 所以相机、桥接、cmdvel_to_sport 必须同在这个 RMW 里，不能再用 fastrtps。
DDS_ENV=/opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/system/dds_env.sh
if [ -f "$DDS_ENV" ]; then
  # shellcheck disable=SC1090
  source "$DDS_ENV" >/dev/null
else
  echo "❌ 找不到 $DDS_ENV（需要先部署 lighting_ws）" >&2
fi

# ---- 6. venv 解释器 ----
VLN_PY="$HOME/vln_env/bin/python"
if [ -x "$VLN_PY" ]; then
  export VLN_PY
else
  echo "❌ 找不到 venv：$VLN_PY" >&2
  echo "   重建：/usr/bin/python3.12 -m venv ~/vln_env" >&2
  echo "        然后 curl -fsSL https://bootstrap.pypa.io/get-pip.py | ~/vln_env/bin/python" >&2
  echo "        ~/vln_env/bin/python -m pip install -r $VLN_DIR/requirements-robot.txt" >&2
  echo "        ~/vln_env/bin/python -m pip install pyrealsense2 pyyaml" >&2
  unset VLN_PY
fi

echo "────────── VLN ROS 2 环境 ──────────"
echo "ROS_DISTRO      : ${ROS_DISTRO:-未设置}"
echo "RMW             : ${RMW_IMPLEMENTATION:-未设置}"
echo "VLN_PY          : ${VLN_PY:-未设置}"
echo "CYCLONEDDS_URI  : ${CYCLONEDDS_URI:-未设置}"
echo "speed 输出话题  : /vln/cmd_vel（安全）| 真机 /vln/cmd_vel_robot"
echo "────────────────────────────────────"
