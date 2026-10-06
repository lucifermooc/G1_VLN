#!/usr/bin/env bash
# ============================================================================
# start_vln.sh —— VLN 启动 / 网页开关 / 停止
#
#   ── 第一步：环境全部 ready（相机 + 真机转发 + 桥接，不含网页）──
#   ./start_vln.sh                       测试·单步（默认，不会自己动）
#   ./start_vln.sh "go to the red chair" 指定指令
#   ./start_vln.sh --robot               真机模式（发到 /cmd_vel_safe）
#   ./start_vln.sh --mode auto           正式·连续（要人放行）
#   ./start_vln.sh --step-exec-s 2       单步走 2 秒
#
#   ── 第二步：把网页挂上去（环境已经 ready 之后）──
#   ./start_vln.sh --web                 挂上网页，打印地址
#   ./start_vln.sh --web-off             摘掉网页（桥接/相机继续跑）
#
#   一步到位（想省事就用这个，等于上面两步连着做）：
#   ./start_vln.sh --all
#
#   ── 其它 ──
#   ./start_vln.sh --status              看当前在跑什么（含网页状态）
#   ./start_vln.sh --stop                全部停掉
#   ./start_vln.sh --stop --keep-camera  停桥接但留着相机
#
# 为什么拆成两步
# ----------------------------------------------------------------------------
#   环境（相机会话、服务端 login/reset、图有没有来）要几十秒才能确认好，
#   而网页是随时可以挂上摘下的东西。先让环境静悄悄 ready、确认没事了，
#   再开网页去操作；网页崩了也只摘网页，不用把整条闭环重启一遍。
#   所以「环境起来」时是**没有网页、也没有授权**的状态 —— 不会自己动。
#
# 两种模式
# ----------------------------------------------------------------------------
#   step（测试，默认）：推一帧 → 你看结果 → 【确认执行】才走 --step-exec-s 秒。
#       不确认就一直静止，等超时自动丢弃。首次调试用这个，它不会自己动。
#   auto（正式）：连续推理并下发，直到 stop=true 或点【⛔ 一键终止】。
#       默认要先放行（网页点【开始连续】，或 ./tools/vln_ctl.py go）。
#
# 设计要点
# ----------------------------------------------------------------------------
# * **安全默认**：不加 --robot 就发到 /vln/cmd_vel（没有任何订阅者）；
#   而且起来时是「未授权 + 没网页」，也就是「跑起来也绝对不会动」。
# * **相机和桥接必须在同一个 shell 里起**，这样两者 RMW 一致。
#   若相机是别人用 CycloneDDS 起的，本脚本会看不到图像（见 START.md 排错）。
# * 日志统一进 logs/，出问题先看 logs/bridge.log。
# ============================================================================
set -uo pipefail

VLN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$VLN_DIR"
mkdir -p logs

# ---------------------------------------------------------------------------
# 默认值（可用命令行覆盖）
# ---------------------------------------------------------------------------
INSTRUCTION="go to the red chair"
USE_WEB=0                   # 1 = 启动桥接时就把网页挂上（--all 用）
DO_WEB=0                    # 1 = 只挂网页（--web）
DO_WEB_OFF=0                # 1 = 只摘网页（--web-off）
WITH_CAMERA=1               # --stop 时是否也停相机
START_CAMERA=1              # --no-camera 时置 0
MODE="safe"                 # safe = 发 /vln/cmd_vel(无人订阅) | robot = 发 /cmd_vel_safe
RUN_MODE="step"             # step = 测试·单步（默认，不会自己动） | auto = 正式·连续
STEP_EXEC_S=""              # 空 = 用桥接默认值
AUTO_CONFIRM=0              # 1 = 正式模式免放行，起来就跑
DO_STOP=0
DO_STATUS=0
WEB_PORT=8090
IMG_TOPIC="/vln/camera/color/image_raw"   # 自己的命名空间，和导航栈的 /camera/camera 完全分开
SERVER_URL="${VLN_SERVER:-ws://127.0.0.1:8050}"
G1_VLN_ROOT="$(cd "$VLN_DIR/.." && pwd)"
CAM_PAT="usb_camera_node.py"                  # VLN 自己的 USB 相机节点（不再使用 RealSense）
CAM_DEV="${VLN_CAM_DEV:-/dev/v4l/by-id/usb-STYT_241029_K_USB_Camera_01.00.00-video-index0}"; CAM_SIZE="${VLN_CAM_SIZE:-2560x720}"; CAM_EYE="left"; CAM_ROT="0"
CMD_PAT="__node:=g1_vln_cmdvel_to_sport"      # 只匹配 VLN 自己的 cmdvel_to_sport
ODOM_PAT="__node:=g1_vln_sport_to_odom"       # VLN 自己的里程计 /vln/odom
MPC_PAT="vln_mpc.mpc_node"                    # 官方 vln_mpc（/vln 命名空间）
FOLLOW=0                                      # --follow：跟人（MPC track 模式、忽略 stop）
CTRL="mpc"                                    # mpc = 整条轨迹交给 MPC 跟踪；wp0 = 只用第 1 个航点（旧行为）
CTL_SOCK="/tmp/vln_bridge.sock"

CAM_LOG="logs/camera.log"
BRIDGE_LOG="logs/bridge.log"

# ---------------------------------------------------------------------------
# 用法
# ---------------------------------------------------------------------------
usage() {
  sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'
}

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --stop)        DO_STOP=1; shift ;;
    --keep-camera) WITH_CAMERA=0; shift ;;
    --status)      DO_STATUS=1; shift ;;
    --web)         DO_WEB=1; shift ;;
    --web-off|--no-web-panel)
                   DO_WEB_OFF=1; shift ;;
    --all|--with-web)
                   USE_WEB=1; shift ;;
    --robot)       MODE="robot"; shift ;;
    -m|--mode|--run-mode)
                   RUN_MODE="${2:?--mode 需要 step 或 auto}"; shift 2 ;;
    --step-exec-s) STEP_EXEC_S="${2:?--step-exec-s 需要秒数}"; shift 2 ;;
    --auto-confirm) AUTO_CONFIRM=1; shift ;;
    --camera)      [ "${2:?--camera 需要 usb}" = usb ] || { echo "❌ 只支持 --camera usb（RealSense 已弃用）" >&2; exit 1; }; shift 2 ;;
    --follow)      FOLLOW=1; shift ;;
    --ctrl)        CTRL="${2:?--ctrl 需要 mpc 或 wp0}"; shift 2 ;;
    --cam-device)  CAM_DEV="${2:?}"; shift 2 ;;
    --cam-size)    CAM_SIZE="${2:?}"; shift 2 ;;
    --cam-eye)     CAM_EYE="${2:?left|right}"; shift 2 ;;
    --cam-rotate)  CAM_ROT="${2:?0|180}"; shift 2 ;;
    --no-camera)   START_CAMERA=0; WITH_CAMERA=0; shift ;;
    --no-web)      USE_WEB=0; shift ;;
    --port)        WEB_PORT="${2:?--port 需要端口号}"; shift 2 ;;
    -i|--instruction) INSTRUCTION="${2:?--instruction 需要指令文本}"; shift 2 ;;
    -h|--help)     usage; exit 0 ;;
    -*)            echo "❌ 未知参数：$1（用 -h 看用法）" >&2; exit 1 ;;
    *)             INSTRUCTION="$1"; shift ;;
  esac
done

# ---------------------------------------------------------------------------
# 网页开关（不需要 ROS 环境，纯粹连桥接的控制通道）
# ---------------------------------------------------------------------------
# 用 python 直连 unix socket。不用 vln_ctl.py 是为了在 venv 还没检查之前
# 就能给出清晰报错，也免得为了看网页状态去 source 一整套 ROS。
web_ctl() {
  local action="$1"
  # 故意不依赖 $VLN_PY：--web / --status 要在「还没 source 环境」时也能用。
  local py="${VLN_PY:-/usr/bin/python3}"
  "$py" - "$action" "$CTL_SOCK" <<'PYEOF'
import json, socket, sys
action, path = sys.argv[1], sys.argv[2]
try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(20.0)
    s.connect(path)
except OSError as e:
    print(f"NOTRUNNING:{e}")
    sys.exit(3)
try:
    s.sendall((json.dumps({"cmd": "web", "body": {"action": action}}) + "\n").encode())
    data = b""
    while b"\n" not in data:
        chunk = s.recv(65536)
        if not chunk:
            break
        data += chunk
finally:
    s.close()
try:
    r = json.loads(data.decode("utf-8"))
except Exception as e:
    print(f"BADREPLY:{e}")
    sys.exit(4)
print(json.dumps(r, ensure_ascii=False))
PYEOF
}

print_web_urls() {
  local py="${VLN_PY:-/usr/bin/python3}"
  "$py" - "$WEB_PORT" <<'PYEOF'
import subprocess
import sys

port = sys.argv[1]
print(f"   本机    : http://127.0.0.1:{port}/")

try:
    out = subprocess.run(["hostname", "-I"], capture_output=True, text=True,
                         timeout=5).stdout
except Exception:
    out = ""
ips = [ip for ip in out.split() if not ip.startswith("127.")]

# docker / 虚拟网桥那一堆 172.x 打出来只会让人困惑，优先 192.168 / 10. 网段
lan = [ip for ip in ips if ip.startswith("192.168.") or ip.startswith("10.")]
for ip in (lan or ips):
    print(f"   局域网  : http://{ip}:{port}/")
PYEOF
}

# ---------------------------------------------------------------------------
# 停止 / 状态
# ---------------------------------------------------------------------------
# 相机被 ROS launch 包了一层：光杀 launch 进程会留下孤儿的
# realsense2_camera_node 继续占着 USB 设备，下次启动就 "Device busy"。
# 所以先杀 launch，再杀 node，最后 SIGKILL 兜底。
kill_pattern() {
  local pat="$1" name="$2"
  if pgrep -f "$pat" >/dev/null 2>&1; then
    pkill -f "$pat" 2>/dev/null
    for _ in 1 2 3 4 5 6; do
      pgrep -f "$pat" >/dev/null 2>&1 || break
      sleep 0.5
    done
    if pgrep -f "$pat" >/dev/null 2>&1; then
      pkill -9 -f "$pat" 2>/dev/null
      sleep 0.5
    fi
    if pgrep -f "$pat" >/dev/null 2>&1; then
      echo "  ⚠️ $name 仍有残留，手工确认： pgrep -af '$pat'"
    else
      echo "  ✓ $name 已停"
    fi
  else
    echo "  · $name 本来就没跑"
  fi
}

if [ "$DO_STOP" = 1 ]; then
  echo "── 停止 VLN ──"
  kill_pattern "vln_ros2_bridge.py"        "桥接"
  kill_pattern "$CMD_PAT"                  "cmdvel_to_sport(VLN)"
  kill_pattern "$MPC_PAT"                  "MPC(VLN)"
  kill_pattern "$ODOM_PAT"                 "里程计(VLN)"
  if [ "$WITH_CAMERA" = 1 ]; then
    kill_pattern "$CAM_PAT"                "相机节点"
  else
    echo "  · 相机按你的要求保留（--keep-camera / --no-camera）"
  fi
  echo "完成。"
  exit 0
fi

if [ "$DO_STATUS" = 1 ]; then
  echo "── 当前状态 ──"
  if pgrep -f "$CAM_PAT" >/dev/null 2>&1; then
    echo "  相机    : 运行中 (pid $(pgrep -f "$CAM_PAT" | head -1))"
  else
    echo "  相机    : 未运行"
  fi
  if pgrep -f "vln_ros2_bridge.py" >/dev/null 2>&1; then
    echo "  桥接    : 运行中 (pid $(pgrep -f vln_ros2_bridge.py | head -1))"
  else
    echo "  桥接    : 未运行"
  fi
  if pgrep -f "$CMD_PAT" >/dev/null 2>&1; then
    echo "  真机转发: 运行中 (pid $(pgrep -f "$CMD_PAT" | head -1))"
  else
    echo "  真机转发: 未运行"
  fi

  for _p in "$MPC_PAT|MPC     " "$ODOM_PAT|里程计  "; do
    if pgrep -f "${_p%%|*}" >/dev/null 2>&1; then echo "  ${_p#*|}: 运行中"; else echo "  ${_p#*|}: 未运行"; fi
  done

  # 网页状态以桥接自己上报的为准（比扫端口准：端口可能是别的程序占的）
  web_line=$(web_ctl status 2>/dev/null || true)
  if printf '%s' "$web_line" | grep -q '^NOTRUNNING'; then
    echo "  网页    : — （桥接没跑，谈不上网页）"
  elif printf '%s' "$web_line" | grep -q '^{'; then
    web_on=$("${VLN_PY:-/usr/bin/python3}" -c \
      'import json,sys; print(json.load(sys.stdin).get("web"))' <<<"$web_line" 2>/dev/null || echo "?")
    web_port=$("${VLN_PY:-/usr/bin/python3}" -c \
      'import json,sys; print(json.load(sys.stdin).get("port") or "")' <<<"$web_line" 2>/dev/null || echo "")
    if [ "$web_on" = "True" ]; then
      echo "  网页    : 开（端口 $web_port）"
      WEB_PORT="$web_port"
      print_web_urls
    else
      echo "  网页    : 关（开： ./start_vln.sh --web）"
    fi
  else
    echo "  网页    : ？ （控制通道没响应）"
  fi
  echo "  桥接日志: $BRIDGE_LOG"
  echo "  相机日志: $CAM_LOG"
  exit 0
fi

# ---------------------------------------------------------------------------
# 1. 环境（必须 source，才能和相机共享同一个 RMW）
# ---------------------------------------------------------------------------
# ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等），
# 在 set -u 下会直接报 "unbound variable" 退出 —— source 期间先关掉 -u。
set +u
# shellcheck disable=SC1091
source "$VLN_DIR/setup_vln_ros2.sh" >/dev/null
SRC_RC=$?
set -u
if [ "$SRC_RC" != 0 ]; then
  echo "❌ source setup_vln_ros2.sh 失败" >&2; exit 1;
fi
if [ -z "${VLN_PY:-}" ]; then
  echo "❌ 没找到 venv（\$VLN_PY 为空），先按 START.md「第 0 步」修环境" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# 第二步：只开关网页（环境已经在跑，这里不碰相机/桥接）
# ---------------------------------------------------------------------------
if [ "$DO_WEB" = 1 ] || [ "$DO_WEB_OFF" = 1 ]; then
  echo "════════════════════════════════════════════"
  if [ "$DO_WEB_OFF" = 1 ]; then
    echo " 摘掉网页面板"
  else
    echo " 挂上网页面板"
  fi
  echo "════════════════════════════════════════════"

  if ! pgrep -f "vln_ros2_bridge.py" >/dev/null 2>&1; then
    echo "❌ 桥接没在跑。先起环境： ./start_vln.sh" >&2
    echo "   （要一步到位就用： ./start_vln.sh --all）" >&2
    exit 1
  fi

  action=$([ "$DO_WEB_OFF" = 1 ] && echo off || echo on)
  reply=$(web_ctl "$action" 2>/dev/null || true)

  if printf '%s' "$reply" | grep -q '^NOTRUNNING'; then
    echo "❌ 连不上桥接的控制通道 $CTL_SOCK" >&2
    echo "   桥接进程在，但通道不通 —— 多半是旧版本桥接（还没有 --ctl-socket）。" >&2
    echo "   重启一下： ./start_vln.sh --stop && ./start_vln.sh${DO_WEB_OFF:+ --web-off}" >&2
    exit 1
  fi
  if printf '%s' "$reply" | grep -q '^BADREPLY'; then
    echo "❌ 控制通道返回了看不懂的东西： $reply" >&2
    exit 1
  fi

  if [ "$DO_WEB_OFF" = 1 ]; then
    echo "🖥️  网页面板已摘掉（桥接、相机照跑；推理逻辑完全不受影响）"
    echo "    想再挂回来： ./start_vln.sh --web"
  else
    WEB_PORT="$("${VLN_PY:-/usr/bin/python3}" -c \
      'import json,sys; print(json.load(sys.stdin).get("port") or "")' \
      <<<"$reply" 2>/dev/null || echo "")"
    WEB_PORT="${WEB_PORT:-$WEB_PORT}"
    if echo "$reply" | grep -q '"web": *false'; then
      echo "⚠️ 网页没能起来，看桥接日志： tail -30 $BRIDGE_LOG" >&2
      exit 1
    fi
    echo "🖥️  网页面板已启动"
    print_web_urls
    echo
    echo "   默认是「未授权」—— 网页上点【授权】或直接操作单步都可以。"
    echo "   摘掉： ./start_vln.sh --web-off      全部停： ./start_vln.sh --stop"
  fi
  exit 0
fi

echo "════════════════════════════════════════════"
echo " VLN 启动 · 第①步：环境 ready"
echo "════════════════════════════════════════════"
echo " 指令      : $INSTRUCTION"
echo " 速度话题  : $([ "$MODE" = robot ] && echo '/vln/cmd_vel_robot → cmdvel_to_sport → G1（真机！）' || echo '/vln/cmd_vel（安全，无人订阅）')"
if [ "$RUN_MODE" = step ]; then
  echo " 运行模式  : 测试·单步（推一帧 → 人确认才走 ${STEP_EXEC_S:-1.0}s）"
else
  if [ "$AUTO_CONFIRM" = 1 ]; then
    echo " 运行模式  : 正式·连续（自动放行）"
  else
    echo " 运行模式  : 正式·连续（需网页点【开始连续】放行）"
  fi
fi
# 这段警告已经过时了：现在没网页也不会偷偷改成连续模式，
# 桥接会老老实实停在 idle 等人（见 vln_ros2_bridge.py 里的说明）。
if [ "$USE_WEB" = 0 ]; then
  echo " 网页      : 还没开（第②步： ./start_vln.sh --web）"
  if [ "$RUN_MODE" = step ]; then
    echo "             ↳ 眼下想推一帧： ./tools/vln_ctl.py step"
  fi
else
  echo " 网页      : 启动时一并挂上（端口 $WEB_PORT）"
fi
echo " RMW       : ${RMW_IMPLEMENTATION:-未设置}"
echo

# ---------------------------------------------------------------------------
# 1.5 真机互斥检查 + 推理服务
# ---------------------------------------------------------------------------
# 相机是独立的 USB 相机，和导航栈互不影响；但速度出口（G1 运动接口）只能一个人用，
# 所以真机模式要求导航栈已停（不替你停：先 ros2 launch robot_bringup stop_all.launch.py）。
if [ "$MODE" = robot ] && pgrep -af "robot.launch.py|g1_navigation.launch.py|g1_cmdvel_to_sport" \
     | grep -vE "g1_vln_cmdvel_to_sport|pgrep" | grep -q .; then
  echo "❌ 导航栈还在运行，真机模式会和它抢 G1 的速度出口。" >&2
  echo "   先停导航栈： ros2 launch robot_bringup stop_all.launch.py" >&2
  echo "   （安全模式 不加 --robot 可以和导航栈同时跑）" >&2
  exit 1
fi
if ! ss -ltn 2>/dev/null | grep -q ":${SERVER_URL##*:} "; then
  echo "· 推理服务没在跑，启动 LightNav-0 服务（约 2~3 分钟）"
  ( env -u PYTHONPATH -u LD_LIBRARY_PATH bash "$G1_VLN_ROOT/server/start_server.sh" ) \
    > logs/server_start.log 2>&1 || { echo "❌ 推理服务启动失败，看 $VLN_DIR/logs/server_start.log" >&2; exit 1; }
fi

# ---------------------------------------------------------------------------
# 2. 清掉上一次的桥接（相机会保留，避免每次等它重新拉起）
# ---------------------------------------------------------------------------
if pgrep -f "vln_ros2_bridge.py" >/dev/null 2>&1; then
  echo "· 发现旧桥接进程，先停掉"
  pkill -f "vln_ros2_bridge.py" 2>/dev/null
  sleep 2
fi

# ---------------------------------------------------------------------------
# 3. 相机
# ---------------------------------------------------------------------------
if [ "$START_CAMERA" = 1 ]; then
  if pgrep -f "$CAM_PAT" >/dev/null 2>&1; then
    echo "· 相机已在运行，跳过启动"
  else
    echo "· 启动 USB 双目相机（$CAM_DEV $CAM_SIZE，取${CAM_EYE}眼）"
    echo "  日志 → $CAM_LOG"
    nohup "$VLN_PY" "$VLN_DIR/usb_camera_node.py" --device "$CAM_DEV" --size "$CAM_SIZE" \
      --eye "$CAM_EYE" --rotate "$CAM_ROT" --topic "$IMG_TOPIC" \
      > "$CAM_LOG" 2>&1 &
    disown 2>/dev/null || true
  fi

  echo "· 等相机出图 …"
  ready=0
  for i in $(seq 1 25); do
    if timeout 4 ros2 topic echo "$IMG_TOPIC" --once --field header \
         >/dev/null 2>&1; then
      ready=1; break
    fi
    sleep 1
  done

  if [ "$ready" = 1 ]; then
    echo "  ✓ 相机就绪"
  else
    echo "  ⚠️ 15 秒内没等到 $IMG_TOPIC 的图像"
    if pgrep -f "$CAM_PAT" >/dev/null 2>&1; then
      echo "     相机进程在跑但没图 —— 最可能是 RMW 不一致（相机是别的 shell 起的）"
      echo "     先停干净再重来： ./start_vln.sh --stop && ./start_vln.sh"
    else
      echo "     相机没起来，看 $CAM_LOG ："
      tail -5 "$CAM_LOG" 2>/dev/null | sed 's/^/       /'
      echo "     常见原因：相机被别的进程占用（Device or resource busy）→ 先 ./start_vln.sh --stop"
    fi
    echo "   仍然继续启动桥接（桥接会等图，等到了自己就开始跑）"
  fi
fi

# ---------------------------------------------------------------------------
# 4. 桥接 + 网页
# ---------------------------------------------------------------------------
BRIDGE_ARGS=(
  --instruction "$INSTRUCTION"
  --server "$SERVER_URL"
  --image-topic "$IMG_TOPIC"
  --encode-width 640
  --fps 4.0
  --mode "$RUN_MODE"
)

[ -n "$STEP_EXEC_S" ] && BRIDGE_ARGS+=(--step-exec-s "$STEP_EXEC_S")
[ "$AUTO_CONFIRM" = 1 ] && BRIDGE_ARGS+=(--auto-confirm)

if [ "$MODE" = robot ]; then
  # 真机：VLN 自己的 cmdvel_to_sport（节点名 g1_vln_cmdvel_to_sport），只听 /vln/cmd_vel_robot。
  # 参数见 vln_cmdvel.yaml（数值同 nav_bridge.yaml），G1 最小有效速度的处理和导航一致。
  # 导航栈已停（上面检查过），不会有第二个节点同时给 G1 发速度。
  BRIDGE_ARGS+=(--publish --cmd-vel-topic /vln/cmd_vel_robot)
  NB_YAML="$VLN_DIR/vln_cmdvel.yaml"
  if [ -f "$NB_YAML" ]; then
    if ! pgrep -f "$CMD_PAT" >/dev/null 2>&1; then
      echo "· 启动 cmdvel_to_sport（/vln/cmd_vel_robot → G1 运动接口）"
      nohup ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
        -r __node:=g1_vln_cmdvel_to_sport --params-file "$NB_YAML" \
        > logs/cmdvel.log 2>&1 &
      disown 2>/dev/null || true
    else
      echo "· cmdvel_to_sport 已在运行"
    fi
  else
    echo "❌ 没找到 $NB_YAML" >&2; exit 1
  fi
else
  # 安全模式：发布到 /vln/cmd_vel，只用来验证「推理→速度」这条链，
  # 机器人不会动（没有任何节点订阅这个话题）
  BRIDGE_ARGS+=(--publish)
fi

BRIDGE_ARGS+=(--ctrl "$CTRL")
[ "$FOLLOW" = 1 ] && BRIDGE_ARGS+=(--follow)
if [ "$CTRL" = mpc ]; then
  BRIDGE_ARGS+=(--max-vx 0.45 --max-wz 0.9)   # 与 vln_mpc_g1.yaml 的限速一致
  MPC_YAML="$VLN_DIR/vln_mpc_g1.yaml"
  if ! pgrep -f "$ODOM_PAT" >/dev/null 2>&1; then
    echo "· 启动 VLN 里程计（/odommodestate → /vln/odom，不用导航栈的 /odom）"
    nohup ros2 run g1_nav_bridge sport_to_odom --ros-args \
      -r __node:=g1_vln_sport_to_odom -r __ns:=/vln --params-file "$MPC_YAML" \
      > logs/odom.log 2>&1 &
    disown 2>/dev/null || true
  fi
  if ! pgrep -f "$MPC_PAT" >/dev/null 2>&1; then
    echo "· 启动 MPC（官方 vln_mpc，10 Hz 跟踪整条轨迹）"
    PYTHONNOUSERSITE=1 PYTHONPATH="$G1_VLN_ROOT/LightNav-0/robot_deploy/src/vln_mpc:${PYTHONPATH:-}" \
      nohup "$VLN_PY" -m vln_mpc.mpc_node --ros-args \
      -r __ns:=/vln --params-file "$MPC_YAML" \
      > logs/mpc.log 2>&1 &
    disown 2>/dev/null || true
  fi
fi

[ "$USE_WEB" = 1 ] && BRIDGE_ARGS+=(--web --web-port "$WEB_PORT")

# 本地控制通道：网页没开/挂了/还没挂的时候，命令行照样能操作节点，
# 也能把网页再拉起来（./start_vln.sh --web 走的就是它）。
BRIDGE_ARGS+=(--ctl-socket "$CTL_SOCK")
rm -f "$CTL_SOCK" 2>/dev/null || true

echo "· 启动桥接"
echo "  日志 → $BRIDGE_LOG"
nohup "$VLN_PY" vln_ros2_bridge.py "${BRIDGE_ARGS[@]}" \
  > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
disown 2>/dev/null || true

# ---------------------------------------------------------------------------
# 5. 等桥接起来，把实际网址打出来
# ---------------------------------------------------------------------------
sleep 6
echo
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "❌ 桥接启动失败，最后 20 行："
  tail -20 "$BRIDGE_LOG" | sed 's/^/    /'
  exit 1
fi

if [ "$USE_WEB" = 1 ]; then
  ACTUAL_PORT=""
  for p in $(seq "$WEB_PORT" $((WEB_PORT + 11))); do
    if ss -ltn 2>/dev/null | grep -q ":$p "; then ACTUAL_PORT="$p"; break; fi
  done

  echo "════════════════════════════════════════════"
  echo " ✅ 环境 ready + 网页已挂上"
  echo "════════════════════════════════════════════"
  if [ -n "$ACTUAL_PORT" ]; then
    if [ "$ACTUAL_PORT" != "$WEB_PORT" ]; then
      echo "   （$WEB_PORT 被占用，自动换到了 $ACTUAL_PORT）"
    fi
    WEB_PORT="$ACTUAL_PORT"
    print_web_urls
  else
    echo " ⚠️ 没探到监听端口，看 $BRIDGE_LOG 里的「网页面板」那几行"
  fi
  echo
else
  echo "════════════════════════════════════════════"
  echo " ✅ 环境 ready（第①步完成）"
  echo "════════════════════════════════════════════"
  echo " 相机　　 : $([ "$START_CAMERA" = 1 ] && echo '已在跑' || echo '—（--no-camera）')"
  echo " 机器人　 : $([ "$MODE" = robot ] && echo '/vln/cmd_vel_robot → cmdvel_to_sport 已接' || echo '安全模式，没接真机')"
  echo " 网页　　 : 还没开（这是故意的）"
  echo
  echo " 第②步：挂上网页去看画面 / 操作"
  echo "   ./start_vln.sh --web"
  echo " 不想开网页，也能干活："
  echo "   ./tools/vln_ctl.py step       推一帧"
  echo "   ./tools/vln_ctl.py confirm    确认执行"
  echo "   ./tools/vln_ctl.py go         正式模式放行"
  echo "   ./tools/vln_ctl.py abort      ⛔ 一键终止"
  echo
  echo " 现在这个状态是「未授权」，也就是：推理可以跑，但速度不会发出去。"
  echo
fi
if [ "$RUN_MODE" = step ]; then
  echo " 页面上要做的（测试·单步）："
  echo "   1) 左上角确认模式是【① 测试·单步】"
  echo "   2) 点【① 推理一次】—— 只推一帧，速度仍然为零"
  echo "   3) 看提示框里的 vx/vy/wz 和模型输出，再决定："
  echo "        【② 确认执行】走这一小步（时长用上面的滑块调）"
  echo "        【跳过】丢弃；不管它，30~60 秒后也会自动丢弃"
  echo "   4) 重复 2~3 步。任何时候点【⛔ 一键终止】立即停下并归零"
  echo "   —— 不点【确认执行】就一直是零速，机器人不会自己动"
else
  echo " 页面上要做的（正式·连续）："
  if [ "$AUTO_CONFIRM" = 1 ]; then
    echo "   1) 已经自动放行，推理和下发现在就开始跑了"
  else
    echo "   1) 点【开始连续】放行 —— 不点就只在待机，不会动"
  fi
  echo "   2) 推理会一直连下去，直到模型说 stop=true（到目标了）"
  echo "   3) 想停：【暂停】只收回授权（推理继续）"
  echo "            【⛔ 一键终止】停推理 + 归零（推荐）"
fi
echo
print_tail_help() {
  echo " 看日志： tail -f $BRIDGE_LOG"
  echo " 看状态： ./start_vln.sh --status   （或 ./tools/vln_ctl.py state）"
  echo " 开关页面： ./start_vln.sh --web  |  ./start_vln.sh --web-off"
  echo " 命令行： ./tools/vln_ctl.py --help （网页按钮的命令行版）"
  echo " 停掉  ： ./start_vln.sh --stop"
}
print_tail_help
echo "════════════════════════════════════════════"
