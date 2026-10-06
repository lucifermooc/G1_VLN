#!/usr/bin/env bash
# ============================================================================
# sport_test.sh —— 真机速度链路分级手动测试
#
# 目的：在把 VLN 桥接接到真机之前，先手动确认
#       /cmd_vel_safe → cmdvel_to_sport → /api/sport/request → G1
#       这一条链每一步都是通的。
#
# 用法（**必须按顺序来，不要跳步**）：
#   ./tools/sport_test.sh probe              ① 只读体检，不动电机
#   ./tools/sport_test.sh up                 ② 起 cmdvel_to_sport（还不会动）
#   ./tools/sport_test.sh link               ③ 发一条【零速】验证链路（不会动）
#   ./tools/sport_test.sh go 0.20            ④ 走一小步（0.20 m/s × 0.5 s ≈ 10 cm）
#   ./tools/sport_test.sh turn 0.30          ⑤ 原地转一点（0.30 rad/s × 0.5 s ≈ 8.6°）
#   ./tools/sport_test.sh stop               ⑥ 连发零速（任何时候都能用）
#   ./tools/sport_test.sh down               ⑦ 停掉 cmdvel_to_sport
#
#   ./tools/sport_test.sh dur 1.0            改单次运动时长（默认 0.5 s）
#   ./tools/sport_test.sh watch              实时看底层 response（排错用）
#   ./tools/sport_test.sh topics             看话题订阅情况
#
# 安全设计
# ----------------------------------------------------------------------------
# * **只发一次 Twist**。cmdvel_to_sport 每次收到消息都调
#   SetVelocity(vx, vy, wz, duration)，duration（默认 0.5 s）走完**机器人自己停**。
#   所以就算这个脚本被 Ctrl-C 掉，机器人也只会走完那 0.5 秒。
#   —— 这是刻意的：不依赖「必须持续发零速」才停得住。
# * 发速度前会先检查 /cmd_vel_safe 有没有订阅者，没有就拒绝发（避免白测一场）。
# * 所有会动机器人的命令都有 5 秒倒计时 + 回车确认，另有 `--yes` 跳过。
# * 数值会被 clamp：|vx| ≤ 1.0、|vy| ≤ 0.5、|wz| ≤ 1.0。
#
# ⚠️ 前提
# * 机器人已经**站好**（不能是零力矩/软趴趴状态），周围留 1 m 空间。
# * 手里拿着遥控器或手机 App，随时能切回待机。
# * 一次只跑一个东西：确认 Nav2 / 别的手柄没有在同时下发速度。
# ============================================================================
set -uo pipefail

VLN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$VLN_DIR"
mkdir -p logs

CMD_TOPIC="/cmd_vel_safe"
SPORT_NODE="g1_cmdvel_to_sport"
SPORT_LOG="logs/sport.log"
SAFE_SOCK="${VLN_SAFE_LOCK:-/tmp/vln_sport_lock}"

# ---------------------------------------------------------------------------
# 环境（必须 source；ROS setup.bash 在 set -u 下会炸，所以临时关掉）
# ---------------------------------------------------------------------------
set +u
# shellcheck disable=SC1091
source "$VLN_DIR/setup_vln_ros2.sh" >/dev/null 2>&1
set -u

die() { echo "❌ $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
need_subscriber() {
  local n
  n="$(ros2 topic info "$CMD_TOPIC" 2>/dev/null \
        | awk -F': ' '/Subscription count/ {print $2}' | tr -d ' ')"
  if [ -z "$n" ]; then
    echo "❌ 话题 $CMD_TOPIC 不存在 —— 说明 cmdvel_to_sport 没起。"
    echo "   先跑： ./tools/sport_test.sh up"
    return 1
  fi
  if [ "$n" -eq 0 ]; then
    echo "❌ $CMD_TOPIC 有 0 个订阅者 —— 发了也没人收。"
    echo "   先跑： ./tools/sport_test.sh up"
    return 1
  fi
  return 0
}

# 发一条 Twist。$1=线性x $2=线性y $3=角速度z
# -1 发一次；-w 1 等发现到订阅者再发（否则 --once 会丢消息）
pub_twist() {
  local vx="$1" vy="$2" wz="$3"
  timeout 15 ros2 topic pub -1 -w 1 "$CMD_TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: $vx, y: $vy, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $wz}}" \
    >/dev/null 2>&1
  return $?
}

# clamp: $1=值 $2=上限
clamp() {
  awk -v v="$1" -v m="$2" 'BEGIN{
    if (v != v) { print "nan"; exit }        # NaN 检测
    if (v > m) v = m; if (v < -m) v = -m;
    printf "%.3f", v
  }'
}

# 倒计时确认。有 --yes 或 stdin 不是 tty 就跳过。
confirm_gate() {
  local desc="$1"
  if [ "$ASSUME_YES" = 1 ]; then
    echo "   (--yes：跳过确认)"
    return 0
  fi
  if [ ! -t 0 ]; then
    echo "   (非交互终端：跳过确认)"
    return 0
  fi
  echo
  echo "   ⚠️  $desc"
  echo "   机器人要动了。确认周围空间安全、手边有急停。"
  for i in 5 4 3 2 1; do
    printf "\r   倒计时 %d … （Ctrl-C 取消）" "$i"
    sleep 1
  done
  printf "\r%-50s\r" " "
  printf "   按【回车】执行，其它键 + 回车 取消： "
  local ans
  read -r ans
  if [ -n "$ans" ]; then
    echo "   已取消。"
    return 1
  fi
  return 0
}

show_sport_log() {
  echo
  echo "   ── cmdvel_to_sport 最近输出 ──"
  if [ -f "$SPORT_LOG" ]; then
    tail -6 "$SPORT_LOG" | sed 's/^/     /'
  else
    echo "     (还没有 $SPORT_LOG)"
  fi
}

# ---------------------------------------------------------------------------
# ① probe —— 只读体检
# ---------------------------------------------------------------------------
do_probe() {
  echo "════════════════════════════════════════════"
  echo " ① 只读体检（不会发任何速度）"
  echo "════════════════════════════════════════════"
  echo
  echo "── 机器人底层话题 ──"
  for t in /lowstate /api/sport/request /api/sport/response /sportmodestate; do
    printf "  %-22s " "$t"
    if ros2 topic list 2>/dev/null | grep -qx "$t"; then
      ros2 topic info "$t" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g'
      echo
    else
      echo "❌ 不存在"
    fi
  done

  echo
  echo "── 速度链路各段 ──"
  local seg
  for seg in /cmd_vel_nav /cmd_vel /cmd_vel_safe /vln/cmd_vel; do
    printf "  %-22s " "$seg"
    if ros2 topic list 2>/dev/null | grep -qx "$seg"; then
      ros2 topic info "$seg" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g'
      echo
    else
      echo "(不存在 —— 这一段还没接上)"
    fi
  done

  echo
  echo "── 关键进程 ──"
  for p in cmdvel_to_sport collision_monitor velocity_smoother controller_server \
           planner_server vln_ros2_bridge.py; do
    if pgrep -f "$p" >/dev/null 2>&1; then
      echo "  ✓ 运行中    $p"
    else
      echo "  · 没在跑    $p"
    fi
  done

  echo
  echo "── 结论提示 ──"
  if ros2 topic list 2>/dev/null | grep -qx "/api/sport/response"; then
    echo "  ✅ 底层运动接口活着（/api/sport/response 有人发）"
  else
    echo "  ❌ 看不到 /api/sport/response —— 底层服务可能没起，或 RMW 域不对"
  fi
  if pgrep -f "cmdvel_to_sport" >/dev/null 2>&1; then
    echo "  ✅ cmdvel_to_sport 在跑 → 可以进 ③ link"
  else
    echo "  ·  cmdvel_to_sport 没跑 → 下一步先跑 ② up"
  fi
}

# ---------------------------------------------------------------------------
# ② up / ⑦ down
# ---------------------------------------------------------------------------
do_up() {
  echo "═══ ② 启动 cmdvel_to_sport（启动本身不会让机器人动）═══"
  if [ -e "$SAFE_SOCK" ]; then :; fi
  if pgrep -f "cmdvel_to_sport" >/dev/null 2>&1; then
    echo "  · 已经在跑了（pid $(pgrep -f cmdvel_to_sport | head -1)），跳过"
  else
    : > "$SPORT_LOG"
    nohup ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
      -p cmd_vel_topic:="$CMD_TOPIC" -p duration:=0.5 \
      >> "$SPORT_LOG" 2>&1 &
    disown 2>/dev/null || true
    sleep 3
    if pgrep -f "cmdvel_to_sport" >/dev/null 2>&1; then
      echo "  ✓ 已启动（pid $(pgrep -f cmdvel_to_sport | head -1)）"
      echo "    日志 → $SPORT_LOG"
    else
      echo "  ❌ 启动失败，日志："
      tail -15 "$SPORT_LOG" | sed 's/^/     /'
      exit 1
    fi
  fi
  echo
  echo "  ── 确认它在订阅 $CMD_TOPIC ──"
  sleep 1
  ros2 topic info "$CMD_TOPIC" 2>/dev/null | sed 's/^/     /'
  need_subscriber || exit 1
  echo "  ✅ 订阅已建立"
  echo
  echo "  下一步： ./tools/sport_test.sh link"
}

do_down() {
  echo "═══ ⑦ 停掉 cmdvel_to_sport ═══"
  if ! pgrep -f "cmdvel_to_sport" >/dev/null 2>&1; then
    echo "  · 本来就没跑"
    return 0
  fi
  pkill -f "cmdvel_to_sport" 2>/dev/null
  for _ in 1 2 3 4 5 6; do
    pgrep -f "cmdvel_to_sport" >/dev/null 2>&1 || break
    sleep 0.5
  done
  if pgrep -f "cmdvel_to_sport" >/dev/null 2>&1; then
    pkill -9 -f "cmdvel_to_sport" 2>/dev/null
    sleep 0.5
  fi
  pgrep -f "cmdvel_to_sport" >/dev/null 2>&1 \
    && echo "  ⚠️ 仍有残留" || echo "  ✓ 已停"
}

# ---------------------------------------------------------------------------
# ③ link —— 零速，验证链路
# ---------------------------------------------------------------------------
do_link() {
  echo "═══ ③ 零速链路测试（机器人不会动）═══"
  need_subscriber || exit 1
  echo "  · 发一条 (0, 0, 0) 到 $CMD_TOPIC"
  pub_twist 0.0 0.0 0.0
  sleep 1.5
  show_sport_log
  echo
  echo "  判读："
  echo "    · 日志出现 「SetVelocity response: ... code=0」     → ✅ 链路全通"
  echo "    · 出现 「SetVelocity response: ... code=非0」        → ❌ 底层拒绝了，见 ⑥ 排错"
  echo "    · 出现 「SetVelocity failed: N」                     → ❌ 连不上运动服务"
  echo "    · 什么都没有                                        → 消息没到，看 topics"
  echo
  echo "  ⚠️ code=0 只说明「指令被接受」，不代表机器人力矩是开的。"
  echo "     机器人要是软趴趴（零力矩）状态，收到指令也不动。"
  echo
  echo "  下一步：机器人站好 → ./tools/sport_test.sh go 0.20"
}

# ---------------------------------------------------------------------------
# ④ go / ⑤ turn —— 真的动
# ---------------------------------------------------------------------------
do_move() {
  local vx="$1" vy="$2" wz="$3" what="$4"
  need_subscriber || exit 1

  local dur
  dur="$(ros2 param get "$SPORT_NODE" duration 2>/dev/null | awk '{print $NF}')"
  [ -n "$dur" ] && [ "$dur" != "not" ] || dur="0.5"

  echo "═══ $what ═══"
  echo "  vx=$vx m/s   vy=$vy m/s   wz=$wz rad/s   duration=$dur s"
  echo "  预计位移： 前 $(awk -v v="$vx" -v d="$dur" 'BEGIN{printf "%.1f", v*d*100}') cm" \
       " 转 $(awk -v w="$wz" -v d="$dur" 'BEGIN{printf "%.1f", w*d*57.2958}')°"
  echo

  confirm_gate "$what：机器人即将移动一小步。" || exit 0

  echo "  → 发送 …"
  pub_twist "$vx" "$vy" "$wz"
  local rc=$?

  # 等 duration 走完（＋余量），这是「机器人自己停」的核心
  echo "  → 等 $dur 秒（duration 到点机器人自停）"
  sleep "$(awk -v d="$dur" 'BEGIN{printf "%.2f", d + 0.4}')"

  # 再补一条零速，双保险
  echo "  → 补发零速"
  pub_twist 0.0 0.0 0.0
  sleep 0.6

  show_sport_log
  echo
  if [ "$rc" = 0 ]; then
    echo "  ✅ 命令已发出。**看机器人有没有真的动。**"
    echo "     没动的话依次查：机器人是否站立 / 是否零力矩 / 是否被别的遥控器接管"
  else
    echo "  ⚠️ ros2 topic pub 返回 $rc（超时或没找到订阅者）"
  fi
}

# ---------------------------------------------------------------------------
# ⑥ stop —— 连发零速
# ---------------------------------------------------------------------------
do_stop() {
  echo "═══ ⑥ 急停：连发零速 ═══"
  if ! ros2 topic list 2>/dev/null | grep -qx "$CMD_TOPIC"; then
    echo "  · $CMD_TOPIC 不存在，没什么可发的"
    return 0
  fi
  echo "  · 连发 10 条零速 …"
  for _ in $(seq 1 10); do
    pub_twist 0.0 0.0 0.0 >/dev/null 2>&1
  done
  echo "  ✓ 已发完"
  show_sport_log
}

# ---------------------------------------------------------------------------
# dur —— 改单次运动时长
# ---------------------------------------------------------------------------
do_dur() {
  local s="$1"
  need_subscriber || exit 1
  ros2 param set "$SPORT_NODE" duration "$s" 2>&1 | sed 's/^/  /'
  echo "  当前 duration = $(ros2 param get "$SPORT_NODE" duration 2>/dev/null | awk '{print $NF}') s"
  echo
  echo "  ⚠️ duration 越长，单条消息就让它走得越远 —— 同时「自停」的兜底时间也越长。"
}

# ---------------------------------------------------------------------------
# watch —— 实时看底层 response
# ---------------------------------------------------------------------------
do_watch() {
  echo "═══ 实时看 /api/sport/response（Ctrl-C 退出）═══"
  echo "  只显示 SetVelocity（api_id=7105）相关行"
  echo
  timeout 60 ros2 topic echo /api/sport/response 2>&1 \
    | grep -A3 -E "api_id: 7105|code" \
    | sed 's/^/  /'
}

do_topics() {
  echo "═══ 速度链路各段订阅情况 ═══"
  for seg in /cmd_vel_nav /cmd_vel /cmd_vel_safe; do
    printf "  %-16s " "$seg"
    if ros2 topic list 2>/dev/null | grep -qx "$seg"; then
      ros2 topic info "$seg" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g'
      echo
    else
      echo "(不存在)"
    fi
  done
}

# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
ASSUME_YES=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --yes|-y) ASSUME_YES=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- ${ARGS[@]+"${ARGS[@]}"}

CMD="${1:-help}"
shift 2>/dev/null || true

case "$CMD" in
  probe)   do_probe ;;
  up)      do_up ;;
  down)    do_down ;;
  link)    do_link ;;
  go)
    VX="$(clamp "${1:-0.20}" 1.0)"
    VY="$(clamp "${2:-0.0}" 0.5)"
    [ "$VX" = nan ] && die "vx 不是数字"
    do_move "$VX" "$VY" 0.0 "④ 直行一小步"
    ;;
  back)
    VX="$(clamp "${1:-0.15}" 1.0)"
    do_move "-$(echo "$VX" | tr -d -)" 0.0 0.0 "④b 后退一小步"
    ;;
  side)
    VY="$(clamp "${1:-0.15}" 0.5)"
    do_move 0.0 "$VY" 0.0 "④c 横移一小步"
    ;;
  turn)
    WZ="$(clamp "${1:-0.30}" 1.0)"
    do_move 0.0 0.0 "$WZ" "⑤ 原地转一点"
    ;;
  stop)    do_stop ;;
  dur)     do_dur "${1:-0.5}" ;;
  watch)   do_watch ;;
  topics)  do_topics ;;
  help|-h|--help|*)
    sed -n '2,45p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
