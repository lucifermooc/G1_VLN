#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LightNav-0 × ROS 2 桥接节点
================================================================================

把「ROS 2 相机取图」和「ROS 2 速度控制」接起来，替代 ``vln_robot_client.py`` 里的
pyrealsense2 直读 + unitree_sdk2py 直发两条私有路径：

    相机 (realsense2_camera)                      VLN 服务端
      /camera/camera/color/image_raw   ────────►  ws://192.168.110.204:8050
              │ sensor_msgs/Image                     │ actions[0]=[fwd,lat,yaw]
              ▼                                        ▼
        [本节点] 4 Hz 送图 ──────────────────────► 取回航点
              │                                        │
              │        waypoint_to_velocity()  ◄────────┘
              ▼ 20 Hz geometry_msgs/Twist
        /vln/cmd_vel   ← 默认话题，**不是** /cmd_vel
              │
              ▼  (需要你自己显式启动，本节点绝不代劳)
        g1_nav_bridge/cmdvel_to_sport → G1LocoClient
              → /api/sport/request → G1 本体

与仓库既有链路的关系
--------------------------------------------------------------------------------
`g1_nav_bridge/README.md` 定义的命令链是：

    Nav2 → /cmd_vel_nav → velocity_smoother → /cmd_vel
         → collision_monitor → /cmd_vel_safe → cmdvel_to_sport → G1

本节点默认输出到 **`/vln/cmd_vel`**，刻意不接进上面任何一环。要真正上真机时，
务必按 README 的正式链路把本节点的输出接到 `/cmd_vel_safe`（保留 collision_monitor
的碰撞停车保护），而不是像手工测试那样直接灌 `/cmd_vel` 绕过它。

⚠️ 安全默认（三层）
--------------------------------------------------------------------------------
1. **不传 `--publish` 就纯观察**：只推理、只打印，一条 Twist 都不发。
2. 即使 `--publish`，话题默认也是 `/vln/cmd_vel`。除非有人另起
   `cmdvel_to_sport` 订阅它，否则不会有速度到达机器人。
3. **控制定时器永远在跑**：指令超时（`--cmd-timeout`）或推理线程异常 → 归零；
   Ctrl-C 退出前连发若干帧零速。`cmdvel_to_sport` 的 `duration` 是第二道保险：
   话题断流后机器人端会在该有效期结束时自己停。

用法
--------------------------------------------------------------------------------
    # 0) 纯观察：只看推理结果和打算下发的速度，不发布（最安全）
    python3 vln_ros2_bridge.py --instruction "go to the red chair"

    # 1) 发布到安全话题 /vln/cmd_vel，用 ros2 topic echo 核对方向
    python3 vln_ros2_bridge.py --instruction "go to the red chair" --publish

    # 2) 参数也可用 ROS 风格覆盖（会盖掉命令行同名项）
    python3 vln_ros2_bridge.py --instruction "go" --ros-args -p max_vx:=0.3
"""

from __future__ import annotations

import json
import base64
import math
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

# 复用机器人端客户端的换算逻辑，保证与 --robot dry_run / unitree_g1 结果逐位一致
from vln_robot_client import (  # noqa: E402
    VelocityCmd,
    encode_jpeg_b64,
    log,
    waypoint_to_velocity,
)

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402


# ===========================================================================
# sensor_msgs/Image → HWC uint8 RGB
# ===========================================================================
# RealSense 的 color 流通常发 rgb8；UVC / 某些配置会发 bgr8。这里都兜住，
# 并显式处理 step（行对齐填充），避免 reshape 直接把图像拧了。
_ENCODINGS: dict[str, tuple[int, str]] = {
    "rgb8": (3, "rgb"),
    "bgr8": (3, "bgr"),
    "rgba8": (4, "rgba"),
    "bgra8": (4, "bgra"),
    "mono8": (1, "mono"),
    "8uc3": (3, "bgr"),
    "8uc1": (1, "mono"),
}


def image_to_rgb(msg: Image) -> np.ndarray:
    """把 `sensor_msgs/Image` 转成 HWC uint8 RGB 数组（不依赖 cv_bridge）。"""
    enc = (msg.encoding or "").strip()
    spec = _ENCODINGS.get(enc.lower())
    if spec is None:
        raise ValueError(
            f"不支持的图像编码 {enc!r}（支持 {sorted(_ENCODINGS)}）；"
            "可在 realsense launch 里设 color_format:=RGB8"
        )
    ch, kind = spec
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        raise ValueError(f"非法图像尺寸 {w}x{h}")

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    step = int(msg.step) or (w * ch)
    if step != w * ch:
        # 每行有填充，先按 step 切行再去掉尾巴
        need = step * h
        if buf.size < need:
            raise ValueError(f"数据长度不足：{buf.size} < {need}")
        buf = buf[:need].reshape(h, step)[:, : w * ch]
    else:
        need = w * ch * h
        if buf.size < need:
            raise ValueError(f"数据长度不足：{buf.size} < {need}")
        buf = buf[:need]

    arr = buf.reshape(h, w, ch)
    if kind == "rgb":
        out = arr
    elif kind == "bgr":
        out = arr[:, :, ::-1]
    elif kind == "rgba":
        out = arr[:, :, :3]
    elif kind == "bgra":
        out = arr[:, :, [2, 1, 0]]
    else:  # mono
        out = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(out)


# ===========================================================================
# 主节点
# ===========================================================================
class VlnRos2Bridge(Node):
    """ROS 2 相机进、ROS 2 Twist 出的 VLN 闭环节点。"""

    def __init__(self, a: Any) -> None:
        super().__init__("vln_ros2_bridge")
        self.a = self._apply_ros_param_overrides(a)

        # ---- 内部状态 ----
        self._img_lock = threading.Lock()
        self._latest_msg: Optional[Image] = None
        self._latest_t = 0.0
        self._n_img = 0
        # 最近若干帧的到达时刻，用于算实际帧率（见 _check_image_rate）
        self._t_img_hist: deque[float] = deque(maxlen=2000)
        self._fps_warned = False
        self._fps_bad = 0

        self._cmd_lock = threading.Lock()
        self._cmd = VelocityCmd()

        self._alive = True
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._worker_started = False
        self._shutting_down = False

        # ---- 人工批准闸门（armed）----
        # 所有下发动作的总闸：armed=False 时控制定时器恒发零速。
        # 只由网页/控制台的按钮改动，HTTP 线程只碰标志，不发速度。
        self._arm_lock = threading.Lock()
        self._arm_forever = False
        self._arm_deadline = 0.0
        self._estop = False
        # 默认一律「未授权」。
        # 以前是「没开网页就一直授权」，但启动拆成两步之后，
        # 第一步「环境 ready」正是一个「没有网页」的状态 —— 那时候如果
        # 默认已授权，状态面板会显示「授权 是」，非常容易让人误判成
        # 「随时会动」。现在只有显式 --start-armed 才一启动就授权。
        self._arm_forever = bool(self.a.start_armed)
        self._restarting = False

        # ---- 模式机：测试（单步）/ 正式（连续）----
        # 两种模式的根本区别是「推理节拍归谁管」：
        #   step: 人点【推理一次】才推一帧，推完挂起等【确认执行】
        #   auto: 自己按 --fps 连续推，推完直接写 self._cmd 下发
        # 闸门(armed)和节拍(mode)是两个正交的东西：前者管「发不发」，后者管「推不推」。
        self._mode: str = a.mode
        self._phase: str = "idle"
        self._step_index = 0
        self._step_exec_s = float(a.step_exec_s)
        self._pending_timeout_s = float(a.pending_timeout_s)

        # 待确认的那一步（仅 step 模式）。**确认之前绝不写 self._cmd。**
        self._pending_lock = threading.Lock()
        self._pending: Optional[dict[str, Any]] = None
        self._pending_t = 0.0

        # 线程间信号。用 Event 而不是轮询，一键终止时才能立刻打断等待。
        self._step_evt = threading.Event()        # 「推理一次」
        self._confirm_evt = threading.Event()     # 「确认执行」
        self._skip_evt = threading.Event()        # 「跳过这一步」
        self._auto_start_evt = threading.Event()  # 正式模式「开始连续」放行
        self._abort_evt = threading.Event()       # 「⛔ 一键终止」

        # ---- 预览通道（独立线程，与推理解耦）----
        self._preview: Optional[threading.Thread] = None
        self._preview_seq = 0

        # ---- 推理线程 → 网页 的单帧分发 ----
        # 网页可以运行期随时挂上/摘掉（见 _ensure_web / _stop_web），
        # 所以所有读写都在 _web_lock 下取一次引用再用，别长期持有。
        self._web: Optional[Any] = None
        self._web_lock = threading.Lock()
        self._web_port = int(self.a.web_port)

        # ---- 统计 ----
        self._n_pred = 0
        self._recv_error = 0
        self._t_infer: list[float] = []
        self._stats: list[dict[str, Any]] = []
        self._last_pred: dict[str, Any] = {}
        self._n_pub = 0
        self._n_zero = 0
        self._n_resized = 0
        self._latency_warned = False
        self._zero_sent = False
        self._infer_fatal: Optional[str] = None

        # ---- 订阅相机（传感器 QoS，否则拿不到 RealSense 的帧）----
        self.create_subscription(
            Image, self.a.image_topic, self._on_image, qos_profile_sensor_data
        )

        # ---- 发布器（不使能就干脆不建，杜绝误发）----
        self._pub = None
        if self.a.publish:
            self._pub = self.create_publisher(Twist, self.a.cmd_vel_topic, 10)

        # ---- MPC 跟踪（--ctrl mpc）----
        # 模型每次给 10 个航点（base_link，拍照时刻）。只用第 1 个航点会丢掉后段的转向/减速，
        # 所以把整条轨迹交给官方 vln_mpc（/vln 命名空间，用 /vln/odom 对齐到拍照时刻、10 Hz 跟踪），
        # 它算出的速度回到本节点，仍然经过下面同一套闸门（授权/急停/超时/停止）才发出去。
        self._mpc = self.a.ctrl == "mpc"
        self._follow = bool(self.a.follow)   # 运行中可由网页切换（见 set_follow）
        self._resume_lock = threading.Lock()  # 重置本轮串行化：连点「应用并重置」不会互相取消
        self._mpc_lock = threading.Lock()
        self._mpc_cmd = (0.0, 0.0)
        self._mpc_cmd_t = 0.0
        self._mpc_enabled_sent: Optional[bool] = None
        self._stop_streak = 0
        if self._mpc:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            enable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self._resp_pub = self.create_publisher(String, "/vln/response", 10)
            self._status_pub = self.create_publisher(String, "/vln/status", latched)
            self._mode_pub = self.create_publisher(String, "/vln/mode", latched)
            self._enable_pub = self.create_publisher(Bool, "/vln/mpc/enable", enable_qos)
            self.create_subscription(TwistStamped, "/vln/mpc/cmd_vel", self._on_mpc_cmd, 10)
            # follow：MPC 用 track 模式（忽略 stop，持续跟踪）；否则 objnav（stop 即停）
            self._mode_pub.publish(String(data="track" if self._follow else "objnav"))
            self._set_vln_status("IDLE")

        # ---- 控制定时器：无论推理是否正常，它都按固定节拍跑 ----
        self.create_timer(1.0 / self.a.control_hz, self._on_control)
        # ---- 收尾看门狗：推理线程一旦结束就退出 spin，否则节点会永远挂着 ----
        self.create_timer(0.5, self._on_exit_watchdog)
        # ---- 相机帧率看门狗：相机节点僵死时把原因喊出来（每 5 秒查一次）----
        self.create_timer(5.0, self._check_image_rate)

        # ---- 网页面板（有 --web 才起，失败不影响主流程）----
        if self.a.web:
            self._ensure_web(announce=False)

        # 「因为没网页就自动降级成 auto」这个旧行为已经删掉了。
        # 启动拆成两步之后，第一步（环境 ready）本来就没有网页 ——
        # 一旦降级成 auto 就会自己跑起来，那正是最危险的。
        # 现在没网页时就老老实实停在 idle，等 --web 挂上来，或用
        # ./tools/vln_ctl.py step 单点。
        if self._mode == "step" and self._web is None:
            log("ℹ️ 测试模式，但网页还没开 —— 不会自己动。")
            log("   开网页： ./start_vln.sh --web")
            log("   或者直接点： ./tools/vln_ctl.py step")

        # 正式模式的「开始连续」放行闸：**只看 --auto-confirm**，
        # 不再看「此刻有没有网页」—— 因为网页是随时可以后挂的，
        # 用「有没有网页」当闸门会出现「挂上网页瞬间开始跑」这种事故。
        self._auto_gate = not self.a.auto_confirm
        if not self._auto_gate:
            self._auto_start_evt.set()

        # ---- 本地控制通道（unix socket）----
        # 用途：网页还没起来 / 已经崩了的时候，从命令行把网页拉起来、拉下来，
        # 或者执行任何 /api/* 能做的事。协议见 _ctl_loop 的注释。
        self._ctl_srv: Optional[socket.socket] = None
        self._ctl_path = ""
        if self.a.ctl_socket:
            self._start_ctl_socket(self.a.ctl_socket)

        self._print_banner()

    # -- 参数 ---------------------------------------------------------------
    def _apply_ros_param_overrides(self, a: Any) -> Any:
        """声明 ROS 参数并把 argparse 的值作为默认值，`--ros-args -p` 可覆盖。"""
        spec: list[tuple[str, Any]] = [
            ("server", a.server),
            ("instruction", a.instruction),
            ("task", a.task),
            ("client_id", a.client_id),
            ("image_topic", a.image_topic),
            ("cmd_vel_topic", a.cmd_vel_topic),
            ("publish", bool(a.publish)),
            ("fps", float(a.fps)),
            ("control_hz", float(a.control_hz)),
            ("cmd_timeout", float(a.cmd_timeout)),
            ("jpeg_quality", int(a.jpeg_quality)),
            ("encode_width", int(a.encode_width)),
            ("max_vx", float(a.max_vx)),
            ("max_vy", float(a.max_vy)),
            ("max_wz", float(a.max_wz)),
            ("mapping", a.mapping),
            ("dt", float(a.dt)),
            ("yaw_sign", float(a.yaw_sign)),
            ("lat_sign", float(a.lat_sign)),
            ("warmup_frames", int(a.warmup_frames)),
            ("max_runtime", float(a.max_runtime)),
            ("max_errors", int(a.max_errors)),
            ("ignore_stop", bool(a.ignore_stop)),
            ("stop_hold_s", float(a.stop_hold_s)),
            ("max_image_age", float(a.max_image_age)),
            ("web", bool(a.web)),
            ("web_host", a.web_host),
            ("web_port", int(a.web_port)),
            ("start_armed", bool(a.start_armed)),
            ("approve_hold_s", float(a.approve_hold_s)),
            ("mode", a.mode),
            ("step_exec_s", float(a.step_exec_s)),
            ("pending_timeout_s", float(a.pending_timeout_s)),
            ("auto_confirm", bool(a.auto_confirm)),
            ("preview_hz", float(a.preview_hz)),
            ("preview_width", int(a.preview_width)),
        ]
        for name, default in spec:
            self.declare_parameter(name, default)
        for name, _ in spec:
            setattr(a, name, self.get_parameter(name).value)
        if not a.instruction:
            raise SystemExit("必须给指令：--instruction 'go to the red chair'")
        return a

    # =======================================================================
    # 网页面板的运行期启停
    # =======================================================================
    # 为什么需要「运行期」启停：启动被拆成两步 ——
    #   ① ./start_vln.sh            环境全部 ready（相机 + cmdvel_to_sport + 桥接）
    #   ② ./start_vln.sh --web      再把网页面板挂上去
    # 这样环境和服务端都确认 OK 了再开网页，出问题时也能单独把网页摘掉重挂，
    # 不用把整条闭环重启一遍（重启一次要重新 login/reset，还要等相机出图）。
    def _ensure_web(self, *, announce: bool = True) -> bool:
        """启动网页面板。已经在跑就返回 True。"""
        with self._web_lock:
            if self._web is not None:
                return True
            base = self._web_port or 8090
            for port in range(base, base + 12):
                try:
                    from vln_web_ui import WebUi

                    ui = WebUi(self, self.a.web_host, port)
                    ui.start()
                except Exception as e:  # 网页只是可选功能，绝不因此搞挂闭环
                    last = e
                    continue
                self._web = ui
                self._web_port = ui.port
                self._write_web_port_file(ui.port)
                if announce:
                    log("🖥️ 网页面板已启动")
                    for u in ui.urls():
                        log("   %s", u)
                return True
        log("❌ 网页面板启动失败：%s（闭环继续跑，不受影响）", last)
        return False

    def _stop_web(self) -> bool:
        """摘掉网页面板。没在跑就返回 False。"""
        with self._web_lock:
            ui = self._web
            self._web = None
        if ui is None:
            return False
        try:
            ui.stop()
        except Exception as e:
            log("⚠️ 关闭网页面板时报错：%s", e)
        return True

    @property
    def web_on(self) -> bool:
        return self._web is not None

    def _write_web_port_file(self, port: int) -> None:
        """把实际端口写到一个固定路径，让 start_vln.sh / vln_ctl.py 不用猜。"""
        try:
            Path("/tmp/vln_web_port").write_text(str(port), encoding="utf-8")
        except Exception:
            pass

    # =======================================================================
    # 本地控制通道（unix domain socket）
    # =======================================================================
    # 协议：一行一条请求，回一行 JSON。
    #   {"cmd":"web","body":{"on":true}}      或纯文本  web on
    #   {"cmd":"step"}                        或纯文本  step
    #   {"cmd":"state"}                       或纯文本  state
    # 响应用的就是 dispatch() 的返回值，和 HTTP 的 POST /api/* 完全一致。
    # 存在意义：网页没起来的时候，也得有个不依赖网页的办法去操作节点。
    def _start_ctl_socket(self, path: str) -> bool:
        try:
            p = Path(path)
            if p.exists():
                p.unlink()
            srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            srv.bind(path)
            srv.listen(8)
            srv.settimeout(0.5)
        except Exception as e:
            log("⚠️ 控制通道启动失败：%s（只影响命令行操作，不影响闭环）", e)
            return False
        self._ctl_srv = srv
        self._ctl_path = path
        threading.Thread(target=self._ctl_loop, daemon=True,
                         name="vln-ctl").start()
        log("控制通道 : %s（unix socket，用于网页启停 / 命令行操作）", path)
        return True

    def _ctl_loop(self) -> None:
        while self._alive and not self._stop.is_set():
            srv = self._ctl_srv
            if srv is None:
                return
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                conn.settimeout(3.0)
                data = b""
                while b"\n" not in data and len(data) < 65536:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                reply = self._handle_ctl_line(data.decode("utf-8", "replace"))
                conn.sendall((json.dumps(reply, ensure_ascii=False) + "\n")
                             .encode("utf-8"))
            except Exception as e:
                try:
                    conn.sendall((json.dumps({"ok": False, "error": str(e)},
                                             ensure_ascii=False) + "\n")
                                 .encode("utf-8"))
                except Exception:
                    pass
            finally:
                try:
                    conn.close()
                except Exception:
                    pass

    def _handle_ctl_line(self, line: str) -> dict[str, Any]:
        line = line.strip()
        if not line:
            return {"ok": False, "error": "空请求"}
        if line.startswith("{"):
            try:
                req = json.loads(line)
            except Exception as e:
                return {"ok": False, "error": f"JSON 解析失败：{e}"}
            return self.dispatch(str(req.get("cmd") or ""),
                                 req.get("body") or {})
        parts = line.split()
        cmd, body = parts[0], {}
        if cmd in ("web", "web_ctl"):
            arg = parts[1] if len(parts) > 1 else "status"
            body = {"action": arg}
        elif cmd in ("confirm", "exec-s", "timeout", "instruction") and len(parts) > 1:
            body = {"value": " ".join(parts[1:])}
        return self.dispatch(cmd, body)

    def _close_ctl_socket(self) -> None:
        srv = self._ctl_srv
        self._ctl_srv = None
        if srv is not None:
            try:
                srv.close()
            except Exception:
                pass
        if self._ctl_path:
            try:
                Path(self._ctl_path).unlink()
            except Exception:
                pass

    # =======================================================================
    # 统一命令派发：HTTP 和 unix socket 都走这里
    # =======================================================================
    def dispatch(self, cmd: str, body: dict[str, Any]) -> dict[str, Any]:
        """执行一条控制命令，返回 ``{"ok":…, "state":…}``。

        网页的 POST /api/* 和本地 unix socket 共用这一个入口，
        免得两条路各写一套、行为慢慢跑偏。
        """
        try:
            if cmd in ("state", "get_state"):
                return {"ok": True, "state": self.get_state()}

            # ---- 网页面板自身的启停（socket 专用，网页里不需要）----
            if cmd in ("web", "web_ctl"):
                action = str(body.get("action") or body.get("on") or "status")
                if action in ("on", "start", "true", "1"):
                    # 已经开着就别再喊一遍「网页面板已启动」——
                    # ./start_vln.sh --web 是幂等的，重复敲很常见。
                    ok = self._ensure_web(announce=not self.web_on)
                    return {"ok": ok, "web": self.web_on,
                            "port": self._web_port if ok else None}
                if action in ("off", "stop", "false", "0"):
                    self._stop_web()
                    return {"ok": True, "web": self.web_on}
                return {"ok": True, "web": self.web_on, "port": self._web_port}

            if cmd == "arm":
                hold = body.get("hold_s", 0)
                try:
                    hold = float(hold)
                except (TypeError, ValueError):
                    hold = 0.0
                self.arm(0.0 if hold == 0 else hold)
            elif cmd == "disarm":
                self.disarm("手动暂停")
            elif cmd == "estop":
                self.estop()
            elif cmd == "resume":
                self.resume_episode()
            elif cmd == "follow":
                self.set_follow(bool(body.get("on")))
                txt = str(body.get("instruction") or "").strip()
                if txt:
                    self.set_instruction(txt)
            elif cmd == "instruction":
                txt = str(body.get("instruction")
                          or body.get("value") or "").strip()
                if not txt:
                    return {"ok": False, "error": "instruction 为空"}
                self.set_instruction(txt)
            # ---- 测试模式：单步 ----
            elif cmd in ("step", "infer", "s"):
                self.request_step()
            elif cmd in ("confirm", "ok", "c"):
                v = body.get("step_exec_s")
                if v is None:
                    v = body.get("value")
                self.confirm_step(v)
            elif cmd in ("skip", "x"):
                self.skip_step()
            elif cmd in ("step_exec", "step-exec", "exec-s"):
                v = body.get("step_exec_s", body.get("value", 1.0))
                self.set_step_exec_s(v)
            elif cmd in ("pending_timeout", "timeout"):
                v = body.get("pending_timeout_s", body.get("value", 60.0))
                self.set_pending_timeout(v)
            # ---- 正式模式 ----
            elif cmd in ("start_auto", "go"):
                self.start_auto()
            elif cmd == "mode":
                m = str(body.get("mode") or body.get("value") or "")
                if m not in ("step", "auto"):
                    return {"ok": False, "error": f"未知模式 {m!r}"}
                self.set_mode(m)
            elif cmd in ("abort", "stop-all", "kill"):
                self.abort()
            else:
                return {"ok": False, "error": f"未知命令 {cmd!r}"}
        except Exception as e:
            log("⚠️ 控制命令 %s 执行出错：%s", cmd, e)
            return {"ok": False, "error": str(e)}
        return {"ok": True, "state": self.get_state()}

    def _print_banner(self) -> None:
        a = self.a
        log("═══ LightNav-0 × ROS 2 桥接 ═══")
        log("服务端   : %s", a.server)
        log("指令     : %s", a.instruction)
        log("相机话题 : %s", a.image_topic)
        if a.publish:
            log("速度话题 : %s   ✅ 已使能发布", a.cmd_vel_topic)
            if a.cmd_vel_topic.rstrip("/") in ("/cmd_vel", "/cmd_vel_safe"):
                log("  ⚠️ 注意：%s 是真实控制链路！确认 cmdvel_to_sport 是否在跑", a.cmd_vel_topic)
        else:
            log("速度话题 : (未使能) —— 纯观察模式，不会发布任何 Twist")
            log("           想发布请加 --publish，默认发到 %s", a.cmd_vel_topic)
        log("限速     : vx≤%.2f vy≤%.2f wz≤%.2f m/s  映射=%s",
            a.max_vx, a.max_vy, a.max_wz, a.mapping)
        log("节拍     : 推理 %.1f Hz / 下发 %.1f Hz / 超时归零 %.2f s",
            a.fps, a.control_hz, a.cmd_timeout)
        if a.encode_width > 0:
            log("送图尺寸 : 宽度缩到 %d px（保持宽高比）", a.encode_width)
        else:
            log("送图尺寸 : 原图不缩。若相机是 1280x720，服务端耗时会明显偏高")
            log("           建议加 --encode-width 640，或把相机改到 640x480x30")
        log("模式     : %s", "🧪 测试（单步：每步都要人确认）"
            if self._mode == "step" else "▶️ 正式（连续：直到 stop 或一键终止）")
        if self._mode == "step":
            log("           单步执行 %.2f s / 待确认超时自动丢弃 %s", self._step_exec_s,
                "不限" if self._pending_timeout_s <= 0
                else "%.0f s" % self._pending_timeout_s)
        elif self._auto_gate:
            log("           待放行：网页点【开始连续】，或 ./tools/vln_ctl.py go")
        log("画面预览 : %s", "%.1f Hz / 宽 %d px（独立线程，与推理解耦）"
            % (a.preview_hz, a.preview_width) if a.preview_hz > 0
            else "关（画面回退为推理帧）")
        log("一键终止 : 网页常驻按钮，或 POST /api/abort")

        if self._web is not None:
            log("网页面板 : ✅ 已启动")
            for u in self._web.urls():
                log("           %s", u)
            if self.is_armed():
                log("授权状态 : ⚠️ 启动即已授权（--start-armed），会直接开始下发")
            else:
                log("授权状态 : 未授权 —— 先看画面，确认后再点「执行」")
            log("快捷键   : 空格 暂停 / E 持续执行 / Esc 急停")
        elif a.web:
            log("网页面板 : ❌ 未启动（见上方错误），闭环不受影响")
        else:
            log("网页面板 : ⬜ 还没开（环境先 ready，这一步是故意的）")
            log("           开网页： ./start_vln.sh --web")
            log("           关网页： ./start_vln.sh --web-off")

        if a.control_hz < 4.0:
            log("  ⚠️ 下发 %.1f Hz 偏低；cmdvel_to_sport 的 duration 到期后机器人会自停",
                a.control_hz)

    # -- 相机回调 -----------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        """**只存原始 msg，不做任何转换。**

        这里曾经直接调用 image_to_rgb()，结果 1280x720 下单线程执行器被
        27 Hz × 2.7 MB 的转换拖满：BEST_EFFORT 队列(depth 5)不断丢帧，
        8 秒只收到 8 条图像消息，图像频繁报“过期”。
        转换改到 4 Hz 的推理线程里做，回调只贴一个引用，开销接近零。
        """
        with self._img_lock:
            self._latest_msg = msg
            self._latest_t = time.monotonic()
            self._t_img_hist.append(self._latest_t)
            self._n_img += 1
            if self._n_img == 1:
                log("首帧图像: %dx%d %s", msg.width, msg.height, msg.encoding)

    def _grab(self) -> Optional[np.ndarray]:
        """取最新一帧并转成 RGB；过期或还没有则返回 None。"""
        with self._img_lock:
            msg = self._latest_msg
            if msg is None or time.monotonic() - self._latest_t > self.a.max_image_age:
                return None
        try:
            return image_to_rgb(msg)
        except Exception as e:  # 编码不支持 / 数据异常，别把推理线程炸掉
            self.get_logger().warn(f"图像转换失败：{e}", throttle_duration_sec=5.0)
            return None

    def _grab_stamped(self) -> tuple[Optional[np.ndarray], int]:
        """同 _grab，另外返回这一帧的拍照时间戳（ns，header.stamp）。MPC 靠它对齐里程计。"""
        with self._img_lock:
            msg = self._latest_msg
            if msg is None or time.monotonic() - self._latest_t > self.a.max_image_age:
                return None, 0
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        try:
            return image_to_rgb(msg), stamp_ns
        except Exception as e:
            self.get_logger().warn(f"图像转换失败：{e}", throttle_duration_sec=5.0)
            return None, 0

    def _encode(self, frame: np.ndarray, width: int = 0, count: bool = True) -> str:
        """按需降采样后再 JPEG 编码。

        1280x720 的 JPEG 让服务端预处理/推理耗时飙到中位 289 ms、峰值 ~1 s
        （对比 640 宽度只要约 140 ms）。这不是网络问题，是像素量。
        宽度会**保持宽高比**：1280x720 → 640x360。

        ``width<=0`` 用 ``--encode-width``（推理路径）；预览路径显式传
        ``--preview-width`` 并 ``count=False``，免得把缩图计数混进来。
        """
        w_target = int(width) if width > 0 else int(self.a.encode_width)
        if w_target > 0 and frame.shape[1] > w_target:
            from PIL import Image as PILImage

            h_target = max(1, int(round(frame.shape[0] * w_target / frame.shape[1])))
            im = PILImage.fromarray(frame).resize((w_target, h_target), PILImage.BILINEAR)
            frame = np.asarray(im)
            if count:
                self._n_resized += 1
        return encode_jpeg_b64(frame, self.a.jpeg_quality)

    def _preview_loop(self) -> None:
        """独立预览通道：不管推理在不在跑，网页总有实时画面。

        为什么必须独立：推理是「送一帧 → 等服务端回包」的一问一答，单步模式下
        还要挂起等人点确认。若画面靠推理线程喂，那测试模式待确认时网页就是黑的，
        你根本没法「先看画面再决定」。所以预览自己按 --preview-hz 取图、缩图、编码。

        代价：多一路 JPEG 编码。480px 宽约 2~4 ms/帧，5 Hz 下占 CPU 可忽略。
        """
        a = self.a
        period = 1.0 / a.preview_hz
        t_acc = time.monotonic()
        while self._alive:
            t_acc += period
            # 没挂网页时整个循环空转：不抓图、不缩图、不编码，CPU 占用归零。
            # 网页是运行期后挂的，挂上之后下一个周期立刻就有画面，
            # 最多等 1/--preview-hz 秒 —— 这个延迟完全察觉不到。
            if self.web_on:
                frame = self._grab()
                if frame is not None:
                    try:
                        self._preview_seq += 1
                        self._feed_web(self._preview_seq,
                                       self._encode(frame, width=a.preview_width,
                                                    count=False))
                    except Exception:
                        pass  # 预览绝不能影响闭环
            dt = t_acc - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                t_acc = time.monotonic()

    def _feed_web(self, seq: int, b64: str) -> None:
        """把刚编码好的那份 JPEG 转发给浏览器（不重新编码）。"""
        # 网页可以在运行期被摘掉（--web-off），所以先取一次引用再判空。
        # Python 的引用赋值是原子的，这里拿到的要么是 None 要么是一个
        # 完整对象，不会拿到“一半的”实例。
        ui = self._web
        if ui is None:
            return
        try:
            ui.publish_frame(seq, base64.b64decode(b64))
        except Exception:
            pass  # 网页只是监视面板，绝不能影响闭环

    def _maybe_warn_latency(self, rtt: float) -> None:
        """rtt 接近 cmd_timeout 时预警 —— 超过就会归零，机器人会走走停停。"""
        if self._latency_warned or self._n_pred < 8:
            return
        arr = np.asarray(self._t_infer)
        p95 = float(np.percentile(arr, 95))
        if p95 > self.a.cmd_timeout * 0.8:
            self._latency_warned = True
            rec = max(1.0, round(p95 * 1.5, 1))
            self.get_logger().warn(
                f"往返 P95 {p95:.0f}ms 已接近/超过 cmd_timeout "
                f"{self.a.cmd_timeout:.2f}s，指令会在推理等待期内过期归零，"
                f"真机上表现为走走停停。建议二选一：\n"
                f"  1) 降分辨率（推荐，治本）：给相机加 rgb_camera.color_profile:=640x480x30，"
                f"或本节点 --encode-width 640\n"
                f"  2) 放宽超时：--cmd-timeout {rec} （但会削弱失速保护）"
            )

    def _check_image_rate(self) -> None:
        """相机帧率看门狗。

        实测踩过的坑：同事的 realsense launch 开了 `pointcloud.enable:=true`，
        但其 config 里的 `pointcloud__neon_.enable` 在当前 realsense2_camera 版本
        **已不再支持**（日志会警告 is not supported），点云退化成 CPU 计算，
        把整个相机节点拖到 0.8~3.4 fps（硬件直读仍有 27 fps，极易误诊为网络问题）。
        这里只要帧率低到不合理，就把排查方向直接写进日志。
        """
        now = time.monotonic()
        with self._img_lock:
            hist = list(self._t_img_hist)
        recent = [t for t in hist if now - t <= 5.0]
        fps = len(recent) / 5.0
        if fps >= 5.0:
            self._fps_warned = False
            self._fps_bad = 0
            return
        if self._fps_warned or (self._n_img == 0 and now < 5.0):
            return
        # 连续两个窗口都低才报：相机重启、DDS 瞬时抖动都会让单个窗口偏低，
        # 只凭一个 5 秒窗口就报错会把真正的故障淹在噪声里。
        self._fps_bad += 1
        if self._fps_bad < 2:
            return
        self._fps_warned = True
        if not hist:
            self.get_logger().error(
                f"5 秒内一帧都没收到（{self.a.image_topic}）。相机节点可能没起、"
                f"话题名不对，或已被别的进程占用（`Device or resource busy`）。"
            )
            return
        self.get_logger().error(
            f"相机帧率仅 {fps:.1f} fps（应在 30 左右），推理会大面积“图像过期”。\n"
            f"  先测硬件：如果 pyrealsense2 直读正常（约 27~30 fps），说明是 ROS 侧的问题；\n"
            f"  最常见原因：realsense launch 开了 pointcloud.enable:=true，而该版本的\n"
            f"  realsense2_camera 不支持 config 里的 pointcloud__neon_.enable，点云退化为\n"
            f"  CPU 计算，会把整个相机节点拖死。\n"
            f"  修复：重启相机并关掉点云 —— \n"
            f"    ros2 launch realsense2_camera rs_launch.py \\\n"
            f"      camera_namespace:=camera camera_name:=camera \\\n"
            f"      enable_color:=true enable_depth:=true pointcloud.enable:=false \\\n"
            f"      rgb_camera.color_profile:=640x480x30"
        )

    def _on_exit_watchdog(self) -> None:
        """推理线程结束时收尾。

        「推理线程结束」≠「这个进程该消失」，因为网页是随时能挂上摘下的，
        控制通道也一直开着 —— 进程一退，人就没法再用 resume 重开或挂网页看状态了。

        所以：只要「网页开着」或「控制通道开着」，就留着进程；
        两个都没有（纯脚本化场景，比如 --max-runtime 跑完、
        且显式 --ctl-socket ""），才真的退出。
        """
        if not self._worker_started or self._shutting_down or self._restarting:
            return
        if self._worker is None or self._worker.is_alive():
            return
        if self._web is not None:
            log("推理线程已结束；网页还开着，进程保留（可点【重新开始本轮】）")
            return
        if self._ctl_srv is not None:
            log("推理线程已结束；进程保留，继续用命令行操作：")
            log("   重开一轮： ./tools/vln_ctl.py resume")
            log("   挂上网页： ./start_vln.sh --web")
            log("   全部停掉： ./start_vln.sh --stop")
            return
        log("推理线程已结束，关闭节点")
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # -- 推理：会话建立 -----------------------------------------------------
    def _open_session(self):
        """连服务端 + login + reset + 等第一帧。返回连好的 ws；失败抛异常。"""
        a = self.a
        from websockets.sync.client import connect

        self._phase = "connecting"
        log("连接服务端 %s …", a.server)
        ws = connect(a.server, max_size=64 * 1024 * 1024,
                     open_timeout=15, close_timeout=3).__enter__()
        try:
            ws.send(json.dumps({"action": "login", "data": {"clientId": a.client_id}}))
            resp = json.loads(ws.recv())["data"]
            if resp.get("rc") != 0:
                raise SystemExit(f"login 失败：{resp}")
            log("login OK (clientId=%s)", a.client_id)

            ws.send(json.dumps({"action": "reset", "data": {}}))
            resp = json.loads(ws.recv())["data"]
            if resp.get("rc") != 0:
                raise SystemExit(f"reset 失败：{resp}")
            log("reset OK —— 新 episode 开始")

            log("等待 %s 的第一帧 …", a.image_topic)
            t_wait = time.monotonic()
            while self._alive and self._grab() is None:
                if time.monotonic() - t_wait > a.image_wait_s:
                    raise SystemExit(
                        f"{a.image_wait_s:.0f} s 内没收到图像。检查：\n"
                        f"  1) 相机是否在跑：ros2 topic hz {a.image_topic}\n"
                        f"  2) 话题名是否对（默认 /camera/camera/color/image_raw）\n"
                        f"  3) 相机是否插上、realsense 节点是否起来"
                    )
                time.sleep(0.1)
        except BaseException:
            try:
                ws.__exit__(None, None, None)  # 建会话失败就别把连接漏在这儿
            except Exception:
                pass
            raise
        if not self._alive:
            ws.__exit__(None, None, None)
            return None
        log("收到图像，开始闭环")
        return ws

    # -- 推理线程 -----------------------------------------------------------
    def _inference_loop(self) -> None:
        """会话建好后，按当前模式分派到 step（单步）或 auto（连续）。"""
        ws = None
        try:
            ws = self._open_session()
            if ws is None:
                return
            self._set_vln_status("RUNNING")
            if self._mode == "step":
                self._run_step_mode(ws)
            else:
                self._run_auto_mode(ws)
        except SystemExit as e:
            self._infer_fatal = str(e)
            log("推理线程退出：%s", e)
            self._stop.set()
        except Exception as e:
            self._infer_fatal = f"{type(e).__name__}: {e}"
            log("推理线程异常：%s", self._infer_fatal)
            self._stop.set()
        finally:
            self._phase = "ended"
            self._set_vln_status("IDLE")
            with self._pending_lock:
                self._pending = None       # 线程都退了，挂起的待确认也就作废
            if ws is not None:
                try:
                    ws.__exit__(None, None, None)
                except Exception:
                    pass
            # 停车帧必须在这里发：此刻推理线程已停、rclpy 上下文仍然有效。
            # 若放到外层的 shutdown()/看门狗里，上下文可能已被关闭，
            # publish 会报 "publisher's context is invalid"，真机上就是没刹车。
            self._send_zero_burst()

    # -- 回包解析 / 记账（step 与 auto 共用）--------------------------------
    def _parse_actions(self, data: dict[str, Any], rtt: float) -> dict[str, Any]:
        """把服务端回包解析成一条统一记录。速度换算只在这里做一次。"""
        a = self.a
        wp0 = [float(x) for x in data["actions"]["actions"][0]]
        vx, vy, wz = waypoint_to_velocity(
            np.asarray(wp0, dtype=np.float64),
            mapping=a.mapping, dt=a.dt,
            max_vx=a.max_vx, max_vy=a.max_vy, max_wz=a.max_wz,
            yaw_sign=a.yaw_sign, lat_sign=a.lat_sign,
        )
        return {
            "seq": self._n_pred,
            "step": int(data["actions"].get("step", -1)),
            "wp0": wp0,
            "waypoints": [[float(v) for v in w] for w in data["actions"]["actions"]],
            "capture_stamp_ns": 0,
            "raw_text": data.get("raw_text", ""),
            "pointing": data.get("pointing"),
            "stop": bool(data.get("stop", False)),
            "visible": data.get("visible"),
            "rtt_ms": round(rtt, 1),
            "server_ms": round(float(data.get("latency_ms", 0.0)), 1),
            "cmd": {"vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4)},
            "ts": time.time(),
        }

    def _record(self, rec: dict[str, Any]) -> None:
        """写入统计、最近列表与网页快照（不含「是否已下发」的语义）。"""
        c = rec["cmd"]
        self._stats.append({
            "seq": rec["seq"], "step": rec["step"], "rtt_ms": rec["rtt_ms"],
            "server_ms": rec["server_ms"], "vx": c["vx"], "vy": c["vy"], "wz": c["wz"],
            "wp0": rec["wp0"], "stop": rec["stop"], "visible": rec["visible"],
            "raw_text": rec["raw_text"], "pointing": rec["pointing"],
        })
        self._last_pred = {
            "seq": rec["seq"], "step": rec["step"], "wp0": rec["wp0"],
            "raw_text": rec["raw_text"], "pointing": rec["pointing"],
            "stop": rec["stop"], "visible": rec["visible"],
            "rtt_ms": rec["rtt_ms"], "server_ms": rec["server_ms"],
            "cmd": c, "ts": time.time(),
        }

    def _log_pred(self, rec: dict[str, Any], tag: str = "") -> None:
        """打一行预测日志。tag 用来说明这一步「下发 / 拟下发 / 待确认」。"""
        c = rec["cmd"]
        log("seq=%3d  rtt=%6.1fms  srv=%6.1fms  wp0=[%+.3f %+.3f %+.3f]  "
            "cmd=(vx%+.3f vy%+.3f wz%+.3f)  stop=%s vis=%s  → %s",
            rec["seq"], rec["rtt_ms"], rec["server_ms"],
            rec["wp0"][0], rec["wp0"][1], rec["wp0"][2],
            c["vx"], c["vy"], c["wz"], rec["stop"], rec["visible"],
            tag or ("下发" if self.a.publish else "拟下发"))
        pt = rec["pointing"] or {}
        if pt.get("opos_state") or pt.get("apos_state"):
            log("       apos=%s%s  opos=%s%s",
                pt.get("apos_state"), " (clamped)" if pt.get("apos_clamped") else "",
                pt.get("opos_state"), " (clamped)" if pt.get("opos_clamped") else "")

    # -- 正式模式：连续推理 + 连续下发 ---------------------------------------
    def _run_auto_mode(self, ws) -> None:
        a = self.a

        # 预热历史窗：先塞纯图像帧进服务端缓冲，不带 instruction
        if a.warmup_frames > 0:
            frame = self._grab()
            if frame is not None:
                log("预热历史窗 %d 帧（仅入缓冲，不推理）…", a.warmup_frames)
                for _ in range(a.warmup_frames):
                    ws.send(json.dumps({"action": "next", "data": {
                        "seq": self._n_pred, "instruction": "",
                        "image": self._encode(frame),
                    }}))
                    ws.recv()

        # 人工放行闸：默认要先在网页点【开始连续】，免得误切模式就冲出去
        if self._auto_gate and not self._auto_start_evt.is_set():
            self._phase = "wait_go"
            log("⏸️ 正式模式待放行：在网页点【开始连续】后才开始推理")
            while self._alive and not self._stop.is_set():
                if self._abort_evt.is_set():
                    log("⛔ 放行前被终止")
                    return
                if self._auto_start_evt.wait(0.1):
                    break
            if not self._alive or self._stop.is_set():
                return

        self._phase = "auto"
        log("▶️ 正式模式：连续推理开始")

        period = 1.0 / a.fps
        next_t = time.monotonic()
        t_start = time.monotonic()

        while self._alive and not self._stop.is_set():
            if self._abort_evt.is_set():
                log("⛔ 收到终止信号，停止推理")
                return
            next_t += period
            if a.max_runtime > 0 and time.monotonic() - t_start > a.max_runtime:
                log("达到 --max-runtime %.0fs，正常结束", a.max_runtime)
                return

            frame, stamp_ns = self._grab_stamped()
            if frame is None:
                self.get_logger().warn(
                    f"图像过期/未到达（{a.image_topic}）", throttle_duration_sec=2.0
                )
                time.sleep(0.05)
                continue

            t0 = time.monotonic()
            payload = self._encode(frame)
            ws.send(json.dumps({"action": "next", "data": {
                "seq": self._n_pred, "image": payload, "instruction": a.instruction,
            }}))
            data = json.loads(ws.recv())["data"]
            rtt = (time.monotonic() - t0) * 1000.0

            if data.get("rc") != 0:
                self._recv_error += 1
                log("服务端返回错误 rc=%s msg=%s（沿用上一条指令）",
                    data.get("rc"), data.get("msg"))
                if self._recv_error > a.max_errors:
                    raise SystemExit(f"连续错误超过 {a.max_errors} 次，退出")
                continue
            if "actions" not in data:
                continue  # 预热帧的回包

            self._recv_error = 0
            rec = self._parse_actions(data, rtt)
            rec["capture_stamp_ns"] = stamp_ns
            stop_now = self._confirmed_stop(rec)
            c = rec["cmd"]
            # ws.recv() 可能阻塞几百毫秒，终止信号就是在这一瞬间到的。
            # 不检查的话，终止后还会多冲出去一帧速度。
            if self._abort_evt.is_set() or not self._alive or self._stop.is_set():
                log("⛔ 终止信号到达，丢弃刚拿到的这一帧（不下发）")
                return
            if not (rec["stop"] and not stop_now):
                # 未确认的单帧 stop 不覆盖上一条指令（航点全零），等下一帧再判
                with self._cmd_lock:
                    self._cmd = VelocityCmd(c["vx"], c["vy"], c["wz"],
                                            time.monotonic(), rec["step"], stop_now)
                self._publish_response(rec, stop_now)
            self._n_pred += 1
            self._t_infer.append(rtt)
            self._maybe_warn_latency(rtt)
            self._record(rec)
            self._log_pred(rec)

            if stop_now:
                log("★ 模型连续 %d 帧判定到达目标（stop=true），停下并结束", a.stop_confirm)
                with self._cmd_lock:
                    self._cmd = VelocityCmd(0.0, 0.0, 0.0, time.monotonic(),
                                            rec["step"], True)
                time.sleep(a.stop_hold_s)
                return

            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    # -- 测试模式：推一次 → 等人确认 → 走一小步 -----------------------------
    def _run_step_mode(self, ws) -> None:
        """单步闭环。关键在于**确认之前绝不写 self._cmd**，速度保持为零。

        注意这仍然是一个 episode：login/reset 建的是有记忆的会话，
        单步只是把「推理节拍」交给人工，而不是每步重开一轮。
        """
        a = self.a
        log("🧪 测试模式：每步都要在网页点【推理一次】→【确认执行】")

        while self._alive and not self._stop.is_set():
            if self._abort_evt.is_set():
                return

            # ---- 1) 等人点【推理一次】----
            self._phase = "idle"
            if not self._wait_for_step():
                return

            # ---- 2) 推一次 ----
            self._phase = "inferring"
            frame, stamp_ns = self._grab_stamped()
            if frame is None:
                log("⚠️ 取不到图像，本次推理取消（检查相机）")
                continue

            t0 = time.monotonic()
            payload = self._encode(frame)
            ws.send(json.dumps({"action": "next", "data": {
                "seq": self._n_pred, "image": payload, "instruction": a.instruction,
            }}))
            data = json.loads(ws.recv())["data"]
            rtt = (time.monotonic() - t0) * 1000.0

            if data.get("rc") != 0:
                self._recv_error += 1
                log("服务端返回错误 rc=%s msg=%s", data.get("rc"), data.get("msg"))
                if self._recv_error > a.max_errors:
                    raise SystemExit(f"连续错误超过 {a.max_errors} 次，退出")
                continue
            if "actions" not in data:
                continue

            self._recv_error = 0
            rec = self._parse_actions(data, rtt)
            rec["capture_stamp_ns"] = stamp_ns
            self._n_pred += 1
            self._t_infer.append(rtt)
            self._maybe_warn_latency(rtt)
            self._record(rec)
            self._log_pred(rec, tag="待确认")

            # ---- 3) 挂起等人裁决（超时则自动丢弃）----
            with self._pending_lock:
                self._pending = rec
                self._pending_t = time.monotonic()
            self._phase = "pending"

            verdict = self._wait_for_decision()
            with self._pending_lock:
                self._pending = None

            if verdict == "abort":
                log("⛔ 待确认期间被终止")
                return
            if verdict == "timeout":
                log("⏱️ 待确认超时 %.0f s，自动丢弃这一步（没执行）",
                    self._pending_timeout_s)
                continue
            if verdict == "skip":
                log("⏭️ 已跳过这一步（没执行）")
                continue

            # ---- 4) 确认 → 走一小步 ----
            if rec["stop"] and not a.ignore_stop and not self._follow:
                log("★ 模型判定到达目标（stop=true）—— 不执行，请决定是否重开一轮")
                continue

            exec_s = self._step_exec_s
            c = rec["cmd"]
            self._phase = "executing"
            with self._cmd_lock:
                self._cmd = VelocityCmd(c["vx"], c["vy"], c["wz"],
                                        time.monotonic(), rec["step"], False)
            # MPC 只在确认后才拿到这条轨迹（里程计历史约 20 s，挂起太久会对不上，MPC 就不出速度）
            self._publish_response(rec, False)
            self.arm(exec_s)
            log("✅ 执行第 %d 步：%.2f s × (vx%+.3f vy%+.3f wz%+.3f)",
                self._step_index + 1, exec_s, c["vx"], c["vy"], c["wz"])
            ok = self._wait_exec(exec_s)
            self.disarm("单步结束" if ok else "被中断")
            if not ok:
                log("⛔ 单步执行被中断")
                return
            self._step_index += 1

    # -- 单步模式的等待原语 -------------------------------------------------
    def _wait_for_step(self) -> bool:
        """等「推理一次」信号。返回 False = 该退出线程了。"""
        while self._alive and not self._stop.is_set():
            if self._abort_evt.is_set():
                return False
            if self._step_evt.wait(0.1):
                self._step_evt.clear()
                return True
        return False

    def _wait_for_decision(self) -> str:
        """等人工裁决。返回 confirm / skip / timeout / abort。

        终止优先于确认（先查 abort），免得终止路上还执行一步。
        """
        t0 = time.monotonic()
        while self._alive and not self._stop.is_set():
            if self._abort_evt.is_set():
                return "abort"
            if self._confirm_evt.is_set():
                self._confirm_evt.clear()
                return "confirm"
            if self._skip_evt.is_set():
                self._skip_evt.clear()
                return "skip"
            if (self._pending_timeout_s > 0
                    and time.monotonic() - t0 > self._pending_timeout_s):
                return "timeout"
            time.sleep(0.05)
        return "abort"

    def _wait_exec(self, dur: float) -> bool:
        """等这一小步走完。返回 False = 被打断（速度已由 disarm 归零）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            if not self._alive or self._stop.is_set() or self._abort_evt.is_set():
                return False
            time.sleep(0.04)
        return True

    # -- 控制定时器 ---------------------------------------------------------
    def _on_control(self) -> None:
        """固定节拍下发。任何异常/超时/未授权/暂停 → 归零。

        顺序很重要：先看安全条件，最后才看人工闸门。任何一条不满足就只发零。
        """
        with self._cmd_lock:
            c = self._cmd
        armed = self.is_armed()
        if self._mpc:
            self._sync_mpc_enable(armed and not self._stop.is_set() and not self._estop)
        # 单步模式下一条指令要撑满整个执行时长；连续模式按 cmd_timeout 判断推理是否断流
        timeout = self.a.cmd_timeout
        if self._mpc and self._mode == "step":
            timeout = max(timeout, self._step_exec_s + 0.2)
        age = time.monotonic() - c.ts
        if (self._stop.is_set() or self._estop or c.stop
                or age > timeout or not armed):
            self._publish(0.0, 0.0, 0.0, zero=True)
        elif self._mpc:
            with self._mpc_lock:
                (vx, wz), t = self._mpc_cmd, self._mpc_cmd_t
            if time.monotonic() - t > self.a.mpc_cmd_timeout:
                self._publish(0.0, 0.0, 0.0, zero=True)   # MPC 没出解 / 里程计没对上
            else:
                self._publish(min(max(vx, 0.0), self.a.max_vx), 0.0,
                              min(max(wz, -self.a.max_wz), self.a.max_wz), zero=False)
        else:
            self._publish(c.vx, c.vy, c.wz, zero=False)

    # -- MPC 接口 -----------------------------------------------------------
    def _on_mpc_cmd(self, msg: TwistStamped) -> None:
        with self._mpc_lock:
            self._mpc_cmd = (float(msg.twist.linear.x), float(msg.twist.angular.z))
            self._mpc_cmd_t = time.monotonic()

    def _set_vln_status(self, state: str) -> None:
        if self._mpc and rclpy.ok():
            try:
                self._status_pub.publish(String(data=state))
            except Exception:
                pass

    def _sync_mpc_enable(self, on: bool) -> None:
        if on == self._mpc_enabled_sent:
            return
        self._mpc_enabled_sent = on
        with self._mpc_lock:
            self._mpc_cmd_t = 0.0      # 旧解作废
        try:
            self._enable_pub.publish(Bool(data=on))
        except Exception:
            pass

    def _confirmed_stop(self, rec: dict[str, Any]) -> bool:
        """stop 要连续 --stop-confirm 帧才算到达（单帧误判不结束本轮）。"""
        if self.a.ignore_stop or self._follow:
            return False
        self._stop_streak = self._stop_streak + 1 if rec["stop"] else 0
        return self._stop_streak >= max(1, self.a.stop_confirm)

    def _publish_response(self, rec: dict[str, Any], stop: bool) -> None:
        if not self._mpc or not rec.get("capture_stamp_ns"):
            return
        self._resp_pub.publish(String(data=json.dumps({
            "episode": 1,                 # vln_mpc 要求 episode/seq 为正整数
            "seq": int(rec["seq"]) + 1,
            "capture_stamp_ns": int(rec["capture_stamp_ns"]),
            "frame_id": "base_link",
            "waypoints": rec["waypoints"],
            "stop": bool(stop),
            "visible": bool(rec["visible"]) if rec["visible"] is not None else None,
        }, separators=(",", ":"))))

    # -- 人工批准闸门 -------------------------------------------------------
    def is_armed(self) -> bool:
        """当前是否已授权下发。"""
        if self._estop or self._shutting_down:
            return False
        with self._arm_lock:
            if self._arm_forever:
                return True
            return time.monotonic() < self._arm_deadline

    def arm_remaining(self) -> float:
        """单次授权还剩多少秒（持续授权返回 0）。"""
        with self._arm_lock:
            if self._arm_forever:
                return 0.0
            return max(0.0, self._arm_deadline - time.monotonic())

    def arm(self, hold_s: float = 0.0) -> None:
        """授权下发。``hold_s <= 0`` = 持续；``> 0`` = 单次，到期自动收回。"""
        if self._estop:
            log("⚠️ 已急停，拒绝授权。请先「重新开始本轮」")
            return
        with self._arm_lock:
            if hold_s and hold_s > 0:
                self._arm_forever = False
                self._arm_deadline = time.monotonic() + hold_s
            elif hold_s is not None and hold_s < 0:
                self._arm_forever = True
                self._arm_deadline = 0.0
            else:
                # 0 = 用命令行给的默认单次时长
                self._arm_forever = False
                self._arm_deadline = time.monotonic() + max(0.05, self.a.approve_hold_s)
        log("✅ 已授权执行%s",
            "（持续，直到暂停/急停）" if self._arm_forever else f"（{self.arm_remaining():.1f} s 后自动收回）")

    def disarm(self, why: str = "手动暂停") -> None:
        """收回授权。只改标志 —— 实际归零由 20 Hz 控制定时器完成。"""
        with self._arm_lock:
            was = self._arm_forever or time.monotonic() < self._arm_deadline
            self._arm_forever = False
            self._arm_deadline = 0.0
        if was:
            log("⏸️ 已收回授权（%s）→ 开始发零速", why)

    def estop(self) -> None:
        """急停：收回授权 + 结束本轮。控制定时器会在 50 ms 内发零速。"""
        log("🛑 急停！收回授权并结束本轮")
        self.disarm("急停")
        self._abort_evt.set()   # 唤醒单步/放行的等待，别让人干等
        with self._pending_lock:
            self._pending = None
        self._estop = True
        self._stop.set()  # 让推理线程退出

    # -- 模式与单步控制（网页调用）------------------------------------------
    def request_step(self) -> None:
        """测试模式：请求推理一次，结果会挂成「待确认」。"""
        if self._mode != "step":
            log("⚠️ 当前是正式模式，【推理一次】无效")
            return
        with self._pending_lock:
            if self._pending is not None:
                log("⚠️ 上一步还没确认，忽略本次【推理一次】")
                return
        self._step_evt.set()

    def confirm_step(self, exec_s: Optional[float] = None) -> None:
        """测试模式：确认执行待确认的那一步（可同时改时长）。"""
        if exec_s is not None:
            self.set_step_exec_s(exec_s)
        with self._pending_lock:
            has = self._pending is not None
        if not has:
            log("⚠️ 没有待确认的步骤，先点【推理一次】")
            return
        self._confirm_evt.set()

    def skip_step(self) -> None:
        """测试模式：丢弃待确认的那一步（不执行）。"""
        with self._pending_lock:
            if self._pending is None:
                log("⚠️ 没有待跳过的步骤")
                return
        log("⏭️ 收到跳过指令")
        self._skip_evt.set()

    def set_step_exec_s(self, s: float) -> None:
        """单步执行时长，网页上可随时调。夹在 0.2~10 s 之间。"""
        try:
            s = float(s)
        except (TypeError, ValueError):
            return
        self._step_exec_s = max(0.2, min(10.0, s))
        log("⚙️ 单步执行时长 → %.2f s（约 %.0f cm/步）",
            self._step_exec_s, self._step_exec_s * self.a.max_vx * 100)

    def set_pending_timeout(self, s: float) -> None:
        """待确认自动丢弃时长。0 = 不限。"""
        try:
            s = float(s)
        except (TypeError, ValueError):
            return
        self._pending_timeout_s = max(0.0, min(600.0, s))
        log("⚙️ 待确认自动丢弃 → %s", "不限" if self._pending_timeout_s <= 0
            else "%.0f s" % self._pending_timeout_s)

    def set_mode(self, mode: str) -> None:
        """切换测试/正式模式。节拍语义变了，所以要重置本轮。"""
        if mode not in ("step", "auto"):
            log("⚠️ 未知模式 %r", mode)
            return
        if mode == self._mode:
            return
        log("🔀 切换模式：%s → %s（重置本轮）", self._mode, mode)
        self._mode = mode
        self.resume_episode()

    def start_auto(self) -> None:
        """正式模式：人工放行后开始连续执行。"""
        if self._mode != "auto":
            log("⚠️ 当前不是正式模式，【开始连续】无效")
            return
        self.arm(-1)                 # 先授权，再放行推理
        self._auto_start_evt.set()
        log("▶️ 正式模式：已放行，开始连续执行（点【暂停】可随时停下）")

    def abort(self) -> None:
        """⛔ 一键终止：立刻归零 + 停推理 + 唤醒所有等待。急停的超集。"""
        log("⛔ 一键终止：归零 + 停止一切推理与下发")
        self.estop()
        with self._cmd_lock:
            self._cmd = VelocityCmd()      # 立刻清零，不等下一拍定时器
        for _ in range(3):                 # 不走 _zero_sent，每次终止都真发
            self._publish(0.0, 0.0, 0.0, zero=True)
            time.sleep(0.02)
        log("⛔ 已终止。画面仍在预览，点【重新开始本轮】可再来一轮")

    def set_follow(self, on: bool) -> None:
        """切换「跟随 / 去目标」。跟随：MPC 用 track、忽略 stop（跟人没有终点）；去目标：objnav，stop 即停。"""
        on = bool(on)
        if on == self._follow:
            return
        self._follow = on
        self._stop_streak = 0
        log("🏃 %s", "跟随模式：忽略 stop，持续跟踪" if on else "去目标模式：stop 即停")
        if self._mpc and rclpy.ok():
            try:
                self._mode_pub.publish(String(data="track" if on else "objnav"))
            except Exception:
                pass

    def set_instruction(self, text: str) -> None:
        """换指令。语义变了，所以顺带重置 episode。"""
        text = text.strip()
        if not text:
            return
        log("📝 新指令：%s（重置 episode）", text)
        self.a.instruction = text
        self.resume_episode()

    def resume_episode(self) -> None:
        """停掉当前推理线程，用当前指令开一轮新的（网页「重新开始本轮」）。"""
        with self._resume_lock:
            self._resume_episode_locked()

    def _resume_episode_locked(self) -> None:
        if self._shutting_down:
            return
        self.disarm("重置 episode")
        self._estop = False
        # 清掉上一轮遗留的全部信号与挂起状态，免得新一轮一启动就误执行
        self._abort_evt.clear()
        self._step_evt.clear()
        self._confirm_evt.clear()
        self._skip_evt.clear()
        with self._pending_lock:
            self._pending = None
            self._pending_t = 0.0
        with self._cmd_lock:
            self._cmd = VelocityCmd()
        self._step_index = 0
        self._phase = "idle"
        # 正式模式未 --auto-confirm 时，新一轮要重新等人放行
        if self._auto_gate:
            self._auto_start_evt.clear()
        self._restarting = True
        try:
            if self._worker is not None and self._worker.is_alive():
                self._stop.set()
                self._worker.join(timeout=10.0)
            # 停不下来就别硬开第二个线程
            if self._worker is not None and self._worker.is_alive():
                log("⚠️ 旧推理线程未退出，取消重置")
                return
            self._stop.clear()
            self._zero_sent = False
            self._infer_fatal = None
            self._worker = threading.Thread(target=self._inference_loop, daemon=True)
            self._worker.start()
            log("🔄 已开始新一轮 episode（%s 模式）",
                "测试·单步" if self._mode == "step" else "正式·连续")
        finally:
            self._restarting = False

    def get_state(self) -> dict[str, Any]:
        """给网页的实时快照。只读，不加锁地拿几个标量（Python 下足够安全）。"""
        c = self._cmd
        with self._img_lock:
            img_t = self._latest_t
            n_img = self._n_img
        img_age = None if n_img == 0 else max(0.0, time.monotonic() - img_t)
        armed = self.is_armed()
        with self._pending_lock:
            pending = dict(self._pending) if self._pending is not None else None
            pend_age = (None if pending is None
                        else round(max(0.0, time.monotonic() - self._pending_t), 1))
        hist = [
            {"seq": r["seq"], "vx": r["vx"], "wz": r["wz"],
             "rtt_ms": r["rtt_ms"], "stop": r["stop"]}
            for r in self._stats[-20:]
        ]
        return {
            "inference": dict(self._last_pred),
            "pending": pending,          # 测试模式待确认的那一步（未确认前速度为零）
            "state": {
                "armed": armed,
                "arm_forever": self._arm_forever,
                "arm_remaining_s": round(self.arm_remaining(), 2),
                "estop": self._estop,
                "publishing": armed and not self._stop.is_set()
                              and (time.monotonic() - c.ts) <= self.a.cmd_timeout,
                "cmd_age_s": round(max(0.0, time.monotonic() - c.ts), 3),
                "publish_enabled": self.a.publish,
                "cmd_vel_topic": self.a.cmd_vel_topic,
                "cmd_timeout": self.a.cmd_timeout,
                "image_age_s": None if img_age is None else round(img_age, 3),
                "n_img": n_img,
                "n_pred": self._n_pred,
                "n_pub": self._n_pub,
                "n_zero": self._n_zero,
                "server": self.a.server,
                "instruction": self.a.instruction,
                "worker_alive": self._worker is not None and self._worker.is_alive(),
                "fatal": self._infer_fatal,
                "web_on": self.web_on,
                "web_port": self._web_port if self.web_on else None,
                "web_clients": None if self._web is None else self._web.hub.n_clients,
                "web_frames": None if self._web is None else self._web.hub.n_published,
                # ---- 模式机 ----
                "mode": self._mode,
                "follow": self._follow,
                "phase": self._phase,
                "step_index": self._step_index,
                "step_exec_s": round(self._step_exec_s, 2),
                "pending_timeout_s": round(self._pending_timeout_s, 1),
                "pending_age_s": pend_age,
                "auto_gate": self._auto_gate,
                "auto_started": self._auto_start_evt.is_set(),
                "aborted": self._abort_evt.is_set(),
                "preview_hz": self.a.preview_hz,
                "preview_seq": self._preview_seq,
            },
            "history": hist,
        }

    def _publish(self, vx: float, vy: float, wz: float, *, zero: bool) -> None:
        if self._pub is None or not rclpy.ok():
            return
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        try:
            self._pub.publish(t)
        except Exception as e:  # 上下文已失效等情况，不要让定时器抛异常
            if not self._shutting_down:
                self.get_logger().warn(f"发布失败：{e}", throttle_duration_sec=5.0)
            return
        self._n_pub += 1
        if zero:
            self._n_zero += 1

    def _send_zero_burst(self, n: int = 10) -> None:
        """收尾连发零速，确保 cmdvel_to_sport 收到「停」。幂等，只发一次。

        必须在 rclpy 上下文还有效时调用；否则 publish 会报
        "publisher's context is invalid"。
        """
        if self._pub is None or self._zero_sent:
            return
        if not rclpy.ok():
            log("rclpy 上下文已关闭，无法发送停车帧 —— 真机靠 cmd_timeout/"
                "duration 自行停止，请务必目视确认")
            return
        self._zero_sent = True
        log("连发 %d 帧零速 …", n)
        t = Twist()
        for _ in range(n):
            try:
                self._pub.publish(t)
                self._n_pub += 1
                self._n_zero += 1
            except Exception as e:
                log("停车帧发送失败：%s", e)
                return
            time.sleep(0.03)

    # -- 生命周期 -----------------------------------------------------------
    def run(self) -> None:
        # 预览线程必须在推理线程之前起来：它是画面的唯一来源，
        # 单步模式挂起等确认时全靠它让网页活着。
        if self.a.preview_hz > 0:
            self._preview = threading.Thread(target=self._preview_loop,
                                             daemon=True, name="vln-preview")
            self._preview.start()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()
        self._worker_started = True
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            log("收到 Ctrl-C")
        except ExternalShutdownException:
            pass  # 外部（或看门狗）关掉了上下文，走正常收尾

    def shutdown(self) -> None:
        self._shutting_down = True
        log("安全停止 …")
        self._alive = False
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=3.0)
        # 关键顺序：零速必须在 destroy_node / rclpy.shutdown 之前发
        self._send_zero_burst()
        self._summary()
        self._close_ctl_socket()
        if self._web is not None:
            self._web.stop()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def status(self) -> dict[str, Any]:
        with self._img_lock:
            n_img = self._n_img
        return {"img_msgs": n_img, "infer_fatal": self._infer_fatal}

    def _summary(self) -> None:
        log("═══ 本次运行统计 ═══")
        log("  相机消息数    : %d", self._n_img)
        log("  预测次数      : %d", self._n_pred)
        if self._t_infer:
            arr = np.asarray(self._t_infer)
            log("  往返延迟      : 平均 %.1f ms / 中位 %.1f ms / P95 %.1f ms",
                arr.mean(), np.median(arr), np.percentile(arr, 95))
        log("  服务端错误次数: %d", self._recv_error)
        if self._n_resized:
            log("  图像降采样    : %d 次（--encode-width %d）",
                self._n_resized, self.a.encode_width)
        log("  发布 Twist 数 : %d（其中零速 %d）%s",
            self._n_pub, self._n_zero,
            "" if self.a.publish else "  ← 未使能发布，恒为 0")
        if self._infer_fatal:
            log("  ⚠️ 推理线程异常: %s", self._infer_fatal)

        if self.a.log_dir and self._stats:
            out = Path(self.a.log_dir)
            out.mkdir(parents=True, exist_ok=True)
            path = out / f"ros2_run_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
            with path.open("w", encoding="utf-8") as fh:
                fh.write(json.dumps({
                    "server": self.a.server, "instruction": self.a.instruction,
                    "task": self.a.task, "image_topic": self.a.image_topic,
                    "cmd_vel_topic": self.a.cmd_vel_topic, "publish": self.a.publish,
                    "fps": self.a.fps, "control_hz": self.a.control_hz,
                    "mapping": self.a.mapping,
                    "encode_width": self.a.encode_width,
                    "jpeg_quality": self.a.jpeg_quality,
                    "limits": {"vx": self.a.max_vx, "vy": self.a.max_vy,
                               "wz": self.a.max_wz},
                }, ensure_ascii=False) + "\n")
                for row in self._stats:
                    fh.write(json.dumps(row, ensure_ascii=False) + "\n")
            log("  结构化日志    : %s", path)


# ===========================================================================
# 命令行
# ===========================================================================
def build_parser() -> Any:
    import argparse

    p = argparse.ArgumentParser(
        prog="vln_ros2_bridge.py",
        description="LightNav-0 的 ROS 2 桥接：订阅相机图像，发布 cmd_vel（默认只观察不发布）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 纯观察（默认，不发布任何速度）
  python3 vln_ros2_bridge.py --instruction "go to the red chair"

  # 发布到安全话题，并在网页上手动画授权（推荐）
  python3 vln_ros2_bridge.py --instruction "go to the red chair" --publish \
      --web --encode-width 640

  # ROS 风格覆盖参数
  python3 vln_ros2_bridge.py --instruction "go" --ros-args -p max_vx:=0.3
""",
    )

    g = p.add_argument_group("服务端")
    g.add_argument("--server", default="ws://192.168.110.204:8050", help="lightnav-serve 地址")
    g.add_argument("--instruction", default="", help="自然语言指令，如 'go to the red chair'")
    g.add_argument("--task", choices=["vln", "tracking"], default="vln", help="仅用于日志标注")
    g.add_argument("--client-id", default="robot-01", help="clientId")

    g = p.add_argument_group("ROS 2 话题")
    g.add_argument("--image-topic", default="/camera/camera/color/image_raw",
                   help="相机彩色图像话题（namespace=camera name=camera 时的默认值）")
    g.add_argument("--cmd-vel-topic", default="/vln/cmd_vel",
                   help="速度输出话题。默认 /vln/cmd_vel，安全；"
                        "要上真机再显式改成 /cmd_vel_safe")
    g.add_argument("--publish", action="store_true",
                   help="真的发布 Twist。不传则纯观察（只打印）")
    g.add_argument("--max-image-age", type=float, default=1.0,
                   help="超过这个秒数没新图就视为无图，不下发")
    g.add_argument("--image-wait-s", type=float, default=20.0,
                   help="启动时等第一帧的最长秒数")

    g = p.add_argument_group("速度限制与换算")
    g.add_argument("--max-vx", type=float, default=0.5, help="最大前进速度 m/s")
    g.add_argument("--max-vy", type=float, default=0.0, help="最大横移 m/s（双足建议 0）")
    g.add_argument("--max-wz", type=float, default=0.8, help="最大偏航角速度 rad/s")
    g.add_argument("--mapping", choices=["normalized", "direct"], default="normalized",
                   help="航点→速度换算方式，与 vln_robot_client 一致")
    g.add_argument("--dt", type=float, default=0.1, help="--mapping direct 时每步时长 (s)")
    g.add_argument("--yaw-sign", type=float, choices=[1.0, -1.0], default=1.0,
                   help="转向反了就改 -1")
    g.add_argument("--lat-sign", type=float, choices=[1.0, -1.0], default=1.0,
                   help="横移反了就改 -1")

    g = p.add_argument_group("节拍与安全")
    g.add_argument("--fps", type=float, default=4.0, help="送图推理帧率（模型训练帧率，通常 4）")
    g.add_argument("--control-hz", type=float, default=20.0, help="速度下发频率")
    g.add_argument("--cmd-timeout", type=float, default=0.6,
                   help="指令保持时长，超过则归零（失速保护）")
    g.add_argument("--jpeg-quality", type=int, default=90)
    g.add_argument("--encode-width", type=int, default=0,
                   help="发往服务端前把图缩到该宽度（保持宽高比）。0=不缩。"
                        "1280x720 直接送图会让服务端耗时翻倍，建议 640")
    g.add_argument("--warmup-frames", type=int, default=0, help="开局预热历史窗帧数")
    g.add_argument("--max-runtime", type=float, default=0.0, help="最长运行秒数，0=不限")
    g.add_argument("--max-errors", type=int, default=5, help="连续服务端错误上限")
    g.add_argument("--ignore-stop", action="store_true", help="忽略模型的 stop 到达信号")
    g.add_argument("--stop-hold-s", type=float, default=0.5, help="到达后保持零速时长")
    g.add_argument("--stop-confirm", type=int, default=2,
                   help="连续多少帧 stop=true 才算到达（防单帧误判）")
    g.add_argument("--follow", action="store_true",
                   help="跟随模式：MPC 用 track、忽略 stop（跟人没有终点）。指令要写成 follow the person in ...")
    g.add_argument("--ctrl", choices=["mpc", "wp0"], default="wp0",
                   help="速度来源：mpc = 整条轨迹交给 vln_mpc 跟踪；wp0 = 只用第 1 个航点换算（旧行为）")
    g.add_argument("--mpc-cmd-timeout", type=float, default=0.3,
                   help="MPC 输出超过这个秒数没更新就发零速")

    g = p.add_argument_group("网页控制台")
    g.add_argument("--web", action="store_true",
                   help="启动时就把网页控制台挂上（不传也没关系，"
                        "之后用 ./start_vln.sh --web 随时挂）")
    g.add_argument("--ctl-socket", default="/tmp/vln_bridge.sock",
                   help="本地控制通道 unix socket 路径（空串关闭）。"
                        "网页没起来时靠它从命令行操作节点")
    g.add_argument("--web-host", default="0.0.0.0",
                   help="网页面板绑定地址，默认 0.0.0.0 便于从笔记本打开")
    g.add_argument("--web-port", type=int, default=8090,
                   help="网页面板端口（被占用时会自动往后试 12 个）")
    g.add_argument("--start-armed", action="store_true",
                   help="开网页时仍然一启动就授权。不传则必须先由人点击授权")
    g.add_argument("--approve-hold-s", type=float, default=1.0,
                   help="「执行一次」的授权时长（秒），到期自动收回授权")

    g = p.add_argument_group("模式 ：测试（单步）/ 正式（连续）")
    g.add_argument("--mode", choices=["step", "auto"], default="step",
                   help="step=测试模式：每次推理都要人点【确认执行】才走，一次一小步；"
                        "auto=正式模式：连接续推理并下发，直到 stop=true 或一键终止")
    g.add_argument("--step-exec-s", type=float, default=1.0,
                   help="测试模式「确认执行」后走多久（秒）。网页上可随时调整")
    g.add_argument("--pending-timeout-s", type=float, default=60.0,
                   help="测试模式：待确认悬挂超过该秒数就自动丢弃这一步（不执行），0=不限")
    g.add_argument("--auto-confirm", action="store_true",
                   help="正式模式一启动就跑。默认要先在网页点【开始连续】")

    g = p.add_argument_group("画面预览（与推理解耦）")
    g.add_argument("--preview-hz", type=float, default=5.0,
                   help="网页实时画面帧率。0=关，此时画面回退成推理帧（旧行为）")
    g.add_argument("--preview-width", type=int, default=480,
                   help="预览缩图宽度（保持宽高比），越小越省 CPU")

    g = p.add_argument_group("其它")
    g.add_argument("--log-dir", default="logs/ros2_bridge", help="结构化日志目录（空串关闭）")
    return p


def split_ros_args(argv: list[str]) -> tuple[list[str], list[str]]:
    """把 `--ros-args …` 从 argv 里摘出来交给 rclpy，其余给 argparse。"""
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


def main(argv: Optional[list[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    main_argv, ros_argv = split_ros_args(argv)
    a = build_parser().parse_args(main_argv)

    rclpy.init(args=ros_argv)
    node = VlnRos2Bridge(a)
    rc = 0
    try:
        node.run()
    finally:
        try:
            node.shutdown()
        except Exception as e:
            log("收尾异常：%s", e)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
