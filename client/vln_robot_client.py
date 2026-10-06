#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LightNav-0 机器人端独立客户端（零 ROS 依赖，可直接拷到机器人上运行）。

它在机器人上做三件事：

     相机 (RealSense / V4L2 / mp4 / 帧目录)
        │  ① 抓 RGB 帧
        ▼
     JPEG → base64
        │  ② {"action":"next","data":{"seq":N,"image":...,"instruction":...}}
        ▼
     lightnav-serve (GPU 主机, 例如 ws://192.168.110.204:8050)
        │  ③ 回包 {"actions":{"actions":[[fwd,lat,yaw] × H]},"stop":...}
        ▼
     航点[0] → 速度指令
        │  ④ 控制线程以 control_hz 持续下发
        ▼
     机器人底盘 (Unitree G1/H1 / dry_run 仅打印)

设计要点
--------
* **推理频率与下发频率解耦**：推理按 ``--fps``（≈ 模型训练帧率 4 Hz）跑，
  速度指令按 ``--control-hz``（默认 20 Hz）以零阶保持下发。中间靠
  ``--cmd-timeout`` 做失速保护：超过该时长没拿到新航点就自动归零。
* **控制线程独占底盘句柄**：所有机器人 SDK 调用只发生在控制线程，
  避免 DDS/串口的多线程竞争。
* **航点换算**：默认用仓库同款归一化映射（与 ``lightnav-ws-client``、
  EVT-Bench 客户端一致），也可以切到 ``--mapping direct`` 用 v=位移/dt。

依赖（机器人端；**不需要 torch、不需要 GPU**）
---------------------------------------------
    pip install "websockets>=12" Pillow numpy
    pip install pyrealsense2             # 用 RealSense 时
    pip install opencv-python-headless   # 用 USB/V4L2 摄像头或回放 mp4 时
    # 真机底盘见 README_CN.md（Unitree 官方 unitree_sdk2_python）

快速自测（不接机器人，先在机器人上把这一步跑通）
------------------------------------------------
    python3 vln_robot_client.py \\
        --server ws://192.168.110.204:8050 \\
        --camera dir --frames ./frames \\
        --instruction "go to the red chair" --robot dry_run
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

# ---------------------------------------------------------------------------
# 常量：把航点换算成 [-1,1] 指令用的「单步最大位移」。
# 与 src/lightnav/cli/ws_client.py、EVT-Bench 客户端保持一致。
# ---------------------------------------------------------------------------
WP_FWD_MAX = 0.375           # 单步最大前进位移 (m)
WP_LAT_MAX = 0.25            # 单步最大横移位移 (m)
WP_YAW_MAX = math.pi / 20.0  # 单步最大偏航 (rad) ≈ 9°


# ===========================================================================
# 工具
# ===========================================================================
def log(msg: str, *args: Any) -> None:
    """带时间戳的日志。"""
    if args:
        msg = msg % args
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def encode_jpeg_b64(rgb: np.ndarray, quality: int = 90) -> str:
    """HWC uint8 RGB → base64 JPEG 字符串。"""
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("utf-8")


@dataclass
class VelocityCmd:
    """一条待下发的速度指令（机器人坐标系）。"""

    vx: float = 0.0      # 前进 m/s（+ 向前）
    vy: float = 0.0      # 横移 m/s（+ 向左）
    wz: float = 0.0      # 偏航 rad/s（+ 逆时针）
    ts: float = 0.0      # 生成时刻 time.monotonic()
    step: int = -1       # 服务端返回的 actions.step，便于观察
    stop: bool = False

    @staticmethod
    def zero() -> "VelocityCmd":
        return VelocityCmd(ts=time.monotonic())


def waypoint_to_velocity(
    wp: np.ndarray,
    *,
    mapping: str,
    dt: float,
    max_vx: float,
    max_vy: float,
    max_wz: float,
    yaw_sign: float = 1.0,
    lat_sign: float = 1.0,
) -> tuple[float, float, float]:
    """把单个航点 ``[forward_m, lateral_m, yaw_rad]`` 换算成 (vx, vy, wz)。

    ``mapping='normalized'``（默认，与仓库客户端一致）:
        先按单步最大位移归一化到 [-1,1]，再乘以平台限速。
            n_vx = clip(fwd / 0.375, -1, 1)
            n_vy = clip(lat / 0.25 , -1, 1)
            n_wz = clip(yaw / (π/20), -1, 1)
        这种方式的物理含义是「这一步相对模型单步极限的激进程度」，
        换平台时不需要重调，最稳。

    ``mapping='direct'``:
        v = 位移 / dt，直接按控制周期换算成物理速度，再 clip 到限速。
        需要你自己确认 dt 与模型训练时的单步时间一致，否则速度会整体偏大/偏小。
    """
    fwd, lat, yaw = float(wp[0]), float(wp[1]), float(wp[2])

    if mapping == "direct":
        if dt <= 0:
            raise ValueError("--dt 必须为正")
        vx = fwd / dt
        vy = lat / dt
        wz = yaw / dt
    else:  # normalized
        vx = clip(fwd / WP_FWD_MAX, -1.0, 1.0) * max_vx
        vy = clip(lat / WP_LAT_MAX, -1.0, 1.0) * max_vy
        wz = clip(yaw / WP_YAW_MAX, -1.0, 1.0) * max_wz

    vy *= lat_sign
    wz *= yaw_sign
    return clip(vx, -max_vx, max_vx), clip(vy, -max_vy, max_vy), clip(wz, -max_wz, max_wz)


# ===========================================================================
# 相机
# ===========================================================================
class CameraBase:
    """相机接口：``read()`` 返回 HWC uint8 RGB。"""

    def open(self) -> None:  # pragma: no cover - 子类实现
        pass

    def read(self) -> np.ndarray:  # pragma: no cover - 子类实现
        raise NotImplementedError

    def close(self) -> None:  # pragma: no cover - 子类实现
        pass

    @property
    def size(self) -> tuple[int, int]:
        """(宽, 高)，用于日志。"""
        return (0, 0)


class RealSenseCamera(CameraBase):
    """Intel RealSense（D435/D455 等），走官方 pyrealsense2。"""

    def __init__(self, width: int, height: int, fps: int, serial: Optional[str] = None):
        self.req_w, self.req_h, self.req_fps = width, height, fps
        self.serial = serial
        self.pipe = None
        self._size = (0, 0)

    def open(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise SystemExit(
                "未安装 pyrealsense2，无法使用 --camera realsense。\n"
                '  安装：pip install pyrealsense2\n'
                "  或改用其它相机：--camera v4l2 --device /dev/video0"
            ) from e

        self._rs = rs
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.req_w, self.req_h, rs.format.rgb8, self.req_fps)
        self.pipe = rs.pipeline()
        profile = self.pipe.start(cfg)
        vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._size = (vsp.width(), vsp.height())
        log("RealSense 已打开：%dx%d @%dfps", *self._size, self.req_fps)

    def read(self) -> np.ndarray:
        frames = self.pipe.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("RealSense 未取到颜色帧")
        return np.asanyarray(color.get_data())  # 已是 RGB

    def close(self) -> None:
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass

    @property
    def size(self) -> tuple[int, int]:
        return self._size


class V4L2Camera(CameraBase):
    """UVC / V4L2 摄像头（也适用于以 UVC 模式暴露的 Orbbec Gemini）。"""

    def __init__(self, device: str, width: int, height: int, fps: int):
        self.device, self.req_w, self.req_h, self.fps = device, width, height, fps
        self.cap = None
        self._size = (0, 0)

    def open(self) -> None:
        import cv2

        # V4L2 后端比默认的更可靠，优先用它
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            raise SystemExit(f"打不开摄像头 {self.device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._size = (
            int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        log("V4L2 摄像头已打开：%s %dx%d", self.device, *self._size)

    def read(self) -> np.ndarray:
        import cv2

        ok, bgr = self.cap.read()
        if not ok:
            raise RuntimeError(f"从 {self.device} 读帧失败")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()

    @property
    def size(self) -> tuple[int, int]:
        return self._size


class OfflineCamera(CameraBase):
    """离线素材源：mp4 或帧目录。用于不接机器人时跑通整条链路。"""

    def __init__(self, video: Optional[str] = None, frames: Optional[str] = None,
                 target_fps: Optional[float] = None, loop: bool = True):
        if not video and not frames:
            raise SystemExit("--camera video/dir 需要 --video 或 --frames")
        self.video, self.frames_dir = video, frames
        self.target_fps, self.loop = target_fps, loop
        self._data: list[np.ndarray] = []
        self._idx = 0
        self._size = (0, 0)

    def _load_video(self, path: str) -> None:
        import cv2

        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise SystemExit(f"打不开视频 {path}")
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
        stride = max(1, round(src_fps / self.target_fps)) if self.target_fps and src_fps > 0 else 1
        i = 0
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            if i % stride == 0:
                self._data.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            i += 1
        cap.release()

    def _load_dir(self, path: str) -> None:
        from PIL import Image

        exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        files = sorted(p for p in Path(path).iterdir() if p.suffix.lower() in exts)
        if not files:
            raise SystemExit(f"{path} 里没有图片")
        for p in files:
            self._data.append(np.asarray(Image.open(p).convert("RGB")))

    def open(self) -> None:
        if self.video:
            self._load_video(self.video)
        else:
            self._load_dir(self.frames_dir)  # type: ignore[arg-type]
        if not self._data:
            raise SystemExit("离线素材为空")
        self._size = (self._data[0].shape[1], self._data[0].shape[0])
        log("离线素材 %d 帧，%dx%d", len(self._data), *self._size)

    def read(self) -> np.ndarray:
        if self._idx >= len(self._data):
            if not self.loop:
                raise StopIteration("素材已放完")
            self._idx = 0
        f = self._data[self._idx]
        self._idx += 1
        return f

    @property
    def size(self) -> tuple[int, int]:
        return self._size


def make_camera(a: argparse.Namespace) -> CameraBase:
    if a.camera == "realsense":
        return RealSenseCamera(a.width, a.height, int(a.fps), a.serial)
    if a.camera == "v4l2":
        return V4L2Camera(a.device, a.width, a.height, int(a.fps))
    if a.camera == "video":
        return OfflineCamera(video=a.video, target_fps=a.fps, loop=not a.no_loop)
    if a.camera == "dir":
        return OfflineCamera(frames=a.frames, target_fps=a.fps, loop=not a.no_loop)
    raise SystemExit(f"未知 --camera {a.camera}")


# ===========================================================================
# 机器人后端
# ===========================================================================
class RobotBase:
    """机器人后端接口。所有方法只在控制线程里被调用。"""

    def prepare(self) -> None:
        """进入可接收速度指令的状态（例如让人形站起来）。"""

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """安全停止：先归零，必要时阻尼/下蹲。"""

    def close(self) -> None:
        pass


class DryRunRobot(RobotBase):
    """只打印，不碰硬件。用于在机器人上验证网络与推理链路。"""

    def __init__(self, max_vx: float = 1.0):
        self.max_vx = max_vx
        self._last = None
        self._printed = 0.0

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        now = time.monotonic()
        key = (round(vx, 3), round(vy, 3), round(wz, 3))
        # 限流打印：指令变化时立刻打印，否则最多每 1 秒一次
        if key != self._last or now - self._printed > 1.0:
            log("  → dry_run 下发  vx=%+.3f m/s  vy=%+.3f m/s  wz=%+.3f rad/s", vx, vy, wz)
            self._last = key
            self._printed = now

    def stop(self) -> None:
        log("  → dry_run 停止（归零）")


class UnitreeHumanoidRobot(RobotBase):
    """宇树双足（G1 / H1 系列）后端，走官方 ``unitree_sdk2_python`` 的 LocoClient。

    ⚠️ 重要说明
    -----------
    仓库自带的适配器只有 **Go2（四足）** 和 **LimX TRON1**，没有宇树双足。
    本类是按 Unitree 官方 SDK 的高层运动接口（``LocoClient``）写的参考实现：

        ChannelFactoryInitialize(0, iface)
        client = LocoClient(); client.SetTimeout(10.0); client.Init()
        client.StandUp()                     # 或 Start()
        client.Move(vx, vy, vyaw)            # 持续速度指令
        client.StopMove() / client.Damp()    # 停 / 阻尼

    不同 SDK 版本的方法名（Move / SetVelocity / SetFsmId）和 DDS 话题名
    （rt/sportmodestate vs rt/lf/sportmodestate）存在差异，所以这里：
      * 先用 ``--robot dry_run`` 验证链路，确认无误再上真机；
      * 方法调用做了多路回退（找不到就换下一个名字）；
      * 你可以在本类里按自己机器人的手册改 3~5 行即可适配。
    详见 robot_client/README_CN.md 的「宇树双足适配」一节。
    """

    def __init__(self, model: str, iface: str, watchdog_s: float,
                 max_vx: float, max_vy: float, max_wz: float,
                 auto_stand: bool = True, hard_stop: bool = True):
        self.model = model.lower()
        self.iface = iface
        self.watchdog_s = watchdog_s
        self.max_vx, self.max_vy, self.max_wz = max_vx, max_vy, max_wz
        self.auto_stand = auto_stand
        self.hard_stop = hard_stop

        self.client: Any = None
        self._last_send = 0.0
        self._watchdog: Optional[threading.Thread] = None
        self._alive = False
        self._stated = False

    # -- 生命周期 ----------------------------------------------------------
    def _import_sdk(self) -> Any:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        except ImportError as e:
            raise SystemExit(
                "未安装 Unitree 官方 SDK：unitree_sdk2_python\n"
                "  安装步骤见 robot_client/README_CN.md\n"
                "  想先验证链路请用：--robot dry_run"
            ) from e
        return ChannelFactoryInitialize

    def _make_client(self) -> Any:
        # 官方 SDK 的模块路径：G1 → unitree_sdk2py.g1.loco.g1_loco_client
        #                      H1 → unitree_sdk2py.h1.loco.h1_loco_client
        if self.model in ("g1", "unitree_g1"):
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        elif self.model in ("h1", "h1_2", "unitree_h1"):
            from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
        else:
            raise SystemExit(f"未知宇树型号 {self.model}（支持 g1 / h1）")
        return LocoClient()

    def prepare(self) -> None:
        init = self._import_sdk()
        # 0 = 任意域号，iface = 与机器人相连的网卡名（如 eth0 / enp2s0）
        init(0, self.iface)
        log("Unitree SDK 已初始化（iface=%s, model=%s）", self.iface, self.model)

        self.client = self._make_client()
        self._try("SetTimeout", 10.0)
        self._try("Init")

        if self.auto_stand:
            log("尝试让机器人站立 …")
            # 不同固件可能叫 Start / StandUp / HighStand，逐个试
            for name in ("Start", "StandUp", "HighStand"):
                if self._try(name):
                    log("  %s() 成功", name)
                    break
            else:
                log("  未找到可用的站立接口，请手动用遥控器让机器人进入可运动状态")

        self._alive = True
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()
        self._stated = True

    def _try(self, name: str, *args: Any) -> bool:
        """尝试调用 SDK 方法；方法不存在或签名不符则返回 False。"""
        fn = getattr(self.client, name, None)
        if fn is None:
            return False
        try:
            fn(*args)
            return True
        except Exception as e:  # 签名/状态不符都不致命
            log("  调用 %s(%s) 失败：%s", name, args, e)
            return False

    # -- 指令 --------------------------------------------------------------
    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        if self.client is None:
            return
        self._last_send = time.monotonic()
        # Move(vx, vy, vyaw) 是 Unitree 高层运动接口的通用签名；
        # 某些版本还有尾参 continous_move（官方拼写如此）。
        if getattr(self.client, "SetVelocity", None) is not None:
            ok = self._try("SetVelocity", vx, vy, wz)
        else:
            ok = False
        if not ok:
            try:
                self.client.Move(vx, vy, wz)
            except TypeError:
                self.client.Move(vx, vy, wz, False)
            except Exception as e:
                log("Move() 失败：%s", e)

    def _watchdog_loop(self) -> None:
        """命令看门狗：超时未收到新指令就主动归零，防止机器人失控跑飞。"""
        while self._alive:
            time.sleep(0.05)
            if self._last_send and time.monotonic() - self._last_send > self.watchdog_s:
                try:
                    if getattr(self.client, "SetVelocity", None) is not None:
                        self.client.SetVelocity(0.0, 0.0, 0.0)
                    else:
                        self.client.Move(0.0, 0.0, 0.0)
                except Exception:
                    pass
                self._last_send = time.monotonic()  # 只归零一次，别刷屏

    # -- 收尾 --------------------------------------------------------------
    def stop(self) -> None:
        log("安全停止：连续发送零速度 …")
        # 连发一小段零速，确保底层收到
        for _ in range(10):
            try:
                if getattr(self.client, "SetVelocity", None) is not None:
                    self.client.SetVelocity(0.0, 0.0, 0.0)
                else:
                    self.client.Move(0.0, 0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.05)
        self._try("StopMove")
        if self.hard_stop:
            # 阻尼模式：关节卸力，机器人会软下来（双足上通常比原地保持更安全）
            self._try("Damp")

    def close(self) -> None:
        self._alive = False
        if self._watchdog is not None:
            self._watchdog.join(timeout=1.0)
        self.stop()


def make_robot(a: argparse.Namespace) -> RobotBase:
    if a.robot in ("dry_run", "dry-run", "none"):
        return DryRunRobot()
    if a.robot in ("g1", "unitree_g1", "h1", "h1_2", "unitree_h1"):
        return UnitreeHumanoidRobot(
            model=a.robot,
            iface=a.iface,
            watchdog_s=a.watchdog_s,
            max_vx=a.max_vx,
            max_vy=a.max_vy,
            max_wz=a.max_wz,
            auto_stand=not a.no_stand,
            hard_stop=not a.no_hard_stop,
        )
    raise SystemExit(f"未知 --robot {a.robot}")


# ===========================================================================
# 紧急停止：键盘单键
# ===========================================================================
class EStop:
    """后台读 stdin 的紧急停止开关。

    支持单键（无需回车，需终端支持）：
        q / Q / Esc → 退出
        空格        → 暂停 / 恢复（暂停时持续下发零速）
    """

    def __init__(self) -> None:
        self.quit = threading.Event()
        self.pause = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._old_term = None

    def start(self) -> None:
        if not sys.stdin.isatty():
            log("stdin 不是终端，跳过键盘热键（Ctrl-C 仍可随时中断）")
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        try:
            import termios
            import tty

            fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            while not self.quit.is_set():
                ch = sys.stdin.read(1)
                if ch in ("q", "Q", "\x1b"):
                    log("收到退出键，准备安全停止 …")
                    self.quit.set()
                    break
                if ch == " ":
                    if self.pause.is_set():
                        self.pause.clear()
                        log("▶ 恢复运动")
                    else:
                        self.pause.set()
                        log("⏸ 已暂停（持续下发零速），再按空格恢复")
        except Exception:
            pass
        finally:
            if self._old_term is not None:
                try:
                    import termios

                    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_term)
                except Exception:
                    pass

    def stop(self) -> None:
        self.quit.set()


# ===========================================================================
# 主循环
# ===========================================================================
class EpisodeRunner:
    def __init__(self, a: argparse.Namespace, camera: CameraBase, robot: RobotBase):
        self.a = a
        self.camera = camera
        self.robot = robot
        self.estop = EStop()

        self.cmd = VelocityCmd()                 # 最新的待下发指令
        self.cmd_lock = threading.Lock()
        self.seq = 0
        self.stats: list[dict] = []
        self._ctrl_thread: Optional[threading.Thread] = None
        self._alive = False
        self.recv_error = 0

        # 统计
        self.t_infer: list[float] = []

    # -- 控制线程 ----------------------------------------------------------
    def _control_loop(self) -> None:
        period = 1.0 / self.a.control_hz
        next_t = time.monotonic()
        while self._alive:
            next_t += period
            with self.cmd_lock:
                c = self.cmd
            age = time.monotonic() - c.ts
            if self.estop.pause.is_set() or c.stop or age > self.a.cmd_timeout:
                # 暂停 / 到达 / 指令过期 → 归零（保持底盘安全）
                self.robot.set_velocity(0.0, 0.0, 0.0)
            else:
                self.robot.set_velocity(c.vx, c.vy, c.wz)
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    def _publish(self, vx: float, vy: float, wz: float, step: int, stop: bool) -> None:
        with self.cmd_lock:
            self.cmd = VelocityCmd(vx, vy, wz, time.monotonic(), step, stop)

    # -- 推理主循环 --------------------------------------------------------
    def run(self) -> int:
        a = self.a
        from websockets.sync.client import connect

        log("连接服务端 %s …", a.server)
        ws_kwargs: dict[str, Any] = {"max_size": 64 * 1024 * 1024,
                                     "open_timeout": 15, "close_timeout": 3}
        ws = connect(a.server, **ws_kwargs).__enter__()

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

        # 预热历史窗：用空 instruction 把当前帧塞进服务端帧缓冲，
        # 这样第一次预测就带着足够的历史，而不是只有 1 帧。
        if a.warmup_frames > 0:
            log("预热历史窗 %d 帧（仅入缓冲，不推理）…", a.warmup_frames)
            frame = self.camera.read()
            for _ in range(a.warmup_frames):
                ws.send(json.dumps({"action": "next", "data": {
                    "seq": self.seq, "image": encode_jpeg_b64(frame, a.jpeg_quality),
                    "instruction": "",
                }}))
                self.seq += 1
                ws.recv()

        self.robot.prepare()
        self._alive = True
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._ctrl_thread.start()
        self.estop.start()

        period = 1.0 / a.fps
        next_t = time.monotonic()
        t_start = time.monotonic()
        n_pred = 0
        rc = 0

        log("═══ 开始闭环：推理 %.1f Hz，下发 %.1f Hz，指令「%s」═══",
            a.fps, a.control_hz, a.instruction)
        try:
            while not self.estop.quit.is_set():
                next_t += period
                if a.max_runtime > 0 and time.monotonic() - t_start > a.max_runtime:
                    log("达到 --max-runtime %.0fs，正常结束", a.max_runtime)
                    break

                # 1) 抓帧
                try:
                    frame = self.camera.read()
                except StopIteration:
                    log("素材播放完毕，结束")
                    break

                # 2) 送服务端
                t0 = time.monotonic()
                ws.send(json.dumps({"action": "next", "data": {
                    "seq": self.seq,
                    "image": encode_jpeg_b64(frame, a.jpeg_quality),
                    "instruction": a.instruction,
                }}))
                self.seq += 1
                data = json.loads(ws.recv())["data"]
                rtt = (time.monotonic() - t0) * 1000.0

                if data.get("rc") != 0:
                    self.recv_error += 1
                    log("服务端返回错误 rc=%s msg=%s（沿用上一条指令）", data.get("rc"), data.get("msg"))
                    if self.recv_error > a.max_errors:
                        raise SystemExit(f"连续错误超过 {a.max_errors} 次，退出")
                    continue

                if "actions" not in data:
                    # 空 instruction 的缓冲回包
                    continue

                self.recv_error = 0
                acts = data["actions"]["actions"]
                step = int(data["actions"].get("step", -1))
                stop = bool(data.get("stop", False))
                visible = data.get("visible")
                lat = float(data.get("latency_ms", 0.0))

                # 3) 航点 → 速度
                vx, vy, wz = waypoint_to_velocity(
                    np.asarray(acts[0], dtype=np.float64),
                    mapping=a.mapping, dt=a.dt,
                    max_vx=a.max_vx, max_vy=a.max_vy, max_wz=a.max_wz,
                    yaw_sign=a.yaw_sign, lat_sign=a.lat_sign,
                )
                self._publish(vx, vy, wz, step, stop)
                n_pred += 1
                self.t_infer.append(rtt)
                self.stats.append({
                    "seq": self.seq - 1, "step": step, "rtt_ms": round(rtt, 1),
                    "server_ms": round(lat, 1), "vx": round(vx, 4),
                    "vy": round(vy, 4), "wz": round(wz, 4),
                    "wp0": [float(x) for x in acts[0]], "stop": stop,
                    "visible": visible, "raw_text": data.get("raw_text", ""),
                    "pointing": data.get("pointing"),
                })

                log("seq=%3d  rtt=%6.1fms  srv=%6.1fms  wp0=[%+.3f %+.3f %+.3f]  "
                    "cmd=(vx%+.3f vy%+.3f wz%+.3f)  stop=%s vis=%s",
                    self.seq - 1, rtt, lat, acts[0][0], acts[0][1], acts[0][2],
                    vx, vy, wz, stop, visible)
                # 跟踪任务：把「目标在哪个方向 / 是否被挡住」也打出来，现场调试很有用
                pt = data.get("pointing") or {}
                if pt.get("opos_state") or pt.get("apos_state"):
                    log("       apos=%s%s  opos=%s%s",
                        pt.get("apos_state"), " (clamped)" if pt.get("apos_clamped") else "",
                        pt.get("opos_state"), " (clamped)" if pt.get("opos_clamped") else "")

                # 4) 到达
                if stop and not a.ignore_stop:
                    log("★ 模型判定到达目标（stop=true），停下并结束 episode")
                    self._publish(0.0, 0.0, 0.0, step, True)
                    time.sleep(a.stop_hold_s)
                    break

                # 5) 节拍
                dt = next_t - time.monotonic()
                if dt > 0:
                    time.sleep(dt)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
            log("收到 Ctrl-C")
        except Exception as e:
            rc = 1
            log("运行出错：%s: %s", type(e).__name__, e)
        finally:
            # 收尾：先归零，再退出
            self._alive = False
            if self._ctrl_thread is not None:
                self._ctrl_thread.join(timeout=2.0)
            log("安全停止 …")
            self.robot.stop()
            try:
                ws.__exit__(None, None, None)
            except Exception:
                pass
            self.camera.close()

        self._summary(n_pred)
        return rc

    def _summary(self, n_pred: int) -> None:
        log("═══ 本次运行统计 ═══")
        log("  预测次数      : %d", n_pred)
        if self.t_infer:
            arr = np.asarray(self.t_infer)
            log("  往返延迟      : 平均 %.1f ms / 中位 %.1f ms / P95 %.1f ms",
                arr.mean(), np.median(arr), np.percentile(arr, 95))
        log("  服务端错误次数: %d", self.recv_error)

        if self.a.log_dir and self.stats:
            out = Path(self.a.log_dir)
            out.mkdir(parents=True, exist_ok=True)
            p = out / f"run_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
            with p.open("w", encoding="utf-8") as f:
                f.write(json.dumps({
                    "server": self.a.server, "instruction": self.a.instruction,
                    "task": self.a.task, "robot": self.a.robot,
                    "camera": self.a.camera, "fps": self.a.fps,
                    "control_hz": self.a.control_hz, "mapping": self.a.mapping,
                    "limits": {"vx": self.a.max_vx, "vy": self.a.max_vy, "wz": self.a.max_wz},
                }, ensure_ascii=False) + "\n")
                for s in self.stats:
                    f.write(json.dumps(s, ensure_ascii=False) + "\n")
            log("  结构化日志    : %s", p)


# ===========================================================================
# CLI
# ===========================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="vln_robot_client.py",
        description="LightNav-0 机器人端独立客户端（零 ROS 依赖）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  # 不接机器人，回放素材跑通链路
  python3 vln_robot_client.py --server ws://192.168.110.204:8050 \\
      --camera dir --frames ./frames --instruction "go to the red chair" --robot dry_run

  # 接 RealSense + 宇树 G1
  python3 vln_robot_client.py --server ws://192.168.110.204:8050 \\
      --camera realsense --robot unitree_g1 --iface eth0 \\
      --instruction "go to the red chair" --max-vx 0.4 --max-wz 0.6
""",
    )

    g = p.add_argument_group("服务端")
    g.add_argument("--server", default="ws://127.0.0.1:8050", help="lightnav-serve 地址")
    g.add_argument("--instruction", required=True, help="自然语言指令（每个 next 都会带上，可随时改）")
    g.add_argument("--task", choices=["vln", "tracking"], default="vln", help="仅用于日志标注")
    g.add_argument("--client-id", default="robot-01", help="clientId（仅日志用）")

    g = p.add_argument_group("相机")
    g.add_argument("--camera", choices=["realsense", "v4l2", "video", "dir"], default="realsense")
    g.add_argument("--device", default="/dev/video0", help="--camera v4l2 用")
    g.add_argument("--video", help="--camera video 用")
    g.add_argument("--frames", help="--camera dir 用")
    g.add_argument("--serial", help="RealSense 序列号（多台时指定）")
    g.add_argument("--width", type=int, default=640)
    g.add_argument("--height", type=int, default=480)
    g.add_argument("--jpeg-quality", type=int, default=90)
    g.add_argument("--no-loop", action="store_true", help="离线素材播完就停（默认循环）")

    g = p.add_argument_group("机器人")
    g.add_argument("--robot", default="dry_run",
                   help="dry_run | unitree_g1 | unitree_h1（默认 dry_run，先验证链路）")
    g.add_argument("--iface", default="eth0", help="宇树 DDS 网卡名（与机器人相连的网口）")
    g.add_argument("--no-stand", action="store_true", help="不自动让机器人站立")
    g.add_argument("--no-hard-stop", action="store_true", help="停止时不做 Damp 阻尼")

    g = p.add_argument_group("速度限制与换算")
    g.add_argument("--max-vx", type=float, default=0.5, help="最大前进速度 m/s")
    g.add_argument("--max-vy", type=float, default=0.0, help="最大横移速度 m/s（宇树双足建议 0）")
    g.add_argument("--max-wz", type=float, default=0.8, help="最大偏航角速度 rad/s")
    g.add_argument("--mapping", choices=["normalized", "direct"], default="normalized",
                   help="航点→速度的换算方式（默认 normalized，与仓库客户端一致）")
    g.add_argument("--dt", type=float, default=0.1, help="--mapping direct 时每步时长 (s)")
    g.add_argument("--yaw-sign", type=float, choices=[1.0, -1.0], default=1.0,
                   help="若转向反了改成 -1")
    g.add_argument("--lat-sign", type=float, choices=[1.0, -1.0], default=1.0,
                   help="若横移反了改成 -1")

    g = p.add_argument_group("节拍与安全")
    g.add_argument("--fps", type=float, default=4.0, help="推理帧率（模型训练帧率，通常 4）")
    g.add_argument("--control-hz", type=float, default=20.0, help="速度下发频率")
    g.add_argument("--cmd-timeout", type=float, default=0.6,
                   help="指令保持时长；超过则归零（失速保护）")
    g.add_argument("--watchdog-s", type=float, default=0.4, help="底盘层看门狗超时")
    g.add_argument("--warmup-frames", type=int, default=0, help="开局预填历史窗帧数")
    g.add_argument("--max-runtime", type=float, default=0.0, help="最长运行秒数，0=不限")
    g.add_argument("--max-errors", type=int, default=5, help="连续服务端错误上限")
    g.add_argument("--ignore-stop", action="store_true", help="忽略模型的 stop 到达信号")
    g.add_argument("--stop-hold-s", type=float, default=0.5, help="到达后保持零速时长")

    g = p.add_argument_group("其它")
    g.add_argument("--log-dir", default="logs/robot_client", help="结构化日志目录（空串关闭）")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    a = build_parser().parse_args(argv)

    log("═══ LightNav-0 机器人端客户端 ═══")
    log("服务端   : %s", a.server)
    log("指令     : %s", a.instruction)
    log("相机     : %s", a.camera)
    log("机器人   : %s%s", a.robot,
        "   ⚠️ 只打印，不驱动真实硬件" if a.robot in ("dry_run", "dry-run", "none") else "")
    log("限速     : vx≤%.2f m/s  vy≤%.2f m/s  wz≤%.2f rad/s", a.max_vx, a.max_vy, a.max_wz)
    log("节拍     : 推理 %.1f Hz / 下发 %.1f Hz / 失速 %.2f s", a.fps, a.control_hz, a.cmd_timeout)

    camera = make_camera(a)
    camera.open()
    robot = make_robot(a)

    def _sigint(_sig: int, _frm: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint)

    try:
        return EpisodeRunner(a, camera, robot).run()
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
