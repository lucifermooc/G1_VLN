from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass

import mujoco
import numpy as np
from PIL import Image

from .model import CAMERA_HEIGHT, CAMERA_WIDTH, PHYSICS_DT
from .mpc import TRACK_V_MAX, W_MAX
from .robots.base import RobotBackend
from .robots.turtlebot import TurtleBotBackend

COMMAND_TIMEOUT_S = 0.35
FRAME_PERIOD_S = 0.05


@dataclass(frozen=True)
class CameraFrame:
    stamp_ns: int
    rgb: bytes
    jpeg: bytes
    pose: tuple[float, float, float]


class Simulation:
    def __init__(self, robot: RobotBackend | None = None) -> None:
        self.robot = robot or TurtleBotBackend()
        self.model = self.robot.model
        self.data = self.robot.data
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._command = (0.0, 0.0)
        self._command_at = 0.0
        self._frame: CameraFrame | None = None
        self._third_person_frame: CameraFrame | None = None
        self._frame_count = 0
        self._started_at = time.monotonic()

    @property
    def robot_name(self) -> str:
        return self.robot.name

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mujoco", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        self._thread = None

    def reset(self) -> None:
        with self._lock:
            self.robot.reset()
            self._command = (0.0, 0.0)
            self._command_at = 0.0

    def set_velocity(self, linear: float, angular: float) -> None:
        # Physical sanity bound only; per-mode VLN limits live in the MPC.
        linear = max(-TRACK_V_MAX, min(TRACK_V_MAX, float(linear)))
        angular = max(-W_MAX, min(W_MAX, float(angular)))
        with self._lock:
            self._command = (linear, angular)
            self._command_at = time.monotonic()

    def frame(self) -> CameraFrame | None:
        with self._lock:
            return self._frame

    def third_person_frame(self) -> CameraFrame | None:
        with self._lock:
            return self._third_person_frame

    @staticmethod
    def _encode_frame(
        stamp_ns: int,
        rgb: np.ndarray,
        pose: tuple[float, float, float],
    ) -> CameraFrame:
        output = io.BytesIO()
        Image.fromarray(rgb, "RGB").save(output, format="JPEG", quality=82)
        return CameraFrame(stamp_ns, rgb.tobytes(), output.getvalue(), pose)

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            state = self.robot.state()
            x, y, z, yaw = state.pose
            linear, angular = state.velocity
            command = self._command
            frame = self._frame
            elapsed = max(time.monotonic() - self._started_at, 1e-6)
            snapshot = {
                "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
                "velocity": {"linear": linear, "angular": angular},
                "command": {"linear": command[0], "angular": command[1]},
                "sim_time": float(self.data.time),
                "camera": {
                    "ready": frame is not None,
                    "stamp_ns": frame.stamp_ns if frame is not None else None,
                    "fps": self._frame_count / elapsed,
                },
            }
            if state.telemetry:
                snapshot["backend"] = state.telemetry
            return snapshot

    def _advance(self, now: float) -> None:
        command = self._command
        if self._command_at and now - self._command_at > COMMAND_TIMEOUT_S:
            command = (0.0, 0.0)
            self._command = command
        self.robot.step(command)

    def _run(self) -> None:
        renderer = mujoco.Renderer(
            self.model,
            height=CAMERA_HEIGHT,
            width=CAMERA_WIDTH,
        )
        third_person_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(third_person_camera)
        next_step = time.monotonic()
        next_frame = next_step
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now < next_step:
                    time.sleep(min(next_step - now, 0.002))
                    continue
                with self._lock:
                    self._advance(now)
                next_step += PHYSICS_DT
                if now - next_step > 0.25:
                    next_step = now
                if now >= next_frame:
                    with self._lock:
                        renderer.update_scene(
                            self.data,
                            camera=self.robot.camera_name,
                        )
                        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
                        render_camera = self.robot.third_person_camera(
                            third_person_camera
                        )
                        renderer.update_scene(self.data, camera=render_camera)
                        third_person_rgb = np.asarray(
                            renderer.render(), dtype=np.uint8
                        ).copy()
                        x, y, _, yaw = self.robot.state().pose
                        capture_pose = (x, y, yaw)
                    stamp_ns = time.time_ns()
                    frame = self._encode_frame(stamp_ns, rgb, capture_pose)
                    third_person_frame = self._encode_frame(
                        stamp_ns,
                        third_person_rgb,
                        capture_pose,
                    )
                    with self._lock:
                        self._frame = frame
                        self._third_person_frame = third_person_frame
                        self._frame_count += 1
                    next_frame = now + FRAME_PERIOD_S
        finally:
            renderer.close()
