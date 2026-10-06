from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, TypeAlias

import mujoco

Twist2D: TypeAlias = tuple[float, float]
RobotPose: TypeAlias = tuple[float, float, float, float]
RenderCamera: TypeAlias = str | mujoco.MjvCamera


@dataclass(frozen=True)
class RobotState:
    pose: RobotPose
    velocity: Twist2D
    telemetry: dict[str, object] = field(default_factory=dict)


class RobotBackend(Protocol):
    name: str
    camera_name: str
    model: mujoco.MjModel
    data: mujoco.MjData

    def reset(self) -> None: ...

    def step(self, command: Twist2D) -> None: ...

    def state(self) -> RobotState: ...

    def third_person_camera(self, camera: mujoco.MjvCamera) -> RenderCamera: ...
