import time

import pytest
from vln_mujoco.robots.base import RobotState
from vln_mujoco.robots.turtlebot import TurtleBotBackend
from vln_mujoco.simulation import COMMAND_TIMEOUT_S, Simulation


class TelemetryTurtleBot(TurtleBotBackend):
    def state(self) -> RobotState:
        state = super().state()
        return RobotState(
            pose=state.pose,
            velocity=state.velocity,
            telemetry={"pose": "backend-pose", "policy_command": {"linear": 0.3}},
        )


def test_turtlebot_backend_integrates_velocity_command() -> None:
    robot = TurtleBotBackend()
    start = robot.state()

    for _ in range(20):
        robot.step((0.4, 0.2))

    state = robot.state()
    assert state.pose[0] > start.pose[0]
    assert state.pose[1] > start.pose[1]
    assert state.pose[3] > start.pose[3]
    assert state.velocity == pytest.approx((0.4, 0.2))


def test_turtlebot_backend_reset_restores_spawn_pose() -> None:
    robot = TurtleBotBackend()
    robot.step((0.4, 0.2))

    robot.reset()

    state = robot.state()
    assert state.pose == pytest.approx((6.5, 13.8, 0.033, 0.0))
    assert state.velocity == pytest.approx((0.0, 0.0))


def test_simulation_owns_command_watchdog_and_robot_identity() -> None:
    simulation = Simulation(TurtleBotBackend())
    simulation.set_velocity(0.4, 0.2)
    simulation._advance(time.monotonic())

    assert simulation.robot_name == "TurtleBot"
    assert simulation.snapshot()["command"] == pytest.approx(
        {"linear": 0.4, "angular": 0.2}
    )

    simulation._command_at = time.monotonic() - COMMAND_TIMEOUT_S - 0.1
    simulation._advance(time.monotonic())

    assert simulation.snapshot()["command"] == pytest.approx(
        {"linear": 0.0, "angular": 0.0}
    )


def test_simulation_namespaces_backend_telemetry() -> None:
    simulation = Simulation(TelemetryTurtleBot())

    snapshot = simulation.snapshot()

    assert isinstance(snapshot["pose"], dict)
    assert snapshot["backend"] == {
        "pose": "backend-pose",
        "policy_command": {"linear": 0.3},
    }
