#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线图片回放成 ROS 2 图像话题
================================================================================

相机还没插 / RealSense 没起来时，用它把 `test_frames/` 里的图片当成
`realsense2_camera` 的彩色流发出来，好把「图像 → 推理 → cmd_vel」整条 ROS 2
链路先跑通、跑顺。

它**只发布图像**，不发布任何速度，也不订阅任何东西 —— 本身无风险。

    # 终端 A：假装是相机
    python3 tools/fake_rs_pub.py --frames ./test_frames --fps 15

    # 终端 B：桥接节点（先纯观察）
    python3 vln_ros2_bridge.py --instruction "go to the red chair"

    # 核对话题确实有数据
    ros2 topic hz /camera/camera/color/image_raw
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


class FakeRsPub(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("fake_rs_pub")
        self.a = a
        self.pub = self.create_publisher(Image, a.topic, 10)

        files = sorted(p for p in Path(a.frames).iterdir() if p.suffix.lower() in _EXTS)
        if not files:
            raise SystemExit(f"{a.frames} 里没有图片（支持 {sorted(_EXTS)}）")

        from PIL import Image as PILImage

        self._frames: list[np.ndarray] = []
        for p in files:
            im = PILImage.open(p).convert("RGB")
            if a.width and a.height:
                im = im.resize((a.width, a.height))
            self._frames.append(np.ascontiguousarray(np.asarray(im)))

        h, w = self._frames[0].shape[:2]
        self.get_logger().info(
            f"载入 {len(self._frames)} 帧 {w}x{h}，将以 {a.fps:.1f} fps 发到 {a.topic}"
        )
        self._idx = 0
        self._timer = self.create_timer(1.0 / a.fps, self._tick)
        self.create_timer(5.0, self._heartbeat)
        self._n = 0

    def _tick(self) -> None:
        if self._idx >= len(self._frames):
            if not self.a.loop:
                self.get_logger().info("回放完毕，停止发布")
                self._timer.cancel()
                return
            self._idx = 0

        arr = self._frames[self._idx]
        self._idx += 1
        h, w = arr.shape[:2]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.a.frame_id
        msg.height, msg.width = h, w
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = arr.tobytes()
        self.pub.publish(msg)

        self._n += 1
        if self._n == 1:
            self.get_logger().info("已发出第一帧")

    def _heartbeat(self) -> None:
        if self.count_subscribers(self.a.topic) == 0:
            self.get_logger().warn(
                f"{self.a.topic} 上还没有订阅者；桥接节点起了吗？"
            )
        else:
            self.get_logger().info(
                f"已发 {self._n} 帧，订阅者 {self.count_subscribers(self.a.topic)} 个"
            )


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_argv: list[str] = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_argv = argv[:i], argv[i:]

    p = argparse.ArgumentParser(
        prog="fake_rs_pub.py",
        description="把图片目录回放成 ROS 2 sensor_msgs/Image，用于无相机时验证链路",
    )
    p.add_argument("--frames", default="./test_frames", help="图片目录")
    p.add_argument("--topic", default="/camera/camera/color/image_raw",
                   help="要与桥接节点 --image-topic 一致")
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--width", type=int, default=0, help="0=保持原尺寸")
    p.add_argument("--height", type=int, default=0)
    p.add_argument("--frame-id", default="camera_color_optical_frame")
    p.add_argument("--no-loop", action="store_true", help="播完就停（默认循环）")
    a = p.parse_args(argv)
    a.loop = not a.no_loop

    rclpy.init(args=ros_argv)
    node = FakeRsPub(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
