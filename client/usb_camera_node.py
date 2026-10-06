#!/usr/bin/env python3
"""免驱 USB 双目相机（并排 MJPG）→ ROS 2 Image。

取并排大图的左半边，发布 rgb8 到 /vln/camera/color/image_raw。
用 ffmpeg 解码裁剪缩放（Thor 的 venv 里没有 cv2），不额外依赖。
  python usb_camera_node.py --device /dev/video6 --size 2560x720 --out-width 640 --eye left
"""
import argparse
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/v4l/by-id/usb-STYT_241029_K_USB_Camera_01.00.00-video-index0")
    ap.add_argument("--size", default="2560x720", help="并排大图尺寸，左右各占一半")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out-width", type=int, default=640, help="输出宽，高按 16:9 等比")
    ap.add_argument("--eye", choices=["left", "right"], default="left")
    ap.add_argument("--rotate", choices=["0", "180"], default="0", help="相机倒装时用 180")
    ap.add_argument("--topic", default="/vln/camera/color/image_raw")
    ap.add_argument("--frame-id", default="vln_camera_color_optical_frame")
    a = ap.parse_args()

    W, H = (int(x) for x in a.size.split("x"))
    hw = W // 2
    ow = a.out_width
    oh = int(round(ow * H / hw / 2)) * 2
    vf = f"crop={hw}:{H}:{0 if a.eye == 'left' else hw}:0,scale={ow}:{oh}"
    if a.rotate == "180":
        vf += ",hflip,vflip"
    cmd = ["ffmpeg", "-loglevel", "error", "-f", "v4l2", "-input_format", "mjpeg",
           "-video_size", a.size, "-framerate", str(a.fps), "-i", a.device,
           "-vf", vf, "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]

    rclpy.init()
    node = Node("vln_usb_camera")
    pub = node.create_publisher(Image, a.topic, qos_profile_sensor_data)
    log = node.get_logger()
    log.info(f"{a.device} {a.size} → {a.eye} {ow}x{oh} rgb8 → {a.topic}")

    nbytes = ow * oh * 3
    n, t0 = 0, time.time()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
    try:
        while rclpy.ok():
            buf = bytearray()
            while len(buf) < nbytes:
                chunk = proc.stdout.read(nbytes - len(buf))
                if not chunk:
                    # 2026-10-06 实测：机器人走动时 hub 报 "disabled by hub (EMI?)" 掉线。
                    # 不退出，每秒重开一次，设备回来就自动恢复（期间不出图，桥接按图像过期发零速）。
                    proc.wait()
                    log.error("ffmpeg 退出（设备被占用或拔出？），1 s 后重连")
                    time.sleep(1.0)
                    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
                    buf = bytearray()
                    continue
                buf += chunk
            m = Image()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = a.frame_id
            m.height, m.width, m.encoding = oh, ow, "rgb8"
            m.step, m.data = ow * 3, bytes(buf)
            pub.publish(m)
            n += 1
            if n % 150 == 0:
                log.info(f"{n / (time.time() - t0):.1f} fps")
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
