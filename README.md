# G1_VLN —— LightNav-0 视觉语言导航（Thor 上，与导航栈互斥）

```
G1_VLN/
  LightNav-0/   LightNav-0 代码（c6f40e3，原样）
  server/       推理服务脚本（start_server.sh / stop_server.sh，REPO_DIR 指向上面的 LightNav-0）
  client/       机器人端：start_vln.sh、vln_ros2_bridge.py、网页 vln_web_ui.py、vln_cmdvel.yaml
  deploy.sh     工作站 → Thor 同步（rsync 到 /home/unitree/G1_VLN）
```

Thor 上 venv/缓存/模型沿用 `~/lightnav/.venv`、`~/lightnav/.cache`、`~/models/LightNav-0`。

## 链路

```
USB 双目相机 ─usb_camera_node─▶ /vln/camera/color/image_raw
      │
vln_ros2_bridge.py ⇄ ws://127.0.0.1:8050（LightNav 服务）
      │  每次推理：10 个航点（base_link，拍照时刻）+ 拍照时间戳 → /vln/response
      ▼
vln_mpc（官方，/vln 命名空间）◀── /vln/odom ◀─ sport_to_odom（g1_nav_bridge，独立实例）◀─ /odommodestate
      │  10 Hz 跟踪整条轨迹 → /vln/mpc/cmd_vel
      ▼
vln_ros2_bridge.py 闸门（授权 / 急停 / 推理断流 / MPC 断流 / 连续 2 帧 stop 才算到达）
      │  安全模式 → /vln/cmd_vel（无人订阅）
      ▼  --robot   → /vln/cmd_vel_robot → cmdvel_to_sport（g1_vln_cmdvel_to_sport）→ G1
```

与导航栈完全独立：不用 /odom、Nav2、D435、地图和定位，导航栈开不开都不影响（真机模式要求导航栈已停，避免两边同时给 G1 发速度）。
`--ctrl wp0` 可退回旧行为（只用第 1 个航点换算速度，会丢掉后段的转向和减速）。
MPC 参数在 `client/vln_mpc_g1.yaml`（vx 0~0.45 m/s、|wz|≤0.9 rad/s，取自导航栈在 G1 上的实测限速）。
网页：`http://192.168.128.146:8090/`。
网页上有【🎯 去目标】/【🏃 跟人】两个预设按钮，运行中点一下就同时切换 MPC 模式（objnav / track）和指令，不用重启（会重置本轮，需重新授权）。
命令行等价：`./start_vln.sh --all --robot --follow "follow the person in ..."`；
运行中只想换指令：`./tools/vln_ctl.py instruction "..."`（会重置本轮，仍未授权）。

## 用法（Thor 上）

```bash
cd ~/G1_VLN/client
./start_vln.sh --all "go to the red chair"           # 安全模式：不发给机器人，可与导航栈同时跑
./start_vln.sh --all --robot "go to the red chair"   # 真机：要求导航栈已停（stop_all.launch.py）；测试·单步，网页点【确认执行】才走
./start_vln.sh --stop                                # 只停 VLN 自己的相机/桥接/转发
../server/stop_server.sh                             # 停推理服务
```
相机选项：`--cam-device /dev/video6 --cam-size 2560x720 --cam-eye left|right --cam-rotate 0|180`（倒装用 180）。
推理服务没在跑时 `start_vln.sh` 会自动起（约 2~3 分钟，vln 任务）。

服务的 `TASK`（vln / tracking）对识别没有影响：这个检查点里两者的 eval_config 完全相同，2026-10-06 同帧对比结果逐项一致。
跟人、去物体都用默认 vln 即可，关键在指令措辞（例：`follow the person in the white t-shirt` 能认出，`go to the person in white clothes` 认不出）。
搭着红衣服的白凳子（4.5 m）：说 chair 的各种写法几乎都认不出（衣服挡住了凳子形状），
`Go to the red object in front and stop.` 8/8 认出且定位准 —— 按"看起来是什么"描述目标，而不是按它实际是什么。

## 注意

- 相机必须水平朝前固定；画面是否倒置固定后确认，倒置用 `--cam-rotate 180`。
- 真机时没有 collision_monitor / Nav2 避障，只有 G1 本体运控。先用测试·单步，旁边有人守急停。
- 恢复导航：`./start_vln.sh --stop`，再 `ros2 launch robot_bringup robot.launch.py`。
