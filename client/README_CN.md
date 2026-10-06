# 机器人端部署说明（宇树双足）

本目录是 **LightNav-0 机器人侧** 的独立客户端，零 ROS 依赖，可直接拷到机器人上跑。
服务端（`lightnav-serve`）已经在 GPU 主机上跑起来了，见文末「服务端现状」。

> ⚠️ **重要前提**：仓库自带的机器人适配器只有 **Unitree Go2（四足，`go2_adapter`）**
> 和 **LimX TRON1（`tron_adapter`）**，**没有宇树双足（G1 / H1）的适配器**。
> 所以宇树双足这条路线需要按本文档的「宇树双足适配」一节接一下，两种方案都给了。

---

## 1. 网络准备

服务端已监听 `0.0.0.0`，机器人要能访问 GPU 主机：

| 服务 | 地址 | 任务模式 |
|---|---|---|
| **vln**（指令导航，如「去客厅的沙发」） | `ws://192.168.110.204:8050` | 输出走向目标点的航点 |
| **tracking**（跟随目标，如「跟随穿红衣服的人」） | `ws://192.168.110.204:8051` | 额外返回 `visible` 可见性标志 |

先在机器人上确认网络通：

```bash
# 1) 能 ping 通
ping -c 3 192.168.110.204

# 2) TCP 端口开（两个都要）
timeout 3 bash -c 'cat < /dev/null > /dev/tcp/192.168.110.204/8050' && echo "8050 OK"
timeout 3 bash -c 'cat < /dev/null > /dev/tcp/192.168.110.204/8051' && echo "8051 OK"
```

GPU 主机侧已确认：`ufw` 未启用，无防火墙拦截，两个端口在局域网 IP 上均可连。

> 💡 机器人侧**不需要 GPU、不需要 torch**，只需要 `websockets` + `Pillow` + `numpy`。
> 总依赖不到 50 MB，Jetson Orin / 宇树板载 x86 都能跑。

---

## 2. 机器人端安装

```bash
# 建议用 venv，别污染系统 python
python3 -m venv ~/vln_env
source ~/vln_env/bin/activate

pip install "websockets>=12" Pillow numpy

# 按你的相机二选一（或都装）
pip install pyrealsense2             # Intel RealSense D435/D455
pip install opencv-python-headless   # USB / V4L2 摄像头、以及回放 mp4 用
```

宇树双足还要装官方 SDK（**只在要驱动真机时才需要**）：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python

# G1 / H1 属于 unitree_hg 系列，还需要 CycloneDDS（注意版本）
pip install cyclonedds==0.10.2
pip install -e .

# 验证 import 成功
python3 -c "from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; print('G1 SDK OK')"
```

把客户端拷到机器人：

```bash
scp vln_robot_client.py requirements-robot.txt  unitree@<机器人IP>:~/vln/
```

---

## 3. 三步验证法（务必按顺序）

### 第 1 步：不接机器人、不接相机，只验网络与推理链路

用几帧图片喂进去，看能不能拿到航点。**这一步通了才继续往下。**

```bash
python3 vln_robot_client.py \
    --server ws://192.168.110.204:8050 \
    --camera dir --frames ./test_frames \
    --instruction "go to the red chair" \
    --robot dry_run --no-loop
```

期望输出（实测基准，`dry_run` 实测往返 **平均 141 ms / P95 159 ms**）：

```
[10:09:23] 连接服务端 ws://192.168.110.204:8050 …
[10:09:23] login OK (clientId=robot-01)
[10:09:23] reset OK —— 新 episode 开始
[10:09:23] ═══ 开始闭环：推理 4.0 Hz，下发 20.0 Hz，指令「go to the red chair」═══
[10:09:24] seq=  3  rtt= 128.4ms  srv= 119.1ms  wp0=[+0.150 +0.010 +0.097]  cmd=(vx+0.200 vy+0.000 wz+0.492)  stop=False vis=False
[10:09:24]        apos=point (clamped)  opos=not_visible
[10:09:24]   → dry_run 下发  vx=+0.201 m/s  vy=+0.000 m/s  wz=+0.000 rad/s
...
```

读到什么算什么：

| 字段 | 含义 |
|---|---|
| `rtt` | 本机→服务端往返；`srv` 是服务端自报的模型耗时。两者差 ≈ 网络+传输 |
| `wp0` | 模型给出的**第一个**航点 `[前进 m, 横移 m, 偏航 rad]`，后 9 行没打印（`--log-dir` 的 jsonl 里有全部信息） |
| `cmd` | 经换算+限速后实际要下发的速度 |
| `stop` | 模型判定已到达目标 |
| `vis` | 跟踪任务的可见性标志（vln 任务为 `False`） |
| `apos` / `opos` | **像素级状态**：`apos`=目标在当前画面的去向指示（`rot_left` 表示该左转）；`opos`=`not_visible` / `point` / `clamped`（目标超出画面被裁剪，说明已经偏得很远）。跟踪任务现场调试主要看这两个 |

> 开局前几帧模型会给出「原地小转向」或零速，是正常的：历史窗还没填满。
> 实测第 4 帧才开始正常前进。

没有素材？随便拷 10 张同场景的照片进 `test_frames/` 即可（支持 jpg/png）。

### 第 2 步：接上相机，仍然 `dry_run`

确认相机能出图、帧率跟得上：

```bash
# RealSense
python3 vln_robot_client.py --server ws://192.168.110.204:8050 \
    --camera realsense --width 640 --height 480 \
    --instruction "go to the red chair" --robot dry_run

# USB / V4L2 摄像头（也适用于 UVC 模式的 Orbbec Gemini）
python3 vln_robot_client.py --server ws://192.168.110.204:8050 \
    --camera v4l2 --device /dev/video0 \
    --instruction "go to the red chair" --robot dry_run
```

**把机器人举起来（轮子/腿离地），看 `dry_run` 打印的速度方向对不对**，
这一步是零风险的，但能验证：
- 前进对应 `vx>0`
- 说「左转」时 `wz>0`
- 说「向前走」时 `vx>0` 且 `wz≈0`

### 第 3 步：上真机

```bash
# 先让机器人站立/进入主运控状态（或用官方遥控器进入「主运控」模式）
python3 vln_robot_client.py \
    --server ws://192.168.110.204:8050 \
    --camera realsense \
    --robot unitree_g1 --iface eth0 \
    --instruction "go to the red chair" \
    --max-vx 0.3 --max-wz 0.5 \
    --max-runtime 60
```

**首跑务必**：`--max-vx 0.3`（保守）、`--max-runtime 60`（自动收尾）、
手放在官方遥控器的急停上、机器人周围留足 2 m 空间、最好有吊带。

运行中可用热键：`空格` 暂停/恢复（暂停时持续下发零速），`q` 退出，`Ctrl-C` 同理。

---

## 4. 宇树双足适配

### 方案 A：用本目录的独立客户端（推荐先走这条）

已实现 `--robot unitree_g1` / `--robot unitree_h1`，走官方 SDK 的高层运动接口：

```
ChannelFactoryInitialize(0, iface)          # iface = 与机器人相连的网卡
client = LocoClient(); client.SetTimeout(10.0); client.Init()
client.Start() / StandUp()                  # 站立（或遥控器手动进入主运控）
client.Move(vx, vy, vyaw)                   # 持续速度指令，本客户端 20 Hz 下发
client.StopMove() / client.Damp()           # 停止 / 阻尼
```

代码在 `vln_robot_client.py` 的 `UnitreeHumanoidRobot` 类，约 120 行。
**不同 SDK 版本存在差异**（下面这些点已经在代码里做了多路回退，但仍需你确认）：

| 不确定项 | 代码里的处理 | 你要确认什么 |
|---|---|---|
| `Move()` vs `SetVelocity()` | 优先 `SetVelocity`，没有则 `Move` | 跑一次看是否报 `AttributeError` |
| `Move(vx,vy,vyaw,continous_move)` 尾参 | 先试 3 参，`TypeError` 再试 4 参 | 若机器人**走走停停**，说明它按「单次定长移动」执行了，需改成 `continous_move=True` 或改用 `SetVelocity` |
| 站立接口 `Start`/`StandUp`/`HighStand` | 依次尝试 | 用 `--no-stand` 跳过，改由遥控器进入主运控 |
| 偏航正负号 | `--yaw-sign 1` | 若左转打成右转，改 `--yaw-sign -1` |
| 横移正负号 | `--lat-sign 1` | 若横移反向，改 `--lat-sign -1` |

**双足平台的关键取舍（默认值已按保守设定）**：

- **`--max-vy` 默认 0**。人形机器人横移（侧向走）稳定性差、很多固件根本不支持，
  且模型的横移分量本来就很小（实测 0.000~0.01 m）。默认直接丢掉横移。
- **`--max-wz 0.8`**：偏航角速度给足（模型转向意图通常很强，实测归一化后经常饱和到 1.0），
  但双足急转容易失稳，可按场地调低到 0.5。
- **`--max-vx 0.5`**：模型单步最大前进 0.375 m，归一化后基本不饱和；
  真机上先用 0.3 跑。
- **`--cmd-timeout 0.6`**：超 0.6 s 没拿到新航点就归零。服务端实测延迟 ~150 ms，
  预留了 4 倍余量。网络差就调大，但别超过 1.0 s。
- **底盘层还有独立看门狗**（`--watchdog-s 0.4`），在 `_watchdog_loop` 里，
  即使控制线程卡死也会主动归零。

### 方案 B：写一个 `g1_adapter` 接进仓库的 ROS 2 全栈

想要仓库那套 **web 控制面板 + MPC 航点跟踪 + 录制** 的完整体验，
就得按 `robot_deploy/README.md` 的「Bring your own robot」写一个适配器包，
然后复制 `tron.launch.py` 换成你的适配器。

需要实现的**共同接口**（`go2_adapter` 和 `tron_adapter` 是两个参考实现）：

| 方向 | 话题 / 服务 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `web/cmd_vel`, `mpc/cmd_vel` | `geometry_msgs/TwistStamped` | 按控制源仲裁，转发当前生效的那个到底盘 |
| 发布 | `odom` | `nav_msgs/Odometry` | `odom → base_link`，**时间戳必须真实**（MPC 靠它对齐航点） |
| 发布 | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | 连接、模式、电量、IMU、电机状态，喂给网页 |
| 发布 | `control/source` | `std_msgs/String` | `disabled` / `manual` / `auto` |
| 发布 | `robot/events` | `std_msgs/String` | 自由格式事件日志 |
| 服务 | `control/set_manual`, `control/set_auto`, `control/stop` | `std_srvs/Trigger` | |
| 服务 | `robot/stand`, `robot/walk`, `robot/sit`, `robot/emergency_stop` | `std_srvs/Trigger` | |

**安全责任（必须由适配器承担）**：
- **命令看门狗**：指令过期就停（`tron_adapter` 用 0.35 s 并发一串零速帧）
- **停止行为**
- 控制源为 `disabled` 时，**把控制权交还机器人自己的遥控器**

起步方式：把 `go2_adapter` 整个目录拷成 `g1_adapter`，然后改这三处：
1. `unitree_client.py` — 把 `SportClient` / `ObstaclesAvoidClient`（`unitree_go` 系列，四足）
   换成 `LocoClient`（`unitree_hg` 系列，人形）
2. `adapter_node.py` — 状态话题从 `rt/lf/sportmodestate` 换成你固件实际的话题名
3. `package.xml` / `setup.py` — 改名并加上 `unitree_sdk2py` 依赖

---

## 5. 参数速查

### 航点换算

模型每行输出 `[forward_m, lateral_m(+左), yaw_rad(+逆时针)]`，**自身没有时间基准**。
两种换算：

| 模式 | 公式 | 什么时候用 |
|---|---|---|
| `--mapping normalized`（默认） | `n = clip(wp / [0.375, 0.25, π/20], -1, 1)`，再乘限速 | 与仓库官方客户端一致，换平台不用重调，**推荐** |
| `--mapping direct` | `v = wp / dt`，再 clip 到限速 | 你确定 `dt` 与模型训练的单步时间一致时 |

`0.375 / 0.25 / π/20` 是轨迹词表的单步极限（约 0.25 m 平移、30° 偏航的保守估计），
与 `src/lightnav/cli/ws_client.py` 中的常量一致。

### 节拍

| 参数 | 默认 | 含义 |
|---|---|---|
| `--fps` | 4 | **推理**帧率，等于 checkpoint 的 `video_fps`。发得越多历史窗填得越快 |
| `--control-hz` | 20 | **下发**帧率，零阶保持。推理慢不影响底盘平滑 |
| `--cmd-timeout` | 0.6 | 指令保持时长，超时归零 |
| `--warmup-frames` | 0 | 开局用空 `instruction` 预填历史窗（不推理，只入缓冲） |

> 关于 `--warmup-frames`：服务端的帧缓冲默认保留 64 帧（`num_history_frames`），
> 按 4 fps 就是 16 秒历史。开局只有 1 帧时模型输出会偏保守（实测前 3 帧都是
> 「原地小幅转向」或零速，第 4 帧才开始正常前进）。
> 想让第一帧就大胆走，可以设 `--warmup-frames 16`（把当前画面重复填进去，
> 约 4 秒开销）；但真实场景下让历史自然累积更符合模型训练分布。

---

## 6. 故障排查

| 现象 | 原因与处理 |
|---|---|
| `ConnectionRefusedError` | 服务端没起 / 端口不对。GPU 主机上 `ss -ltnp \| grep 805` 确认 |
| `login 失败：{'rc': 400...}` | 协议版本不符，确认和服务端是同一份代码 |
| 连上但一直 `image received` | 你发的是空 `instruction`。检查是不是误用了 `--warmup-frames` 而没进主循环 |
| `rc=500` 反复出现 | 模型输出无法解码（航点超出词表范围）。看服务端日志，通常是输入图像异常（全黑/损坏） |
| 往返延迟 > 500 ms | 网络慢或服务端排队。检查 `timings_ms.queue_wait_ms`；同 GPU 上连的机器人太多了就加 `--max_batch_size` |
| 机器人**走走停停** | 宇树 `Move()` 被当成「单次定长移动」执行了。改用 `SetVelocity`，或 `continous_move=True`（见「宇树双足适配」） |
| 转向反了 | `--yaw-sign -1` |
| 横移反了 | `--lat-sign -1` |
| 机器人不动但日志正常 | 底盘没进入可运动状态。用遥控器进「主运控」模式，或检查 `--iface` 网卡名对不对 |

---

## 7. 服务端现状（GPU 主机 `ajx-X660-G45`）

已启动并验证：

| 服务 | 监听 | GPU | 任务 | 日志 | 录制 |
|---|---|---|---|---|---|
| vln | `0.0.0.0:8050` | GPU2 | 指令导航 | `logs/servers/vln_gpu2_8050.log` | `output/episodes/vln/` |
| tracking | `0.0.0.0:8051` | GPU4 | 目标跟随 | `logs/servers/tracking_gpu4_8051.log` | `output/episodes/tracking/` |

两个服务都用了 `--ready_file`，出现 `logs/servers/*.ready` 才算真正就绪
（服务端绑定端口前会先做一次预热推理，所以「端口开了」≈「引擎好了」）。

停止服务：

```bash
pkill -f "lightnav-serve"
```

重启（在 GPU 主机上）：

```bash
source /data/wanglei/miniconda3/etc/profile.d/conda.sh && conda activate lightnav
cd /data/wanglei/VLN/LightNav-0
nohup env CUDA_VISIBLE_DEVICES=2 lightnav-serve \
    --task vln --model_path "$PWD/checkpoints/LightNav-0" \
    --backend vllm_local --gpu_memory_utilization 0.85 \
    --host 0.0.0.0 --port 8050 \
    --ready_file "$PWD/logs/servers/vln_8050.ready" \
    --record_dir "$PWD/output/episodes/vln" \
    --cam_hfov_deg 90 --cam_height 0.6 \
    > logs/servers/vln_gpu2_8050.log 2>&1 &
```

> ⚠️ `--gpu_memory_utilization` 是**占整卡显存的比例**。GPU4 上有别的任务，
> 只剩 62 GiB，0.85 × 79 GiB = 67 GiB 会启动失败，所以 tracking 用的 0.70。
> 换机器/换卡时先 `nvidia-smi` 看剩余显存再定这个值。

### 看模型在机器人自己的画面上预测了什么

服务端每次会话都会自动录制（`--record_dir`），机器人跑完一段之后在 GPU 主机上渲染：

```bash
cd /data/wanglei/VLN/LightNav-0
source /data/wanglei/miniconda3/etc/profile.d/conda.sh && conda activate lightnav

lightnav-render output/episodes/vln              # 整个 tree，每个 episode 各出一段
lightnav-render output/episodes/vln/run_*/robot-01/episode_000 --height 1080 --overwrite
```

输出 `<episode>/traj_pointing.mp4`，画面上叠加了预测轨迹带、目标像素点（apos/opos）、
以及指令 / GO-STOP / 步数 / 首航点速度的 HUD。**这是远程调试机器人最常见的手段**——
机器人在现场跑，你在 GPU 主机上看它的视角和预测，不用来回搬设备。

录制目录结构：

```
output/episodes/vln/run_<服务启动时间>/<clientId>/episode_000/
    image_000001.jpg …      ← 机器人送来的原始帧（12 次预测就是 12 张）
    actions.json            ← 每步的原始航点、visibility、latency
    traj_pointing.mp4       ← 渲染产物（渲染帧数 ≠ 录制帧数，会按 realtime 插值）
```

渲染的**投影几何**（水平 FOV、相机离地高度）不通过命令行传，而是服务端启动时的
`--cam_hfov_deg 90 --cam_height 0.6` 写进 manifest 的。**换机器人时这两个值一定要改**，
否则轨迹带画的位置会偏：它们决定「预测的前方 0.375 m 落在画面哪一行」。

其它常用渲染参数：

| 参数 | 用途 |
|---|---|
| `--height 1080` | 放大输出分辨率，便于肉眼看细节 |
| `--no-pointing` | 只留轨迹带，去掉像素标记 |
| `--no-hud` | 只留轨迹带+标记，不要遥测浮层 |
| `--timeline per_step` | 每步等长，用于逐帧慢看（默认 `realtime` 按真实耗时） |
| `--fps 10` | 覆盖帧率（默认取 manifest 的 `video_fps`） |
| `--min-steps 5` | 跳过过短的 episode |
