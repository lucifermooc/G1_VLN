# VLN × ROS 2 桥接（LightNav-0 → G1）

把相机（ROS 2 话题）和运动控制（ROS 2 话题）接进 VLN 推理闭环，替代原来
`vln_robot_client.py` 里「pyrealsense2 直连 + unitree_sdk2py 直连」的私有路径。

```
/camera/camera/color/image_raw   (sensor_msgs/Image)
         │
         ▼
  vln_ros2_bridge.py ──ws──► 192.168.110.204:8050  (LightNav-0 推理)
         │                         │
         │  ◄── [fwd, lat, yaw] ───┘
         ▼
   /vln/cmd_vel  (geometry_msgs/Twist)   ← 默认发这里，安全，无人订阅
         │
         │  （真机阶段才接上）
         ▼
   /cmd_vel_safe → cmdvel_to_sport → G1LocoClient → G1
```

---

## 1. 文件说明

| 文件 | 作用 |
|---|---|
| `vln_ros2_bridge.py` | 主桥接节点：订阅相机 → 调服务端 → 发布 Twist |
| `vln_web_ui.py` | 网页控制台：实时画面 + 推理结果 + 「执行/暂停/急停」按钮 |
| `tools/fake_rs_pub.py` | 回放一个图片目录为 `sensor_msgs/Image`，无相机时联调用 |
| `setup_vln_ros2.sh` | 一键 source 环境（**必须 source**，会设置 `$VLN_PY`） |
| `vln_robot_client.py` | 原版客户端（pyrealsense2 直连），保留作参考/对照 |
| `test_frames/` | 28 张离线测试帧 |
| `logs/ros2_bridge/*.jsonl` | 每次运行的结构化日志 |

---

## 2. 三步走

### 步骤 0：环境

```bash
cd ~/vln && source ./setup_vln_ros2.sh
```

会依次 source ROS Jazzy → CycloneDDS 工作空间 → `lighting_ws`，并设置
`$VLN_PY=~/vln_env/bin/python`。看到末尾的汇总横幅即成功。

**RMW 默认已改为 `rmw_fastrtps_cpp`**，原因见 [第 5.5 节](#55-踩过的大坑cyclonedds-会让相机彻底僵死)，
这是让相机稳定出帧的关键。想切回 CycloneDDS 用：

```bash
VLN_RMW=rmw_cyclonedds_cpp source ./setup_vln_ros2.sh
```

> ⚠️ **相机与本桥接必须在同一个 RMW 下**，否则各看各的 Domain，桥接永远
> 等不到第一帧。改 RMW 时要两边一起改。

### 步骤 1：起相机

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  serial_no:="'347622073141'" \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=false \
  publish_tf:=true \
  rgb_camera.color_profile:=640x480x30
```

看到 `RealSense Node Is Up!` 且 `Open profile: ... Color(0) ... 640, Height: 480` 即可。

> ❗ **两处和同事给的原命令不同，都不是可选项：**
>
> 1. **`pointcloud.enable:=false`** —— 原命令带的 `config_file` 里写了
>    `pointcloud__neon_.enable`，但当前版本的 `realsense2_camera` **已不支持该参数**
>    （启动日志会警告 `is not supported`），点云退化成纯 CPU 计算，会把整个相机
>    节点拖到 0.8 fps 甚至完全不出图。
> 2. **`rgb_camera.color_profile:=640x480x30`** —— VLN 只吃彩色图，640 足够，
>    带宽少一半，服务端延迟从 ~289 ms 降到 ~192 ms。
>
> 实测（同一台机器，每次都是新起节点）：
>
> | 配置 | color | depth | points |
> |---|---|---|---|
> | 只开 color | 27.3 fps | – | – |
> | color + depth，关点云 | **29.5 fps** | **30.7 fps** | – |
> | 开点云（原命令） | 0.8 ~ 3.4 fps | 0 帧 | 0 帧 |

### 步骤 2：纯观察模式跑桥接（**默认不发布任何速度**）

```bash
$VLN_PY vln_ros2_bridge.py --instruction "go to the red chair" --encode-width 640
```

日志会逐帧打印：

```
[..] seq=  7  rtt= 187.4ms  srv= 124.8ms  wp0=[+0.149 -0.021 -0.200]  cmd=(vx+0.198 vy-0.000 wz-0.800)  stop=False vis=False  → 拟下发
[..]        apos=point (clamped)  opos=not_visible
```

`→ 拟下发` 表示只计算没发布。

### 步骤 3：使能发布，用 `ros2 topic echo` 核对

```bash
# 终端 B
$VLN_PY vln_ros2_bridge.py --instruction "go to the red chair" --publish --encode-width 640

# 终端 C
ros2 topic echo /vln/cmd_vel --field linear
```

`/vln/cmd_vel` **没有任何订阅者**，所以这一步不会让机器人动。

### 步骤 4（推荐）：开网页控制台

在步骤 3 的命令上再加一个 `--web`，就能在浏览器里看画面、看推理结果、
然后自己决定要不要真的执行：

```bash
$VLN_PY vln_ros2_bridge.py --instruction "go to the red chair" \
    --publish --web --encode-width 640
```

打开 `http://127.0.0.1:8090/`（或局域网 IP）。**默认未授权**，
不点「执行」就不会下发任何非零速度。详见[第 6 节](#6-网页控制台推荐用法)。

---

## 3. 安全设计

- **默认话题是 `/vln/cmd_vel`，不是 `/cmd_vel`。** 除非显式指定，否则不可能
  误接上真实控制链路。
- **不加 `--publish` 就完全不创建发布器**，从代码层面杜绝误发。
- 输出话题若是 `/cmd_vel` 或 `/cmd_vel_safe`，启动横幅会**大声告警**。
- 推理线程异常、连续服务端错误、图像过期、`stop=true`
  → 控制定时器立刻归零（`--cmd-timeout`，默认 0.6 s）。
- 退出时连发 10 帧零速（在推理线程的 `finally` 里发，此时上下文仍有效）。

### 真机接线（**现在先别做**）

确认无误后再改这两处：

```bash
# 1) 桥接输出改到安全话题，保留 collision_monitor
$VLN_PY vln_ros2_bridge.py --instruction "..." \
    --publish --cmd-vel-topic /cmd_vel_safe --encode-width 640

# 2) 另开一个终端，把速度送到机器人
ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
    -p cmd_vel_topic:=/cmd_vel_safe -p duration:=0.5
```

> ⚠️ **不要**直接发 `/cmd_vel`。README 记录的完整链路是
> `Nav2 → /cmd_vel_nav → velocity_smoother → /cmd_vel → collision_monitor
> → /cmd_vel_safe → cmdvel_to_sport → G1LocoClient → G1`，
> 发 `/cmd_vel` 会绕过碰撞监控。
>
> ⚠️ `cmdvel_to_sport` **没有看门狗**。它只缓存最新一条 Twist，然后调
> `SetVelocity(vx, vy, wz, duration)`，机器人靠 `duration`（默认 0.5 s）到期自停。
> 所以**客户端下发频率必须大于 1/duration**（默认 `--control-hz 20` 足够）。

---

## 4. 主要参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--instruction` | （必填） | 指令，如 `"go to the red chair"` |
| `--server` | `ws://192.168.110.204:8050` | 推理服务端 |
| `--image-topic` | `/camera/camera/color/image_raw` | 相机彩色话题 |
| `--cmd-vel-topic` | `/vln/cmd_vel` | 速度话题，**安全默认** |
| `--publish` | 关 | 不加则纯观察 |
| `--encode-width` | `0`（不缩） | 送服务端前缩到该宽度（保持宽高比）。**建议 640** |
| `--fps` | `4.0` | 推理帧率（模型训练帧率） |
| `--control-hz` | `20.0` | 速度下发频率 |
| `--cmd-timeout` | `0.6` | 指令保持时长，超过归零 |
| `--max-vx / --max-vy / --max-wz` | `0.5 / 0.0 / 0.8` | 限速（双足横移建议 0） |
| `--max-image-age` | `1.0` | 超过这么久没新图就不下发 |
| `--max-runtime` | `0`（不限） | 最长运行秒数 |
| `--yaw-sign` / `--lat-sign` | `1.0` | 转向/横移方向反了就改 `-1` |
| `--log-dir` | `logs/ros2_bridge` | 结构化日志目录 |
| `--web` | 关 | 开网页控制台（见第 6 节） |
| `--web-host` | `0.0.0.0` | 监听地址 |
| `--web-port` | `8090` | 监听端口，被占会自动向后试 11 个 |
| `--start-armed` | 关 | 启动即授权（不加则必须网页上点一下才会发速度） |
| `--approve-hold-s` | `1.0` | 「单步」按钮的授权时长 |
| `--stop-hold-s` | `0.5` | 暂停/急停时补发零速的持续时间 |

也支持 ROS 风格覆盖：

```bash
$VLN_PY vln_ros2_bridge.py --instruction "go" --ros-args -p max_vx:=0.3 -p fps:=2.0
```

---

## 5. 性能：为什么一定要缩图

在真相机（D435I，1280x720 RGB8，26.9 fps）上实测 10 秒：

| 配置 | 服务端耗时中位 | 相机消息数 | 预测次数 | 零速帧占比 |
|---|---|---|---|---|
| 1280x720，不缩图 | 289 ms（峰值 990） | 8 | 5 | **82%**（走走停停） |
| 1280x720 + `--encode-width 640` | **192 ms（P95 225）** | **282** | **40** | **7%** |

原因有两个，都已修好：

1. **服务端**：1280x720 的 JPEG 让预处理/推理耗时翻倍，峰值接近 1 秒。
   缩到 640 宽度后回到 ~190 ms。
2. **本机**：图像回调里做 RGB 转换会把单线程执行器拖满。1280x720@27Hz
   下 BEST_EFFORT 队列（depth 5）疯狂丢帧，10 秒只收到 8 条图像消息，
   图像频繁报「过期」，于是 82% 的时间在发零速。
   **现在回调只贴一个 msg 引用，转换移到 4 Hz 的推理线程里做。**

### 判据

**P95 往返延迟必须 < `--cmd-timeout`（默认 600 ms）**，否则指令会在推理
等待期内过期归零，真机上表现为走走停停。

桥接节点跑满 8 帧后会自己检查并告警，给出建议值：

```
[WARN] 往返 P95 950ms 已接近/超过 cmd_timeout 0.60s，指令会在推理等待期内过期归零，
       真机上表现为走走停停。建议二选一：
  1) 降分辨率（推荐，治本）：给相机加 rgb_camera.color_profile:=640x480x30，
     或本节点 --encode-width 640
  2) 放宽超时：--cmd-timeout 1.4 （但会削弱失速保护）
```

---

### 5.5 踩过的大坑：CycloneDDS 会让相机彻底僵死

这是排查了最久的一个问题，现象非常具有欺骗性，先记下结论：

> **在这台 Jetson 上，宇树那份 CycloneDDS 配置无法承载 RealSense 的图像流。**
> 不论怎么调相机参数，帧率都会在十几秒内从 27 fps 一路衰到 0，且**节点不死、
> 进程在、话题在、但就是不出图**（重启节点也只能好十几秒）。
> 换成 `rmw_fastrtps_cpp` 后稳定 23~30 fps 不掉。

#### 现象

| 时间（每个采样窗口 5 s） | ROS 侧帧率 |
|---|---|
| 刚起节点 | 26.9 fps |
| +15 s | 3.4 fps |
| +30 s | 11.6 → 10.4 fps（抖动） |
| +60 s | **0.5 fps** |
| +90 s | **0 fps**（永久） |

与此同时：

- **硬件本身完全正常**：绕过 ROS 用 `pyrealsense2` 直读，全程稳定 **26.3 fps**。
- USB 链路正常：`5000M`，没掉速。
- 负载正常（0.96），温度正常（39 ℃），非 USB autosuspend，非热降频。
- `ros2 topic hz` 也确认发布端确实在降，不是订阅端慢。

#### 根因

两个条件叠加：

1. **`CYCLONEDDS_URI` 把接口钉死在 `enP2p1s0`**（宇树内网口），
   没有回环、没有共享内存。于是**同机**的两个节点之间传图，
   也要把数据绕一圈网卡走完整网络栈。
2. **图像话题是 `RELIABLE` + `KEEP_LAST(1)`**。

彩色图 640x480 RGB8 ≈ 900 KB/帧 × 30 帧/s ≈ **27 MB/s 的可靠传输**，
在本机环回路径上很快堆满发送队列。`RELIABLE` 会阻塞等 ACK，
KEEP_LAST(1) 又无法丢弃，**写入端被卡住 → 相机节点的单线程执行器被拖死 →
彻底不再发布**。这也解释了为何重启只能好十几秒：队列重新从空开始。

#### 验证与修法

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI          # fastrtps 会忽略它，但顺手清掉免得混淆
```

改完立刻测（连续 6 个 5 秒窗口）：

```
24.6 / 21.8 / 29.6 / 22.8 / 23.0 / 22.8 fps      ← 稳定，不再衰减
```

整链路跑 60 秒：推理 `n_pred` 58 → 137 → 217 → 297（**持续 4.0 Hz**），
`image_age_s` 0.01~0.03 s，网页 MJPEG **4.1 fps**。

目前 `setup_vln_ros2.sh` 已默认使用 `rmw_fastrtps_cpp`。

#### 排错时怎么区分“相机坏了”和“ROS/DDS 坏了”

**先绕开 ROS 直读硬件**，这一步能省掉几个小时的弯路：

```bash
~/vln_env/bin/python - <<'PY'
import pyrealsense2 as rs, time
p = rs.pipeline(); c = rs.config()
c.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
p.start(c); t = time.time(); n = 0
while time.time() - t < 5:
    p.wait_for_frames(); n += 1
print(f"hardware fps = {n / 5:.1f}")
p.stop()
PY
```

- 这里也是 0 → 相机/USB 真有问题。
- 这里 26 fps 而 `ros2 topic hz` 是 0 → **是 DDS，不是相机**，直接上 fastrtps。

另外本节点内置了帧率看门狗（每 5 s 一次），连续低于 5 fps 会直接报错并带上修复命令，
不用再自己一个个试。

---

## 6. 网页控制台（推荐用法）

一个人在旁边看屏幕、决定要不要让机器动——这就是网页控制台要解决的问题。

```bash
cd ~/vln && source ./setup_vln_ros2.sh
$VLN_PY vln_ros2_bridge.py \
    --instruction "go to the red chair" \
    --publish \
    --web \
    --encode-width 640 \
    --web-port 8090
```

然后浏览器打开：

- 同一台机器：`http://127.0.0.1:8090/`
- 局域网其他设备（手机/笔记本）：`http://192.168.128.146:8090/`

启动时会把上面两个 URL 都打印出来。

### 页面上有什么

| 区域 | 内容 |
|---|---|
| 左侧大图 | 相机实时画面（MJPEG，就是送给服务端的那张缩图） |
| 大图左上角 | 当前**实际下发**的 `vx / vy / wz`（不是模型输出，是经过所有限速/超时/急停过滤后真正发出去的） |
| 大图右上角 | 状态：`已授权 / 未授权`、`本轮运行中 / 已结束`、`急停` |
| 右侧「推理结果」 | `seq`、`step`、路点索引、时延、服务端动作、`stop`、`visible`、指向检查 |
| 右侧「运行状态」 | 授权剩余秒数、图像年龄、已发指令数、推理线程是否在跑、错误数 |
| 底部「最近预测」 | 最近几帧的原始回落文本 |
| 底部日志 | 节点事件流 |

### 四个按钮（核心安全设计）

| 按钮 | 行为 |
|---|---|
| **执行（持续）** | 授权直到手动收回。每次推理都会真的发速度 |
| **单步** | 只授权 `--approve-hold-s`（默认 1.0 s），到期自动收回，适合一帧一帧推 |
| **暂停** | 立刻收回授权 + 发一串零速，回到安全状态 |
| **急停** | 收回授权 + 零速 + **结束本轮**（推理线程退出，不再产生新帧） |
| **重新开始本轮** | 从急停/结束状态重新起一轮（`reset` + 新 episode） |

键盘快捷键：`空格` = 暂停，`E` = 执行（持续），`Esc` = 急停。

> **默认未授权。** 不加 `--start-armed` 时，桥接启动后就是未授权状态，
> 页面上会看到下发速度全是 `0.000`，直到你点「执行」或「单步」。
> 也就是说：**没有人在网页上点一下，机器不会收到任何非零速度。**

### 三个设计细节

1. **不重复编码。** 送给服务端的 JPEG 字节，原样转给浏览器，
   没有第二次编码/拷贝开销（这也是为什么 MJPEG 能跟上 4 Hz 推理）。
2. **没有网页也能跑。** `--web` 不加时就是纯命令行行为，不影响原有用法。
3. **开了网页进程就不退出。** 推理线程结束后进程会留在原地，
   所以「重新开始本轮」随时可用；想彻底退出用 `Ctrl-C`。

### REST 接口

页面轮询用 `/api/state`（每 300 ms），写操作是这些 POST：

| 接口 | 体 | 说明 |
|---|---|---|
| `/api/arm` | `{"hold_s": 1.0}` | `>0` 授权 N 秒；`<0` 永久；`0` 用 `--approve-hold-s` |
| `/api/disarm` | – | 收回授权（立即归零） |
| `/api/estop` | – | 急停 + 结束本轮 |
| `/api/resume` | – | 本轮重新开始（清除急停） |
| `/api/instruction` | `{"text": "..."}` | 改指令，下一帧生效 |

用 curl 时**必须带 JSON 头**，否则 form-encoded 体不会被解析：

```bash
curl -X POST -H 'Content-Type: application/json' \
     -d '{"hold_s":-1}' http://127.0.0.1:8090/api/arm
```

### 实测过的按钮流程

| 步骤 | 结果 |
|---|---|
| 启动（未授权） | `实际下发 = 0.000` |
| 执行（持续） | `armed/forever/pub = True`，下发计数 41 → 101 持续增长 |
| 暂停 | 立即归零，冻结在 102 |
| 单步（1.0 s） | 102 → 111 → 到点自动收回 122 |
| 急停 | `estop=True`，推理线程退出 |
| 重新开始本轮 | `estop=False`，线程重新起来 |
| 改指令 | 生效，seq=438，rtt=173 ms |
| 收尾暂停 | 回到全 0 |

---

## 7. 无相机时怎么联调

用 `tools/fake_rs_pub.py` 回放图片目录，只发图不发速度：

```bash
# 终端 A：假相机
$VLN_PY tools/fake_rs_pub.py --frames ./test_frames --fps 15

# 终端 B：桥接
$VLN_PY vln_ros2_bridge.py --instruction "go to the red chair" --publish \
    --max-runtime 12
```

---

## 8. 排错

| 现象 | 原因 / 处理 |
|---|---|
| `等待 /camera/... 的第一帧` 一直不动 | 相机节点没起；话题名不对（`ros2 topic list \| grep image`）；**或两边 RMW 不一致**（见 5.5） |
| 图像帧率从 27 一路掉到 0 | **RMW 是 CycloneDDS**，见 [5.5 节](#55-踩过的大坑cyclonedds-会让相机彻底僵死)。换 `rmw_fastrtps_cpp` |
| 节点在、话题在、就是不出图 | 同上；先用 `pyrealsense2` 直读区分“硬件坏”还是“DDS 坏” |
| `Warning: Parameter 'pointcloud__neon_.enable' ... is not supported` | 同事的 `realsense_g1.yaml` 里有这个参数，当前版本不认，**必须 `pointcloud.enable:=false`** |
| `图像过期/未到达` 刷屏 | 相机帧率掉了，或回调被拖慢。检查 `ros2 topic hz`，并加 `--encode-width 640` |
| 服务端延迟几百毫秒~1 秒 | 缩图（第 5 节）；也可能是服务端 GPU 被别的任务占用 |
| 机器人转向相反 | `--yaw-sign -1` |
| 机器人横移相反 | `--lat-sign -1` |
| 订阅不到图像 | RealSense 要 `qos_profile_sensor_data`(BEST_EFFORT)，节点已内置 |
| 网页打不开 | 端口被占：`--web-port 8091`；或防火墙。节点会自动向后试 11 个端口，看启动日志里的实际 URL |
| 网页画面卡住不动 | 正常情况：点了「急停」本轮已结束，不会再有新帧。点「重新开始本轮」 |
| `curl` 设 hold_s 没反应 | 少了 `-H 'Content-Type: application/json'`，节点不解析 form-encoded |
| `ModuleNotFoundError: No module named 'yaml'` | `~/vln_env/bin/python -m pip install pyyaml` |

---

## 9. 环境要点

- **必须用 `~/vln_env`（Python 3.12）**：`pyrealsense2` 在 aarch64 上只有
  `cp312` wheel；conda 的 Python 3.14 装不上。
- venv 里 `rclpy` 可用，无需额外配置。
- `librealsense` 在 `/usr/local/lib`（本项目走 ROS 话题，实际不直接用）。
- **RMW 默认 `rmw_fastrtps_cpp`**（见 5.5）。本机两种实现都在：
  `librmw_cyclonedds_cpp.so` / `librmw_fastrtps_cpp.so`。
  需要和 G1 自带的 CycloneDDS 节点互通时才切回：
  `VLN_RMW=rmw_cyclonedds_cpp source ./setup_vln_ros2.sh`。
- 网页控制台只用标准库（`http.server` + `threading`），**没装任何新包**。
