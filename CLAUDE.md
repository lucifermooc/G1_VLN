# G1_VLN —— 给后来接手的人（和 Claude）

LightNav-0（Qwen3-VL-4B 视觉语言导航）在 G1 / Jetson Thor 上的独立部署。**与主工程 `unitree-G1/lightning_ws` 的导航栈完全独立**，
不改 lightning_ws 下任何代码（用户明确要求：测试功能单独成包，别让仓库臃肿、别影响别的机器部署）。
最后更新：2026-10-06。用户是初学者，用中文、通俗、一步一步解释。

## 一眼看懂

```
G1_VLN/
  LightNav-0/   模型代码（c6f40e3，原样；robot_deploy/ 里有官方 MPC）
  server/       推理服务脚本（start_server.sh / stop_server.sh / env.sh / bench_ws.py）
  client/       机器人端（我们改过的同事 ~/vln 副本）
                start_vln.sh        一键启动/停止/状态
                vln_ros2_bridge.py  相机 → 服务端 → MPC → 速度，含网页后端、闸门
                vln_web_ui.py       网页（端口 8090）
                usb_camera_node.py  USB 双目相机 → ROS 图像
                vln_mpc_g1.yaml     /vln 命名空间的里程计 + MPC 参数
                vln_cmdvel.yaml     VLN 自己的 cmdvel_to_sport 参数
  deploy.sh     工作站 → Thor（rsync 到 /home/unitree/G1_VLN）
  README.md     用法与链路图
  light-nav.md  模型与链路原理（怎么想、怎么变成速度、现象解释）
```

Thor = `unitree@192.168.128.146`。venv/缓存在 `~/lightnav/`，模型在 `~/models/LightNav-0`，机器人端 python 环境 `~/vln_env`（含 casadi 3.8.1）。
**部署后 Thor 上不保留源码 src 的习惯只针对 lightning_ws；G1_VLN 在 Thor 上是源码直接跑，改了先在本机改再 `./deploy.sh`。**

## 模型与服务端环境

**原理（模型怎么想、一帧怎么变成速度、为什么有那些现象）单独写在 [light-nav.md](light-nav.md)，先看那个。** 这里只记"装了什么、在哪、怎么起"。

| 项 | 内容 |
|---|---|
| 模型 | LightNav-0（Qwen3-VL-4B 微调），HF `LightOriginsHQ/LightNav-0` @ `826dc5f`，约 9.1 GB（safetensors 单文件），不是 gated |
| 位置 | `/home/unitree/models/LightNav-0`（含 `eval_config.json`、`action_tokenizer/`、`chat_template.jinja`） |
| 下载 | hf-mirror 的 9.7 GB 大文件 302 到 xethub，只有 ~30 KB/s，所以改从 ModelScope 同名仓库下（7~9 MB/s），sha256 与 HF LFS oid 一致 |
| 代码 | `G1_VLN/LightNav-0`（c6f40e3，原样）；Thor 上 `PYTHONPATH=src` 直接跑，不 `pip install -e`（pyproject 锁 py3.11，Thor 是 3.12） |
| 解释器 | `~/lightnav/.venv`（python3.12，`--without-pip` + pip wheel 引导） |
| torch | 2.10.0+cu130 / torchvision 0.25.0+cu130（download.pytorch.org/whl/cu130，含 sm_110；CUDA 13 运行库以 nvidia-* wheel 带进 venv，Thor 没装 CUDA toolkit） |
| vLLM | **0.19.0+cu130**（Jetson AI Lab `pypi.jetson-ai-lab.io/sbsa/cu130`，sm_110 原生）。官方 0.19.1 aarch64 wheel 没有 sm_110 SASS，不能用；LightNav 补丁涉及的源码两版逐字节相同 |
| 其他锁定 | transformers 5.8.0、compressed-tensors 0.15.0.1、nvidia-cutlass-dsl 4.5.2（不锁会解析到 dev 版，启动报 `ThrMma`）；`pip check` 报的两条冲突是 vLLM 0.19.0 元数据的版本锁，可忽略 |
| 量化 | `fp8_llm_only`（LLM fp8，ViT bf16）；`QUANT=bf16` 可切回。`GPU_MEM_UTIL` 默认 0.3，脚本拒绝 >0.30（机器人栈和语义地图共用 122 GB 统一内存） |
| 端口 | `ws://127.0.0.1:8050`，默认只监听本机，**服务端无鉴权**，`HOST=0.0.0.0` 才对局域网开放 |
| 日志 | `G1_VLN/LightNav-0/logs/server_8050.log`；`TASK=vln`（默认）/`tracking` 对识别无差别 |
| 内存占用 | 服务起来后整机已用约 21 GB（总 122 GB） |
| 环境变量坑 | 启动前要干净：`env -u PYTHONPATH -u LD_LIBRARY_PATH`（脚本已处理）；`PYTHONNOUSERSITE=1`（casadi 等不能装进 `~/.local`）；Thor 访问不到 huggingface.co，要 `HF_ENDPOINT=https://hf-mirror.com` |
| ptxas | Triton 3.6 自带 ptxas 12.9 不认 sm_110a（`Value 'sm_110a' is not defined for option 'gpu-name'`）→ `server/env.sh` 里 `TRITON_PTXAS_PATH/TRITON_PTXAS_BLACKWELL_PATH` 指向 `.venv/.../nvidia/cu13/bin/ptxas`（13.0.88） |

模型输出与关键常量（详见 light-nav.md）：
- 输出 5 个 token：`<apos_K> <opos_K> <act_l0/l1/l2>`；指向网格 48×27=1296，`apos` 1297/1298/1299 = 左转/右转/停；`opos=0` = 目标不可见。
- 动作 = 3 级 RVQ（各 256 码字）→ 10 个航点（x 前向, y 横向, yaw），间隔 0.1 s；`act_l0=6` 是停止码。
- 输入 256×448（16:9，stretch），历史最多 64 帧 @4 Hz，SlowFast 分 5 层压缩；单步约 300~400 ms。
- 重要结论：`--task vln|tracking` 无差别；识别靠指令措辞（见"实测结论"）。

## 数据流（谁做什么）

```
USB 相机(左眼 640x360) → vln_ros2_bridge ⇄ ws://127.0.0.1:8050 (lightnav-serve)
   模型规划：每次输出 10 个航点(base_link，拍照时刻) + 目标点 opos + stop
   → /vln/response（含拍照时间戳）→ vln_mpc（官方，/vln 命名空间，用 /vln/odom 对齐，10 Hz CasADi MPC 跟踪整条轨迹）
   → /vln/mpc/cmd_vel → 桥接闸门(授权/急停/断流/连续2帧stop) → 安全模式 /vln/cmd_vel(无人订阅)
                                                              | --robot  /vln/cmd_vel_robot → g1_nav_bridge/cmdvel_to_sport(节点名 g1_vln_cmdvel_to_sport) → G1
```
- **规划是模型做的，MPC 只跟踪**。没有地图、没有避障。目标不可见时模型给"直行 1.5 m"，机器人会一直往前走 —— 必须有人盯着。
- `--ctrl wp0` 退回旧行为（只用第 1 个航点换速度，会丢掉后段的转向/减速）。上次实测 wp0 方案整轮只输出直行。
- 里程计：独立的 `sport_to_odom` 实例（节点 `g1_vln_sport_to_odom`，输出 `/vln/odom`），不用导航栈的 `/odom`。
  G1 本体时钟与 Thor 差约 30 ms（实测），可直接按时间戳对齐。
- MPC 限速取自导航栈实测：vx 0~0.45（G1 收到负 vx 会摔倒）、|wz|≤0.9。`cmdvel_to_sport` 会把小速度抬到最小有效值
  （直行 <0.3 抬到 0.3；|wz|≥0.15 抬到 0.6；turn_min_vx=0，G1 能原地转）。

## 怎么用（Thor 上）

```bash
cd ~/G1_VLN/client
./start_vln.sh --all "Go to the red object in front and stop."          # 安全模式：不发给机器人，可和导航栈同时跑
./start_vln.sh --all --robot "Go to the red object in front and stop."  # 真机：要求导航栈已停（stop_all.launch.py）
./start_vln.sh --all --robot --follow "follow the person in the white t-shirt"   # 跟人（MPC track，忽略 stop）
./start_vln.sh --status | --stop        # --stop 只停 VLN 自己的进程，不碰导航栈
../server/stop_server.sh ; ../server/start_server.sh   # 推理服务（vln 默认；start_vln.sh 发现没跑会自动起，约 2~3 分钟）
```
网页 `http://192.168.128.146:8090/`：默认「测试·单步」+「未授权」，不点就不会动。
【① 推理一次】→ 看结果 → 【② 确认执行】走 `单步时长`(默认 1 s) 后自动停；【② 正式·连续】→【开始连续】才会一直走；
【⛔ 一键终止】/【暂停】随时可用。网页上有【🎯 去目标】/【🏃 跟人】预设，运行中切换，不用重启（会重置本轮）。
命令行等价：`./tools/vln_ctl.py step|confirm|skip|go|pause|abort|instruction "..."`。
**用户的安全习惯：验证时先安全模式；真机前要他明确同意并确认场地/人员；VLN 期间没有任何避障，旁边要有人守遥控器/急停。**
Claude 的自动模式分类器会拦截"启动真机链路"的命令 —— 此时不要绕，让用户在 Thor 上自己执行。

## 相机

- 用头顶免驱 USB 双目（Sunplus/STYT，`1bcf:0b15`）：2560x720 并排 MJPG，取左眼 1280x720 → 640x360（16:9，和模型输入一致）。RealSense **已弃用**。
- 设备用稳定路径 `/dev/v4l/by-id/usb-STYT_241029_K_USB_Camera_01.00.00-video-index0`（video 编号会变）。
- 头部 D435 下俯 47.6°，不能用于 VLN（2.5 m 外只拍到地面）。
- **USB 会掉线**：走动时 `usb 1-4-port2: disabled by hub (EMI?)`，相机挂在 Realtek 4 口 USB2 hub 上。
  `usb_camera_node.py` 已自动每秒重连；根治靠直插主板口 + 固定线缆/屏蔽线。掉线期间图像过期 → 速度自动归零。
- 倒装用 `--cam-rotate 180`；右眼 `--cam-eye right`。

## 实测结论（别重查）

1. **模型/服务端**：vLLM 0.19.0+cu130（Jetson AI Lab 源，sm_110 原生）+ torch 2.10.0+cu130，fp8_llm_only，GPU_MEM_UTIL≤0.30。
   Triton 自带 ptxas 12.9 不认 sm_110a → `env.sh` 里设 `TRITON_PTXAS(_BLACKWELL)_PATH` 指向 venv 的 `nvidia/cu13/bin/ptxas`。单步推理约 300~400 ms。
2. **`--task vln` 和 `tracking` 对识别没有区别**（检查点里两者 eval_config 完全相同，同帧结果逐项一致）。不要为了跟人去重启服务。
3. **识别靠措辞，按"看起来是什么"描述**：
   - 能认：`follow the person in the white t-shirt`（1.5~2 m 的人 8/8）、`Go to the man in the white t-shirt in front and stop.`、
     `Walk to the white robot on the right.`、`Go to the red object in front and stop.`（搭红衣服的白凳子 4.5 m 8/8）、墙上橙色 logo。
   - 认不出：`go to the person in white clothes`、各种 `chair` 写法（被红衣服盖住后）、`red clothes`、中文指令（中英文无差别，但英文更稳）。
   - 远处很小的人（几十个像素）认不出。画面下部的近处物体认得差。裁剪放大无改善。
   - 参考官方 Prompt Guide（README）：动作词 + 方向 + 无歧义物体 + 可选 `and stop`。
4. **连续画面比静态画面稳**（模型利用历史帧）：从 4.5 m 遥控靠近凳子，152 步里 151 步认出，opos 跟着目标移动，约 1.5~2 m 处给 `stop`。
5. **安全模式测不出闭环**：机器人不执行规划时，模型看到的历史和自己的规划对不上，会退回"直行"。要测转向/减速/停止必须真机单步。
6. 真机单步已跑通（2026-10-06 17:08~17:09，18 步）：`cmdvel_to_sport` 对每个 SetVelocity 返回 `code=0`，G1 接受指令。

## 已修的坑（同事 ~/vln 原版的问题）

- 原版用 fastrtps，而 `cmdvel_to_sport` 要 CycloneDDS 才连得上 G1 → 真机模式从没通过。现在全部 source 导航栈的 `dds_env.sh`。
- `--stop` 原来用宽匹配会误杀导航栈的相机/cmdvel_to_sport；现在只匹配 VLN 自己的进程（`usb_camera_node.py`、`__node:=g1_vln_*`、`vln_mpc.mpc_node`）。
- **网页 JS 语法错误**：`PAGE` 是普通（非 raw）Python 字符串，里面写 `\n` 被解析成真换行 → 整页 JS 崩 → 按钮全失灵、状态一直"连接中…"、数值全是"—"。
  视频是 `<img>`，所以画面还在，非常有迷惑性。**往 PAGE 里写 JS 反斜杠必须写两个，注释里也别出现反斜杠**。
  改完用 node 检查：取出页面 `<script>` → `node --check`（Thor 有 node）。
- vln_mpc 默认话题是相对名，放进 `/vln` 命名空间会变成 `/vln/vln/response` → 参数里全部写绝对路径；`episode/seq` 必须是正整数。
- 连点【应用并重置】会互相取消 → `resume_episode` 加了锁，等上一次做完。
- 单帧 `stop=True` 原来就结束本轮 → 改成连续 2 帧（`--stop-confirm`）；跟随模式忽略 stop。
- 日志 `logs/ros2_bridge/*.jsonl` 带每步的航点/opos/raw_text；分析时用 `ros2 bag record /vln/mpc/cmd_vel /vln/odom /vln/response ...` 对齐更方便。

## 还没解决 / 下一步

- **没有避障**。真机连续模式很危险；想上真机连续，至少要加距离保护（例如接激光/深度的近距离停车），或把目标限定在空旷场地。
- 模型不一定自己停对地方（上次安全模式连续 75 s 一次 stop 都没给）。到达前要人盯着点【暂停】。
- 连续模式的真机闭环（转向方向是否正确、MPC 是否追得上、第一次 wz=0.9 的原地转是否过猛）只做了单步，未做连续。
- 人在走动时能否持续跟住（跟随模式）未在真机验证。
- 开机自启未做（重启 Thor 后要手动 `start_server.sh`）。
- 近处/低处目标认不出，推测与头顶相机视角有关（未证实）；可试降低相机高度，或对画面做裁剪再送模型。

## 开发约定

- 在本机 `/home/ap/project/G1_VLN` 改，`./deploy.sh` 同步到 Thor；Thor 上改了的话要拷回来。
- 不要改 `lightning_ws`、`g1_web`、接口文档等主工程文件。需要导航栈的东西只用它已安装的公开包（`g1_nav_bridge` 的 `cmdvel_to_sport`、`sport_to_odom`，路径 `/opt/G1/lighting_ws/install`）。
- 记忆文件（Claude 的 memory）里有 `lightnav-thor-eval.md`、`test-features-standalone.md`、`deploy-deletes-src.md`。
