#!/usr/bin/env python3
"""VLN 控制台 —— 命令行版。

网页上能点的按钮，这里都能点。调试/自动化时不用开浏览器。

    ./tools/vln_ctl.py state                 看完整状态
    ./tools/vln_ctl.py pending               只看待确认的那一步

    ./tools/vln_ctl.py step                  测试模式：推理一次
    ./tools/vln_ctl.py confirm [秒]          测试模式：确认执行（可顺手改时长）
    ./tools/vln_ctl.py skip                  测试模式：跳过这一步

    ./tools/vln_ctl.py auto-m                切到正式·连续
    ./tools/vln_ctl.py go                    正式模式：开始连续（放行）
    ./tools/vln_ctl.py pause                 暂停（只收回授权，推理继续）

    ./tools/vln_ctl.py abort                 ⛔ 一键终止（停推理 + 归零）
    ./tools/vln_ctl.py resume                重新开始本轮（用当前指令）

    ./tools/vln_ctl.py exec-s 1.5            设单步执行时长（秒）
    ./tools/vln_ctl.py timeout 60            设待确认自动丢弃（秒，0=不限）
    ./tools/vln_ctl.py instruction "..."     换指令并重置本轮

    ./tools/vln_ctl.py web                   看网页开没开
    ./tools/vln_ctl.py web on                挂上网页（等价 ./start_vln.sh --web）
    ./tools/vln_ctl.py web off               摘掉网页（等价 ./start_vln.sh --web-off）

    ./tools/vln_ctl.py watch                 每秒刷一次状态，Ctrl-C 停

两种通道，自动选：
  * 网页已经开着  → 走 HTTP POST /api/*
  * 网页没开/挂了 → 走桥接的本地 unix socket（/tmp/vln_bridge.sock）
所以「一条命令让网页起死回生」这件事本身不会依赖网页。

    --port N   指定网页端口（默认从 /tmp/vln_web_port 读，读不到用 8090）
    --sock P   指定控制通道路径（默认 /tmp/vln_bridge.sock）
"""

import json
import socket
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

PORT = 8090
SOCK_PATH = "/tmp/vln_bridge.sock"
PORT_FILE = "/tmp/vln_web_port"
_PORT_FIXED = False


def _url(path: str) -> str:
    return f"http://127.0.0.1:{PORT}{path}"


def _resolve_port() -> None:
    """没显式给 --port 时，从桥接写的端口文件里读真实端口。

    网页端口被占用时桥接会自动往后试 12 个，写死 8090 会连不上。
    """
    global PORT
    if _PORT_FIXED:
        return
    try:
        PORT = int(Path(PORT_FILE).read_text(encoding="utf-8").strip())
    except Exception:
        pass


def _http_get_state() -> dict:
    with urllib.request.urlopen(_url("/api/state"), timeout=5) as r:
        return json.load(r)


def _http_post(path: str, body: dict) -> dict:
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(
        _url(path), data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


def _sock_request(cmd: str, body: dict) -> dict:
    """走桥接的本地控制通道。一行 JSON 请求，一行 JSON 响应。"""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(20.0)
    try:
        s.connect(SOCK_PATH)
        s.sendall((json.dumps({"cmd": cmd, "body": body},
                              ensure_ascii=False) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data and len(data) < 4_000_000:
            chunk = s.recv(65536)
            if not chunk:
                break
            data += chunk
    finally:
        s.close()
    if not data.strip():
        raise RuntimeError("控制通道没返回内容")
    return json.loads(data.decode("utf-8"))


def get_state() -> dict:
    _resolve_port()
    try:
        return _http_get_state()
    except Exception:
        return _sock_request("state", {}).get("state") or {}


def post(path: str, body: dict | None = None) -> dict:
    """POST /api/*，失败自动回退到控制通道。"""
    _resolve_port()
    body = body or {}
    try:
        return _http_post(path, body)
    except Exception as http_err:
        cmd = path.removeprefix("/api/")
        try:
            return _sock_request(cmd, body)
        except Exception as sock_err:
            raise RuntimeError(
                f"网页连不上（{http_err}），控制通道也连不上（{sock_err}）。"
                "桥接起了吗？ ./start_vln.sh --status"
            ) from sock_err


# --------------------------------------------------------------------------
# 打印
# --------------------------------------------------------------------------
def _fmt(v, nd=2):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def show(r: dict, terse: bool = False) -> None:
    # GET /api/state 直接返回 get_state()，POST /api/* 返回 {"ok":..., "state": get_state()}
    # —— 差一层，这里统一成后者。
    inner = r.get("state")
    payload = inner if isinstance(inner, dict) and "inference" in inner else r
    st = payload.get("state") or {}
    inf = payload.get("inference") or {}
    pend = payload.get("pending")

    phase = st.get("phase", "?")
    mode = st.get("mode", "?")
    badge = {
        "idle": "待命", "connecting": "连服务器", "inferring": "推理中",
        "pending": "⏳ 待确认", "executing": "▶ 执行中", "auto": "▶ 连续跑",
        "wait_go": "⏸ 待放行", "ended": "本轮结束",
    }.get(phase, phase)

    if st.get("aborted"):
        badge = "⛔ 已终止"
    elif st.get("estop"):
        badge = "🛑 已急停"
    elif not st.get("worker_alive"):
        badge = "· 本轮结束"

    print(f"  模式 {mode}   阶段 {badge}   步 #{st.get('step_index', 0)}"
          f"   步长 {_fmt(st.get('step_exec_s'))}s")
    print(f"  授权 {'是' if st.get('armed') else '否'}"
          f"{'（持续）' if st.get('arm_forever') else ''}"
          f"   剩余 {_fmt(st.get('arm_remaining_s'), 1)}s"
          f"   发布 {'开' if st.get('publish_enabled') else '关'}"
          f"   话题 {st.get('cmd_vel_topic')}")
    print(f"  计数 图 {st.get('n_img')}  推理 {st.get('n_pred')}"
          f"  下发 {st.get('n_pub')}  零速 {st.get('n_zero')}"
          f"   图龄 {_fmt(st.get('image_age_s'), 3)}s")
    web_txt = (f"开（端口 {st.get('web_port')}，窗口 {st.get('web_clients')}）"
               if st.get("web_on") else "关（./tools/vln_ctl.py web on）")
    print(f"  网页 {web_txt}")
    if st.get("fatal"):
        print(f"  ❗ fatal: {st['fatal']}")

    if inf:
        c = inf.get("cmd") or {}
        print(f"  最新   seq={inf.get('seq')} step={inf.get('step')}"
              f" wp0={inf.get('wp0')} stop={inf.get('stop')}"
              f" rtt={inf.get('rtt_ms')}ms")
        print(f"         cmd vx={_fmt(c.get('vx'), 3)}"
              f" vy={_fmt(c.get('vy'), 3)} wz={_fmt(c.get('wz'), 3)}")
        if inf.get("raw_text"):
            print(f"         模型: {str(inf['raw_text'])[:110]}")
    else:
        print("  最新   （还没推理过）")

    if pend:
        c = pend.get("cmd") or {}
        age = st.get("pending_age_s")
        tmo = st.get("pending_timeout_s")
        left = "不限" if not tmo else f"{max(0.0, tmo - (age or 0)):.0f}s 后丢弃"
        print(f"  ⏳ 待确认  step={pend.get('step')}"
              f"  vx={_fmt(c.get('vx'), 3)} vy={_fmt(c.get('vy'), 3)}"
              f" wz={_fmt(c.get('wz'), 3)}  ({left})")
        if pend.get("stop"):
            print("         ★ 模型判定到达目标，建议跳过")
    elif mode == "step" and st.get("worker_alive"):
        print("  ⏳ 待确认  （无，点 step 推一帧）")


WATCH_KEYS = ("mode", "phase", "step_index", "n_pred", "n_pub", "n_zero")


def _n_pred(s: dict) -> int:
    return int(((s or {}).get("state") or {}).get("n_pred") or 0)


def _wait_pred_change(before: int, timeout: float = 8.0) -> None:
    """等推理计数涨过 before。

    推理是异步的（HTTP 线程只负责放行，真正跑在推理线程里），
    所以发完 step 立刻回读只会看到「推理 0」这个发送前的快照 —— 很误导人。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if _n_pred(get_state()) > before:
                return
        except Exception:
            return
        time.sleep(0.15)


def watch() -> None:
    last = None
    while True:
        try:
            s = get_state()
        except Exception as e:  # noqa: BLE001
            print(f"\n连接失败：{e}")
            time.sleep(1)
            continue
        st = s.get("state") or {}
        sig = tuple(st.get(k) for k in WATCH_KEYS) + (
            (s.get("pending") or {}).get("step"),)
        if sig != last:
            ts = time.strftime("%H:%M:%S")
            pend = s.get("pending")
            extra = f"  ⏳待确认#{pend['step']}" if pend else ""
            print(f"[{ts}] {st.get('mode')}/{st.get('phase')}"
                  f"  pred={st.get('n_pred')} pub={st.get('n_pub')}"
                  f" zero={st.get('n_zero')}{extra}")
            last = sig
        time.sleep(0.5)


# --------------------------------------------------------------------------
def main(argv: list[str]) -> int:
    global PORT, SOCK_PATH, _PORT_FIXED

    args: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--port":
            try:
                PORT = int(argv[i + 1])
            except (IndexError, ValueError):
                print("--port 需要一个端口号", file=sys.stderr)
                return 2
            _PORT_FIXED = True
            i += 2
            continue
        if argv[i] == "--sock":
            if i + 1 >= len(argv):
                print("--sock 需要一个路径", file=sys.stderr)
                return 2
            SOCK_PATH = argv[i + 1]
            i += 2
            continue
        args.append(argv[i])
        i += 1

    cmd = (args[0] if args else "state").lower()
    arg1 = args[1] if len(args) > 1 else None

    try:
        if cmd == "state":
            show(get_state())
            return 0
        if cmd == "pending":
            pend = get_state().get("pending")
            if not pend:
                print("（没有待确认的步骤）")
                return 1
            print(json.dumps(pend, ensure_ascii=False, indent=1))
            return 0
        if cmd == "watch":
            watch()
            return 0

        # ---- 网页启停：只走控制通道 ----
        # 原因很直白：网页没开的时候，HTTP 这条路根本不存在。
        if cmd in ("web", "web-on", "web-off", "webctl", "web-ctl"):
            action = {"web-on": "on", "web-off": "off"}.get(
                cmd, (arg1 or "status").lower())
            r = _sock_request("web", {"action": action})
            on = bool(r.get("web"))
            if action in ("on", "start") and on:
                p = r.get("port") or PORT
                print(f"🖥️ 网页面板已启动： http://127.0.0.1:{p}/")
                try:
                    import socket as _s
                    hn = _s.gethostname()
                    for info in _s.getaddrinfo(hn, None, _s.AF_INET):
                        ip = info[4][0]
                        if not ip.startswith("127."):
                            print(f"                   http://{ip}:{p}/")
                except Exception:
                    pass
            elif action in ("off", "stop"):
                print("🖥️ 网页面板已关闭" if not on else "⚠️ 关闭失败")
            else:
                if on:
                    print(f"🖥️ 网页面板：开（端口 {r.get('port') or PORT}）")
                else:
                    print("🖥️ 网页面板：关（用 web on 打开）")
            return 0 if r.get("ok") else 1

        # --- 下面是动作，一律先发请求再回读状态 ---
        if cmd in ("step", "infer", "s"):
            before = _n_pred(get_state())
            post("/api/step")
            _wait_pred_change(before)
            r = {"ok": True, "state": get_state()}
        elif cmd in ("confirm", "ok", "c"):
            body = {"step_exec_s": float(arg1)} if arg1 else {}
            r = post("/api/confirm", body)
        elif cmd in ("skip", "x"):
            r = post("/api/skip")
        elif cmd in ("go", "start", "start_auto"):
            r = post("/api/start_auto")
        elif cmd in ("pause", "disarm"):
            r = post("/api/disarm")
        elif cmd in ("abort", "stop-all", "kill"):
            r = post("/api/abort")
        elif cmd in ("estop",):
            r = post("/api/estop")
        elif cmd in ("resume", "restart"):
            r = post("/api/resume")
        elif cmd in ("step-mode", "step_m", "step-m", "m-step", "mode-step"):
            r = post("/api/mode", {"mode": "step"})
        elif cmd in ("auto-m", "auto_m", "auto-mode", "m-auto", "mode-auto"):
            r = post("/api/mode", {"mode": "auto"})
        elif cmd in ("arm",):
            r = post("/api/arm", {"hold_s": float(arg1) if arg1 else 0.0})
        elif cmd in ("exec-s", "exec_s", "step-exec"):
            r = post("/api/step_exec", {"step_exec_s": float(arg1 or 1.0)})
        elif cmd in ("timeout", "pending-timeout"):
            r = post("/api/pending_timeout", {"pending_timeout_s": float(arg1 or 60)})
        elif cmd == "instruction":
            if not arg1:
                print("instruction 需要一个字符串", file=sys.stderr)
                return 2
            r = post("/api/instruction", {"instruction": arg1})
        else:
            print(__doc__)
            return 2
    except RuntimeError as e:
        print(f"❌ {e}", file=sys.stderr)
        return 1
    except urllib.error.URLError as e:
        print(f"❌ 连不上网页服务（{e}）。桥接起了吗？ ./start_vln.sh --status",
              file=sys.stderr)
        return 1

    show(r)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
