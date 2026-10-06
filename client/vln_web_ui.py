#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLN 网页控制台：实时画面 + 推理结果 + 人工批准闸门
================================================================================

给 ``vln_ros2_bridge.py`` 挂一个浏览器界面，让你能：

* 实时看到**模型看到的那一帧**（MJPEG）
* 实时看到推理结果（航点、换算后的速度、延迟、pointing 状态）
* 用一个按钮决定**是否真的把速度发出去**

设计要点
--------------------------------------------------------------------------------
1. **零额外编码开销**：不在 Jetson 上重新编码图像。推理线程本来就要把图编码成
   JPEG 发给服务端，这里直接**转发同一份字节**给浏览器。所以画面帧率 = 推理帧率
   （默认 4 Hz），也正好就是「模型看到的」那些帧，监控意义最强。
2. **叠加信息不画进图里**：指令/延迟/授权状态由浏览器用 HTML 盖在图上。
   在已经跑着 livox / point_lio / slam 的 Jetson 上，这点差别很关键。
3. **只碰标志位，不发速度**：HTTP 线程只改 ``armed`` 之类的标志，
   所有 ``publish()`` 仍由 rclpy 定时器线程执行，避免 publisher 并发调用。
4. 只用标准库，无额外依赖。

安全语义
--------------------------------------------------------------------------------
``armed`` 是所有下发的总闸：

* ``armed=False`` → 控制定时器**恒发零速**，机器人不可能动
* ``--web`` 启动时**默认不授权**，必须由人点按钮
* 授权有「持续」和「单次」两种：单次会在 ``--approve-hold-s`` 秒后自动收回
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

BOUNDARY = "vlnframe"


# ===========================================================================
# 帧分发：推理线程 → 多个浏览器
# ===========================================================================
class FrameHub:
    """保存最新一帧 JPEG，并支持多个 MJPEG 客户端各自等待「下一帧」。"""

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._seq = 0
        self._jpeg = b""
        self._t = 0.0
        self.n_clients = 0
        self.n_published = 0

    def publish(self, seq: int, jpeg: bytes) -> None:
        with self._cv:
            self._seq = seq
            self._jpeg = jpeg
            self._t = time.monotonic()
            self.n_published += 1
            self._cv.notify_all()

    def wait_next(self, last_seq: int, timeout: float = 5.0) -> tuple[int, bytes, float]:
        """等一帧比 ``last_seq`` 新的图；超时则原样返回（调用方负责发心跳）。"""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._seq == last_seq:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._cv.wait(left)
            return self._seq, self._jpeg, self._t


# ===========================================================================
# 页面
# ===========================================================================
# 注意：PAGE 是普通（非 raw）Python 字符串。页面 JS 里要写换行转义，必须在这里写两个反斜杠加 n；
# 写一个会被 Python 解析成真换行，整页 JS 语法错误（按钮全部失灵、状态一直显示"连接中"）。
PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VLN 控制台</title>
<style>
  :root{
    --bg:#0e1116; --panel:#161b22; --line:#2a3038; --fg:#e6edf3; --dim:#8b949e;
    --ok:#3fb950; --warn:#d29922; --bad:#f85149; --accent:#58a6ff;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans CJK SC",sans-serif}
  code,.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}

  header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;
         padding:10px 16px;background:var(--panel);border-bottom:1px solid var(--line)}
  header h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.3px}
  .badge{padding:2px 9px;border-radius:999px;font-size:12px;font-weight:600;
         border:1px solid transparent;white-space:nowrap}
  .b-ok{background:rgba(63,185,80,.15);color:var(--ok);border-color:rgba(63,185,80,.4)}
  .b-warn{background:rgba(210,153,34,.15);color:var(--warn);border-color:rgba(210,153,34,.4)}
  .b-bad{background:rgba(248,81,73,.15);color:var(--bad);border-color:rgba(248,81,73,.4)}
  .b-dim{background:rgba(139,148,158,.12);color:var(--dim);border-color:var(--line)}
  .spacer{flex:1}

  main{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:14px;padding:14px;
       align-items:start}
  @media (max-width:1080px){main{grid-template-columns:1fr}}

  .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;
         overflow:hidden}
  .panel h2{margin:0;padding:9px 13px;font-size:12px;font-weight:600;color:var(--dim);
            text-transform:uppercase;letter-spacing:.6px;border-bottom:1px solid var(--line)}

  .frame-wrap{position:relative;background:#05070a;aspect-ratio:16/9;
              display:flex;align-items:center;justify-content:center}
  .frame-wrap::after{content:"等待图像 …";position:absolute;color:var(--dim);font-size:13px}
  .frame-wrap.no-::after{content:""}
  #stream{position:relative;z-index:1;width:100%;height:100%;object-fit:contain;display:block}

  .ov{position:absolute;z-index:2;pointer-events:none}
  .ov-cmd{left:12px;top:11px;display:flex;gap:16px;
          text-shadow:0 1px 4px #000,0 0 12px #000}
  .ov-cmd div{display:flex;flex-direction:column}
  .ov-cmd .k{font-size:10px;color:#9aa4b0;letter-spacing:.8px}
  .ov-cmd .v{font-size:23px;font-weight:700;line-height:1.15}
  .ov-meta{left:13px;bottom:10px;font-size:12px;color:#c3ccd6;
           text-shadow:0 1px 4px #000}
  .ov-state{right:11px;top:11px;font-size:12px;font-weight:700;padding:3px 10px;
            border-radius:6px;backdrop-filter:blur(4px)}
  .s-armed{background:rgba(63,185,80,.85);color:#04210c}
  .s-idle{background:rgba(139,148,158,.85);color:#0b0e12}
  .s-estop{background:rgba(248,81,73,.9);color:#fff}
  .s-done{background:rgba(210,153,34,.9);color:#211a04}
  .ov-pos{right:12px;bottom:10px;font-size:12px;color:#c3ccd6;text-align:right;
          text-shadow:0 1px 4px #000}
  /* 本轮结束时画面必然定格 —— 必须明说，否则看起来像相机坏了 */
  .ov-done{left:50%;top:50%;transform:translate(-50%,-50%);display:none;
           padding:11px 20px;border-radius:9px;background:rgba(13,17,23,.84);
           border:1px solid rgba(210,153,34,.75);color:var(--warn);
           font-weight:700;font-size:14px;line-height:1.6;text-align:center;
           white-space:pre-line;backdrop-filter:blur(3px)}
  .ov-done.on{display:block}
  /* 预览是独立线程，本轮结束后画面仍在实时刷新 —— 只轻度降他和提示 */
  .frame-wrap.done #stream{filter:saturate(.75)}

  /* 待确认：测试模式挂起时必须非常显眼 */
  .ov-pend{left:50%;top:50%;transform:translate(-50%,-50%);display:none;
           padding:12px 22px;border-radius:10px;background:rgba(13,17,23,.92);
           border:2px solid var(--accent);text-align:center;backdrop-filter:blur(3px)}
  .ov-pend.on{display:block}
  .ov-pend .t{font-size:11px;color:var(--accent);font-weight:700;
              letter-spacing:.8px;text-transform:uppercase}
  .ov-pend .c{font-size:21px;font-weight:700;margin:5px 0 3px}
  .ov-pend .w{font-size:11px;color:var(--warn);min-height:14px}
  .s-pend{background:rgba(88,166,255,.9);color:#04121f}
  .s-busy{background:rgba(210,153,34,.85);color:#211a04}

  .modesel{display:flex;gap:7px;padding:12px 12px 0}
  .modesel button{flex:1;padding:9px 6px;font-size:13px}
  .modesel button.sel{background:rgba(88,166,255,.18);border-color:var(--accent);
                      color:var(--accent)}
  .stepctl{display:flex;align-items:center;gap:10px;padding:0 12px 12px}
  .stepctl .k{font-size:12px;color:var(--dim);white-space:nowrap}
  .stepctl input[type=range]{flex:1;accent-color:var(--accent);cursor:pointer}
  .stepctl .v{font-size:14px;font-weight:700;min-width:58px;text-align:right}
  button.abort{background:rgba(248,81,73,.24);border-color:var(--bad);color:#fff;
               font-size:15px;padding:15px 10px;letter-spacing:.4px}
  button.abort:hover:not(:disabled){background:rgba(248,81,73,.46)}

  .bar{display:flex;gap:9px;flex-wrap:wrap;padding:12px}
  button{flex:1 1 120px;padding:12px 10px;border-radius:8px;border:1px solid var(--line);
         background:#21262d;color:var(--fg);font-size:14px;font-weight:600;cursor:pointer;
         transition:.12s}
  button:hover:not(:disabled){background:#2d333b;border-color:#4b5563}
  button:disabled{opacity:.4;cursor:not-allowed}
  button.go{background:rgba(63,185,80,.16);border-color:rgba(63,185,80,.55);color:var(--ok)}
  button.go:hover:not(:disabled){background:rgba(63,185,80,.3)}
  button.stop{background:rgba(248,81,73,.15);border-color:rgba(248,81,73,.5);color:var(--bad)}
  button.stop:hover:not(:disabled){background:rgba(248,81,73,.3)}
  button.wide{flex:1 1 100%}
  .hint{padding:0 12px 10px;font-size:11px;color:var(--dim)}

  .instr{display:flex;gap:9px;padding:0 12px 12px}
  .instr input{flex:1;padding:10px;border-radius:8px;border:1px solid var(--line);
               background:#0d1117;color:var(--fg);font-size:13px}
  .instr input:focus{outline:none;border-color:var(--accent)}
  .instr button{flex:0 0 auto;padding:10px 14px}

  table.kv{width:100%;border-collapse:collapse;font-size:13px}
  table.kv td{padding:5px 13px;border-bottom:1px solid rgba(42,48,56,.6)}
  table.kv td:first-child{color:var(--dim);white-space:nowrap;width:42%}
  table.kv td:last-child{text-align:right}
  table.kv tr:last-child td{border-bottom:none}

  #log{max-height:280px;overflow-y:auto;font-size:12px}
  #log .row{display:grid;grid-template-columns:44px 1fr auto;gap:8px;padding:4px 13px;
            border-bottom:1px solid rgba(42,48,56,.4);align-items:baseline}
  #log .row:last-child{border-bottom:none}
  #log .s{color:var(--dim)}
  #log .c{font-family:ui-monospace,monospace;white-space:nowrap}
  .pos{color:var(--ok)} .neg{color:var(--accent)} .zero{color:var(--dim)}
  .stop-badge{color:var(--warn);font-weight:700}
</style>
</head>
<body>

<header>
  <h1>VLN 控制台</h1>
  <span class="badge b-dim" id="bTopic">—</span>
  <span class="badge b-dim" id="bPub">—</span>
  <span class="badge b-dim" id="bRtt">—</span>
  <span class="badge b-dim" id="bImg">—</span>
  <span class="spacer"></span>
  <span class="badge b-dim" id="bConn">连接中…</span>
</header>

<main>
  <div>
    <div class="panel">
      <h2>相机画面（模型实际输入 · <span id="frameRate">—</span> Hz）</h2>
      <div class="frame-wrap" id="wrap">
        <img id="stream" src="/stream.mjpg" alt="camera">
        <div class="ov ov-cmd">
          <div><span class="k">VX 前进</span><span class="v mono" id="oCmdVx">—</span></div>
          <div><span class="k">VY 横移</span><span class="v mono" id="oCmdVy">—</span></div>
          <div><span class="k">WZ 转向</span><span class="v mono" id="oCmdWz">—</span></div>
        </div>
        <div class="ov ov-state s-idle" id="oState">未授权</div>
        <div class="ov ov-meta" id="oMeta">—</div>
        <div class="ov ov-pos" id="oPos">—</div>
        <div class="ov ov-done" id="oDone"></div>
        <div class="ov ov-pend" id="oPend">
          <div class="t">待确认 · step <span id="pStep">—</span></div>
          <div class="c mono" id="pCmd">—</div>
          <div class="w" id="pWarn"></div>
        </div>
      </div>
    </div>

    <div class="panel" style="margin-top:14px">
      <h2>执行控制 · <span id="tMode">—</span></h2>

      <div class="modesel">
        <button id="btnModeStep">① 测试 · 单步</button>
        <button id="btnModeAuto">② 正式 · 连续</button>
      </div>

      <div class="stepctl" id="rowStepCtl">
        <span class="k">单步时长</span>
        <input type="range" id="rngStep" min="0.2" max="5" step="0.1" value="1">
        <span class="v mono" id="vStep">1.0 s</span>
      </div>

      <div class="bar" id="rowStep">
        <button class="go" id="btnInfer">① 推理一次</button>
        <button class="go" id="btnConfirm">② 确认执行</button>
        <button          id="btnSkip">跳过</button>
      </div>

      <div class="bar" id="rowAuto">
        <button class="go" id="btnStartAuto">开始连续</button>
        <button          id="btnPause">暂停</button>
      </div>

      <div class="bar modesel" id="rowPreset">
        <button id="btnPresetGo">🎯 去目标（到达即停）</button>
        <button id="btnPresetFollow">🏃 跟人（持续跟随）</button>
      </div>

      <div class="hint" id="armHint">—</div>

      <div class="bar" style="padding-top:0">
        <button class="abort wide" id="btnAbort">⛔ 一键终止（停推理 + 归零）</button>
      </div>

      <div class="hint" id="curInstr">当前指令：—</div>
      <div class="instr">
        <input id="instr" placeholder="指令，例如 go to the red chair">
        <button id="btnApply">应用并重置</button>
      </div>
      <div class="bar" style="padding-top:0">
        <button class="wide" id="btnResume">重新开始本轮（用当前指令）</button>
      </div>
      <div class="hint">快捷键：<b>E</b> 推进（单步=推理一次 / 连续=开始） ·
        <b>空格</b>（单步=确认执行 / 连续=暂停） · <b>S</b> 跳过 · <b>Esc</b> 一键终止</div>
    </div>
  </div>

  <div>
    <div class="panel">
      <h2>推理结果</h2>
      <table class="kv">
        <tr><td>序号 seq</td><td class="mono" id="kSeq">—</td></tr>
        <tr><td>模型 step</td><td class="mono" id="kStep">—</td></tr>
        <tr><td>航点 wp0 (fwd/ lat/ yaw)</td><td class="mono" id="kWp">—</td></tr>
        <tr><td>往返延迟 rtt</td><td class="mono" id="kRtt">—</td></tr>
        <tr><td>服务端耗时</td><td class="mono" id="kSrv">—</td></tr>
        <tr><td>是否到达 stop</td><td class="mono" id="kStop">—</td></tr>
        <tr><td>目标可见 visible</td><td class="mono" id="kVis">—</td></tr>
        <tr><td>pointing</td><td class="mono" id="kPt">—</td></tr>
        <tr><td>原始输出</td><td class="mono" id="kRaw" style="font-size:11px;word-break:break-all">—</td></tr>
      </table>
    </div>

    <div class="panel" style="margin-top:14px">
      <h2>运行状态</h2>
      <table class="kv">
        <tr><td>授权</td><td class="mono" id="kArmed">—</td></tr>
        <tr><td>指令龄期</td><td class="mono" id="kAge">—</td></tr>
        <tr><td>预测 / 发布 / 零速</td><td class="mono" id="kCnt">—</td></tr>
        <tr><td>推理线程</td><td class="mono" id="kThr">—</td></tr>
        <tr><td>异常</td><td class="mono" id="kErr">—</td></tr>
      </table>
    </div>

    <div class="panel" style="margin-top:14px">
      <h2>最近预测</h2>
      <div id="log"></div>
    </div>
  </div>
</main>

<script>
const $ = (id) => document.getElementById(id);

function fmt(v, n) {
  if (v === null || v === undefined) return "—";
  const s = Number(v).toFixed(n === undefined ? 3 : n);
  return (Number(v) >= 0 ? "+" : "") + s;
}
function cls(v) { return Number(v) > 1e-4 ? "pos" : (Number(v) < -1e-4 ? "neg" : "zero"); }
function esc(s) {
  const d = document.createElement("div");
  d.textContent = s === null || s === undefined ? "" : String(s);
  return d.innerHTML;
}
function setText(id, txt) { const e = $(id); if (e.textContent !== txt) e.textContent = txt; }

let lastSeq = -1;
let seqHist = [];                       // 最近若干次 seq 变化时刻，用来算真实帧率
let lastChangeT = performance.now();    // 上次 seq 变化的时刻（断流检测用）
let streamResetT = 0;

function render(s) {
  const inf = s.inference || {};
  const st  = s.state || {};
  const cmd = inf.cmd || {};
  const pend = s.pending || null;   // 测试模式：挂起待确认的那一步

  // --- 画面叠加 ---
  const armed = st.armed;
  setText("oCmdVx", fmt(cmd.vx));
  setText("oCmdVy", fmt(cmd.vy));
  setText("oCmdWz", fmt(cmd.wz));
  ["oCmdVx","oCmdVy","oCmdWz"].forEach((id,i)=>{
    const e = $(id);
    e.className = "v mono " + cls([cmd.vx,cmd.vy,cmd.wz][i]);
  });

  const os = $("oState");
  const isStep = (st.mode || "auto") === "step";
  const phase = st.phase || "idle";
  const done = !st.worker_alive;   // 推理线程退出 = 本轮已结束
  let stTxt = "待命", stCls = "s-idle";
  if (st.aborted) { stTxt = "已终止"; stCls = "s-estop"; }
  else if (st.estop) { stTxt = "已急停"; stCls = "s-estop"; }
  else if (done) { stTxt = "本轮已结束"; stCls = "s-done"; }
  else if (phase === "pending") { stTxt = "待确认"; stCls = "s-pend"; }
  else if (phase === "executing") { stTxt = "执行中"; stCls = "s-armed"; }
  else if (armed) { stTxt = st.arm_forever ? "连续执行中" : ("执行中 " + st.arm_remaining_s.toFixed(1) + "s"); stCls = "s-armed"; }
  else if (phase === "inferring" || phase === "connecting") { stTxt = "推理中…"; stCls = "s-busy"; }
  else if (phase === "wait_go") { stTxt = "待放行"; stCls = "s-pend"; }
  if (os.textContent !== stTxt) os.textContent = stTxt;
  os.className = "ov ov-state " + stCls;

  $("wrap").classList.toggle("done", done || st.aborted);
  const od = $("oDone");
  od.classList.toggle("on", done && !st.aborted);
  if (done && !st.aborted) setText("oDone",
    (st.estop ? "已急停 · 本轮结束" : "本轮已结束")
    + "\\n画面仍在实时预览 · 点【重新开始本轮】继续");

  // --- 待确认浮层（测试模式）---
  const op = $("oPend");
  op.classList.toggle("on", !!pend && isStep && !done);
  if (pend && isStep && !done) {
    setText("pStep", pend.step);
    setText("pCmd", "vx " + fmt(pend.cmd.vx) + "   vy " + fmt(pend.cmd.vy)
      + "   wz " + fmt(pend.cmd.wz));
    if (pend.stop) setText("pWarn", "★ 模型判定已到达目标，建议跳过");
    else if (st.pending_timeout_s > 0)
      setText("pWarn", Math.max(0, st.pending_timeout_s - st.pending_age_s).toFixed(0)
        + " s 后自动丢弃（不会执行）");
    else setText("pWarn", "已设为不限时，请手动确认或跳过");
  }

  if (inf.seq === undefined || inf.seq === null) {
    setText("oMeta", "等待推理 …");
  } else {
    setText("oMeta", "seq " + inf.seq + " · rtt " + inf.rtt_ms.toFixed(0) + "ms · srv "
      + inf.server_ms.toFixed(0) + "ms · wp0 [" + fmt(inf.wp0[0]) + " " + fmt(inf.wp0[1])
      + " " + fmt(inf.wp0[2]) + "]" + (inf.stop ? "  ★到达" : ""));
  }
  const pt = inf.pointing || {};
  setText("oPos", (pt.apos_state ? ("apos " + pt.apos_state + (pt.apos_clamped ? "*" : "") + "   ") : "")
    + (pt.opos_state ? ("opos " + pt.opos_state) : ""));

  // --- 页头徽章 ---
  const bTopic = $("bTopic");
  bTopic.textContent = st.cmd_vel_topic;
  bTopic.className = "badge " + (st.publish_enabled ? "b-warn" : "b-dim");
  const bPub = $("bPub");
  bPub.textContent = st.publish_enabled ? (st.publishing ? "正在发布速度" : "发布已使能·当前零速") : "观察模式（不发布）";
  bPub.className = "badge " + (st.publish_enabled ? (st.publishing ? "b-bad" : "b-warn") : "b-dim");
  const bRtt = $("bRtt");
  if (inf.rtt_ms !== undefined && inf.rtt_ms !== null) {
    bRtt.textContent = "rtt " + inf.rtt_ms.toFixed(0) + " ms";
    bRtt.className = "badge " + (inf.rtt_ms > st.cmd_timeout * 1000 ? "b-bad" : "b-ok");
  }
  const bImg = $("bImg");
  if (st.image_age_s === null || st.image_age_s === undefined) {
    bImg.textContent = "无图像"; bImg.className = "badge b-bad";
  } else {
    bImg.textContent = "图像 " + st.image_age_s.toFixed(2) + " s";
    bImg.className = "badge " + (st.image_age_s > 1.0 ? "b-warn" : "b-ok");
  }

  // --- 右侧表格 ---
  setText("kSeq", inf.seq === null || inf.seq === undefined ? "—" : String(inf.seq));
  setText("kStep", inf.step === null || inf.step === undefined ? "—" : String(inf.step));
  setText("kWp", inf.wp0 ? ("[" + fmt(inf.wp0[0]) + " " + fmt(inf.wp0[1]) + " " + fmt(inf.wp0[2]) + "]") : "—");
  setText("kRtt", inf.rtt_ms === null || inf.rtt_ms === undefined ? "—" : inf.rtt_ms.toFixed(1) + " ms");
  setText("kSrv", inf.server_ms === null || inf.server_ms === undefined ? "—" : inf.server_ms.toFixed(1) + " ms");
  setText("kStop", inf.stop === undefined ? "—" : (inf.stop ? "是" : "否"));
  setText("kVis", inf.visible === undefined || inf.visible === null ? "—" : String(inf.visible));
  setText("kPt", (pt.apos_state || pt.opos_state)
    ? ((pt.apos_state || "-") + (pt.apos_clamped ? "*" : "") + " / " + (pt.opos_state || "-"))
    : "—");
  setText("kRaw", inf.raw_text || "—");

  setText("kArmed", armed ? (st.arm_forever ? "持续授权" : ("单次 " + st.arm_remaining_s.toFixed(2) + " s")) : "未授权");
  setText("kAge", st.cmd_age_s === null || st.cmd_age_s === undefined ? "—" : st.cmd_age_s.toFixed(3) + " s");
  setText("kCnt", st.n_pred + " / " + st.n_pub + " / " + st.n_zero);
  setText("kThr", st.worker_alive ? "运行中" : "已结束");
  setText("kErr", st.fatal || "无");

  // --- 模式选择器与分区 ---
  setText("curInstr", "当前指令（模型正在用的）：" + (st.instruction || "—") + (st.follow ? "　· 跟随模式" : "　· 去目标模式"));
  $("btnPresetGo").classList.toggle("sel", !st.follow);
  $("btnPresetFollow").classList.toggle("sel", !!st.follow);
  $("btnModeStep").classList.toggle("sel", isStep);
  $("btnModeAuto").classList.toggle("sel", !isStep);
  setText("tMode", isStep ? "测试 · 单步" : "正式 · 连续");
  $("rowStep").style.display = isStep ? "" : "none";
  $("rowAuto").style.display = isStep ? "none" : "";
  $("rowStepCtl").style.display = isStep ? "" : "none";

  // --- 提示 ---
  let hint;
  if (st.aborted) {
    hint = "⛔ 已一键终止：推理已停、速度为零。画面仍在实时预览，点【重新开始本轮】继续。";
  } else if (st.estop) {
    hint = "🛑 已急停结束本轮。点【重新开始本轮】用当前指令再来一轮。";
  } else if (done) {
    hint = "本轮已结束（推理线程已退出）。画面仍在预览，点【重新开始本轮】继续。";
  } else if (isStep) {
    hint = pend
      ? "⏳ 待确认：此刻速度仍为零。点【确认执行】才走这一小步，或【跳过】丢弃。"
      : (phase === "executing"
         ? "正在执行这一小步，时间到自动归零。"
         : "测试模式：点【推理一次】推一帧，看过结果再决定执不执行。");
  } else if (phase === "wait_go") {
    hint = "正式模式待放行：点【开始连续】后才开始推理与下发。";
  } else {
    hint = armed
      ? (st.arm_forever ? "已持续授权：会一直发速度，直到点【暂停】或【一键终止】。"
                        : "单次授权：" + st.arm_remaining_s.toFixed(1) + " 秒后自动收回。")
      : "未授权：控制定时器只发零速，机器人不会动。";
  }
  $("armHint").textContent = hint;

  // --- 按钮可用性 ---
  const canOp = st.publish_enabled && !done && !st.aborted && !st.estop;
  $("btnInfer").disabled     = !canOp || !isStep || !!pend;
  $("btnConfirm").disabled   = !canOp || !pend;
  $("btnSkip").disabled      = !canOp || !pend;
  $("btnStartAuto").disabled = !canOp || isStep || st.auto_started;
  $("btnPause").disabled     = !armed;
  $("btnResume").disabled    = !done;
  $("btnResume").classList.toggle("go", done);   // 本轮结束后高亮，引导点击
  $("btnAbort").disabled = false;   // 一键终止永远可点 —— 它本身就是安全动作

  // --- 单步时长滑块（别在用户拖的时候往回写）---
  const rs = $("rngStep");
  if (document.activeElement !== rs) rs.value = st.step_exec_s;
  setText("vStep", st.step_exec_s.toFixed(1) + " s");

  // --- 最近预测列表 ---
  const hist = (s.history || []).slice().reverse();
  const box = $("log");
  const key = hist.map(r => r.seq).join(",");
  if (box.dataset.key !== key) {
    box.dataset.key = key;
    box.innerHTML = hist.map(r => (
      '<div class="row"><span class="s">' + r.seq + "</span>"
      + '<span class="c"><span class="' + cls(r.vx) + '">vx ' + fmt(r.vx) + '</span>  '
      + '<span class="' + cls(r.wz) + '">wz ' + fmt(r.wz) + "</span>"
      + (r.stop ? ' <span class="stop-badge">到达</span>' : "") + "</span>"
      + '<span class="s">' + r.rtt_ms.toFixed(0) + "ms</span></div>"
    )).join("") || '<div class="row"><span class="s">暂无</span></div>';
  }
}

async function refresh() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    if (!r.ok) throw new Error(r.status);
    const s = await r.json();
    render(s);
    window.__st = s.state;                                  // 供断流检测读取
    trackSeq(s.inference ? s.inference.seq : null);
    const bc = $("bConn");
    bc.textContent = "已连接";
    bc.className = "badge b-ok";
    window.__fail = 0;
  } catch (e) {
    window.__fail = (window.__fail || 0) + 1;
    if (window.__fail > 2) {
      const bc = $("bConn");
      bc.textContent = "连接断开";
      bc.className = "badge b-bad";
    }
  }
}

// 真实帧率 = seq 的增长速度。
// 原先算的是 1/轮询间隔（恒为 ~3.3），跟图像帧率没关系，纯属误导。
function trackSeq(seq) {
  if (seq === null || seq === undefined) return;
  const now = performance.now();
  if (seq !== lastSeq) { lastSeq = seq; lastChangeT = now; seqHist.push(now); }
  seqHist = seqHist.filter((t) => now - t <= 3000);
  if (seqHist.length >= 2) {
    const span = (now - seqHist[0]) / 1000;
    if (span > 0.5) setText("frameRate", ((seqHist.length - 1) / span).toFixed(1));
  }
}

async function post(path, body) {
  try {
    await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
  } catch (e) { /* 下一轮轮询会纠正界面 */ }
  refresh();
}

// --- 模式切换 ---
// --- 任务预设：同时切 MPC 模式和指令（运行中可用，不用重启；会重置本轮，仍需重新授权）---
$("btnPresetGo").onclick = () => post("/api/follow", { on: false, instruction: "Go to the red object in front and stop." });
$("btnPresetFollow").onclick = () => post("/api/follow", { on: true, instruction: "follow the person in the white t-shirt" });
$("btnModeStep").onclick = () => post("/api/mode", { mode: "step" });
$("btnModeAuto").onclick = () => post("/api/mode", { mode: "auto" });

// --- 测试模式：推理 → 确认 → 走一小步 ---
$("btnInfer").onclick   = () => post("/api/step", {});
$("btnConfirm").onclick = () => post("/api/confirm", { step_exec_s: Number($("rngStep").value) });
$("btnSkip").onclick    = () => post("/api/skip", {});

// --- 正式模式 ---
$("btnStartAuto").onclick = () => post("/api/start_auto", {});
$("btnPause").onclick     = () => post("/api/disarm", {});

// --- 一键终止：不弹确认框。它本身就是安全动作，不该耽误时间 ---
$("btnAbort").onclick = () => post("/api/abort", {});

$("btnResume").onclick = () => post("/api/resume", {});
$("btnApply").onclick = () => {
  const v = $("instr").value.trim();
  if (!v) return;
  post("/api/instruction", { instruction: v });
  const b = $("btnApply");
  b.textContent = "已应用 ✓（本轮已重置）";
  setTimeout(() => { b.textContent = "应用并重置"; }, 2000);
};

// --- 单步时长（拖动时只改显示，松手才提交）---
$("rngStep").oninput  = () => setText("vStep", Number($("rngStep").value).toFixed(1) + " s");
$("rngStep").onchange = () => post("/api/step_exec", { step_exec_s: Number($("rngStep").value) });

document.addEventListener("keydown", (e) => {
  const t = e.target.tagName;
  if (t === "INPUT" || t === "TEXTAREA") return;
  const st = window.__st || {};
  const isStep = (st.mode || "auto") === "step";
  if (e.key === " ") {
    e.preventDefault();
    (isStep ? $("btnConfirm") : $("btnPause")).click();
  } else if (e.key === "e" || e.key === "E") {
    (isStep ? $("btnInfer") : $("btnStartAuto")).click();
  } else if (e.key === "s" || e.key === "S") {
    if (isStep) $("btnSkip").click();
  } else if (e.key === "Escape") {
    $("btnAbort").click();
  }
});

// MJPEG 断流/卡死自动重连
function reloadStream() {
  const now = performance.now();
  if (now - streamResetT < 4000) return;   // 别刷太频繁
  streamResetT = now;
  lastChangeT = now;
  $("stream").src = "/stream.mjpg?t=" + Date.now();
}
$("stream").onerror = () => setTimeout(reloadStream, 1500);

// 连接还在、但 4 秒没有新帧 → 判定为卡死。
// 本轮结束后本来就不会有新帧，那种情况靠 oDone 提示，不要重连。
setInterval(() => {
  const st = window.__st;
  if (!st) return;
  if (st.preview_hz === 0) return;   // 关掉预览时画面本当不动，不要重连
  if (performance.now() - lastChangeT > 4000) reloadStream();
}, 1000);

refresh();
setInterval(refresh, 300);
</script>
</body>
</html>
"""


# ===========================================================================
# HTTP 处理
# ===========================================================================
class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "VlnWebUi/1.0"

    # -- 工具 ---------------------------------------------------------------
    @property
    def ui(self) -> "WebUi":
        return self.server.ui  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        pass  # 静音默认的每请求一行日志

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, obj: Any, code: int = 200) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _read_json(self) -> dict[str, Any]:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return {}
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8")) or {}
        except Exception:
            return {}

    # -- GET ----------------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/stream.mjpg":
            self._serve_mjpeg()
        elif path == "/api/state":
            self._send_json(self.ui.node.get_state())
        elif path == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
        else:
            self._send_json({"ok": False, "error": "not found"}, 404)

    def _serve_mjpeg(self) -> None:
        hub = self.ui.hub
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        hub.n_clients += 1
        last = -1
        try:
            while not self.ui.closed:
                seq, jpeg, _t = hub.wait_next(last, timeout=5.0)
                if seq == last:
                    continue  # 超时无新帧，继续等（连接保持）
                if not jpeg:
                    continue
                last = seq
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            hub.n_clients -= 1

    # -- POST ---------------------------------------------------------------
    # 所有 POST 都转发给 node.dispatch()，本地 unix socket 走的是同一个入口。
    # 这样「网页能做的」和「命令行能做的」永远一致，不会两条路慢慢跑偏。
    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        node = self.ui.node
        body = self._read_json()

        if not path.startswith("/api/"):
            self._send_json({"ok": False, "error": "not found"}, 404)
            return

        cmd = self._POST_CMD.get(path.removeprefix("/api/"))
        if cmd is None:
            self._send_json({"ok": False, "error": "not found"}, 404)
            return

        self._send_json(node.dispatch(cmd, body))

    _POST_CMD = {
        "arm": "arm",
        "disarm": "disarm",
        "estop": "estop",
        "resume": "resume",
        "instruction": "instruction",
        "step": "step",
        "confirm": "confirm",
        "skip": "skip",
        "step_exec": "step_exec",
        "pending_timeout": "pending_timeout",
        "start_auto": "start_auto",
        "mode": "mode",
        "follow": "follow",
        "abort": "abort",
    }


# ===========================================================================
# 服务器
# ===========================================================================
class WebUi:
    """把 ``node``（需要提供 get_state/arm/disarm/estop/set_instruction）暴露成网页。"""

    def __init__(self, node: Any, host: str = "0.0.0.0", port: int = 8090) -> None:
        self.node = node
        self.host = host
        self.port = port
        self.hub = FrameHub()
        self.closed = False
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._t0 = time.monotonic()

    # -- 给节点调用 ---------------------------------------------------------
    def publish_frame(self, seq: int, jpeg: bytes) -> None:
        self.hub.publish(seq, jpeg)

    # -- 生命周期 -----------------------------------------------------------
    def start(self) -> None:
        class _Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        _Server.ui = self  # type: ignore[attr-defined]

        for port in range(self.port, self.port + 12):
            try:
                self._httpd = _Server((self.host, port), _Handler)
                self.port = port
                break
            except OSError as e:
                print(f"[web] 端口 {port} 被占用（{e}），试下一个 …", flush=True)
        if self._httpd is None:
            raise RuntimeError(f"{self.port}–{self.port + 11} 端口全部占用，放弃启动网页")

        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        kwargs={"poll_interval": 0.3},
                                        daemon=True, name="vln-web")
        self._thread.start()

    def stop(self) -> None:
        self.closed = True
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass

    def urls(self) -> list[str]:
        """列出可访问的 URL（本机 + 各网卡地址），方便从笔记本打开。"""
        out = [f"http://127.0.0.1:{self.port}/"]
        for ip in _local_ips():
            if ip.startswith("127."):
                continue
            out.append(f"http://{ip}:{self.port}/")
        return out


def _local_ips() -> list[str]:
    ips: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except Exception:
        pass
    # 不真正发包，只为拿到出口网卡地址
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.110.204", 8050))
        ips.add(s.getsockname()[0])
    except Exception:
        pass
    finally:
        s.close()
    return sorted(ips)
