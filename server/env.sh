# shellcheck shell=bash
# Shared settings for the LightNav-0 server scripts on the G1 Jetson Thor.
# Sourced by install_server.sh / start_server.sh / stop_server.sh -- not executed.
#
# Every write these scripts make lands under $LN_ROOT or $MODEL_PATH; $HOME/.cache,
# $HOME/.local and the system Python are never touched. Override any value by
# exporting it before calling a script.

: "${LN_ROOT:=/home/unitree/lightnav}"
: "${VENV:=$LN_ROOT/.venv}"
# 代码在 G1_VLN/LightNav-0（本目录的上一级）；venv、缓存、模型继续用 $LN_ROOT / $MODEL_PATH，不重装。
_G1_VLN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${REPO_DIR:=$_G1_VLN_ROOT/LightNav-0}"
: "${MODEL_PATH:=/home/unitree/models/LightNav-0}"
# Not plain `python3`: interactive shells on this Thor resolve that to miniconda 3.14.
: "${PY_BASE:=/usr/bin/python3.12}"
: "${PORT:=8050}"
export LN_ROOT VENV REPO_DIR MODEL_PATH PY_BASE PORT

# Caches that would otherwise default to $HOME/.cache, ~/.triton, ~/.nv, ~/.config.
export PIP_CACHE_DIR="$LN_ROOT/.cache/pip"
export XDG_CACHE_HOME="$LN_ROOT/.cache"
export HF_HOME="$LN_ROOT/.cache/huggingface"
export TORCH_HOME="$LN_ROOT/.cache/torch"
export VLLM_CACHE_ROOT="$LN_ROOT/.cache/vllm"
export VLLM_CONFIG_ROOT="$LN_ROOT/.config/vllm"
export TRITON_HOME="$LN_ROOT/.cache/triton_home"
export TRITON_CACHE_DIR="$LN_ROOT/.cache/triton"
export CUDA_CACHE_PATH="$LN_ROOT/.cache/nv_compute"
export FLASHINFER_WORKSPACE_BASE="$LN_ROOT"
# The venv already ignores ~/.local; make it explicit for any helper interpreter.
export PYTHONNOUSERSITE=1
# A ROS-sourced shell puts /opt/ros/... on PYTHONPATH ahead of the venv.
unset PYTHONPATH
# Triton 3.6 bundles ptxas 12.9, which does not know sm_110a (Thor); use the CUDA 13.0 ptxas from the nvidia-cuda-nvcc wheel.
export TRITON_PTXAS_PATH=$LN_ROOT/.venv/lib/python3.12/site-packages/nvidia/cu13/bin/ptxas
export TRITON_PTXAS_BLACKWELL_PATH=$TRITON_PTXAS_PATH
