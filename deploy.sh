#!/usr/bin/env bash
# 把 G1_VLN 同步到 Thor 的 /home/unitree/G1_VLN（venv、缓存、模型仍在 ~/lightnav、~/models，不动）。
#   ./deploy.sh                 同步
#   THOR=unitree@192.168.128.146 ./deploy.sh
set -euo pipefail
THOR="${THOR:-unitree@192.168.128.146}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rsync -a --delete --exclude 'logs/' --exclude '__pycache__/' --exclude '.git/' \
  --exclude 'deploy.sh' "$HERE/" "$THOR:/home/unitree/G1_VLN/"
ssh "$THOR" 'chmod +x ~/G1_VLN/client/start_vln.sh ~/G1_VLN/client/tools/* ~/G1_VLN/server/*.sh ~/G1_VLN/*.sh 2>/dev/null; mkdir -p ~/G1_VLN/client/logs; echo deployed'
