#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .venv
.venv/bin/pip install -r backend/requirements.txt
cmake -S engine -B engine/build -DCMAKE_BUILD_TYPE=Release
cmake --build engine/build -j "$(nproc)"
(cd frontend && npm install && npm run build)
echo 'Setup complete. Start with ./scripts/start.sh'
