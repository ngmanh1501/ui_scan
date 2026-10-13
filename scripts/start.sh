#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f .venv/bin/uvicorn ]]; then echo 'Run scripts/setup.sh first'; exit 1; fi
.venv/bin/uvicorn backend.server:app --host 127.0.0.1 --port 8000 &
BACKEND_PID=$!
(cd frontend && npm run dev -- --host 127.0.0.1) &
FRONTEND_PID=$!
trap 'kill "$BACKEND_PID" "$FRONTEND_PID" 2>/dev/null || true' EXIT
sleep 2
xdg-open http://127.0.0.1:5173 >/dev/null 2>&1 || true
wait
