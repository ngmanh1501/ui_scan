"""Local-only API for Scan3D Studio. No remote execution or shell commands."""
from __future__ import annotations
import asyncio
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parents[1]
ENGINE_NAME = 'scan3d_engine.exe' if os.name == 'nt' else 'scan3d_engine'
ENGINE_PATHS = [ROOT / 'engine' / 'build' / 'Release' / ENGINE_NAME, ROOT / 'engine' / 'build' / ENGINE_NAME]
ENGINE = next((p for p in ENGINE_PATHS if p.is_file()), ENGINE_PATHS[-1])
DATA = ROOT / 'data'
DATA.mkdir(exist_ok=True)
ALLOWED = {'camera','projector','capture','tri','tri_multi','autocalib','build_model','locate','reload'}
app = FastAPI(title='Scan3D Studio Local API')
state = {'id': None, 'command': None, 'status': 'idle', 'started': None, 'exit_code': None, 'logs': []}
proc = None
lock = asyncio.Lock()

class RunRequest(BaseModel):
    command: str

class ConfigRequest(BaseModel):
    content: dict

def log(line):
    state['logs'].append(line.rstrip())
    if len(state['logs']) > 1500:
        del state['logs'][:500]

@app.get('/api/health')
def health():
    return {'ok': True, 'engine_exists': ENGINE.is_file(), 'engine_path': str(ENGINE), 'offline': True}

@app.get('/api/status')
def status():
    return state

@app.get('/api/devices')
def devices():
    # This is detection, not a guarantee of device readiness.
    if os.name == 'nt':
        try:
            from serial.tools import list_ports
            ports = sorted(p.device for p in list_ports.comports())
        except ImportError:
            ports = []
        return {'video_nodes': [], 'serial_ports': ports, 'display': True, 'camera_detection': 'OpenCV index from config.json'}
    video = sorted(str(p) for p in Path('/dev').glob('video*'))
    serial = sorted(str(p) for pat in ('ttyUSB*','ttyACM*') for p in Path('/dev').glob(pat))
    return {'video_nodes': video, 'serial_ports': serial, 'display': bool(os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY'))}

@app.get('/api/config')
def get_config():
    p = ROOT / 'engine' / 'config.json'
    if not p.exists():
        return {'exists': False, 'content': {}}
    try:
        return {'exists': True, 'content': json.loads(p.read_text())}
    except json.JSONDecodeError:
        raise HTTPException(422, 'Config is not valid JSON')

@app.put('/api/config')
def put_config(request: ConfigRequest):
    if state['status'] in ('running', 'cancelling'):
        raise HTTPException(409, 'Cannot update config during a scan')
    p = ROOT / 'engine' / 'config.json'
    if p.exists():
        p.with_suffix('.json.bak').write_bytes(p.read_bytes())
    p.write_text(json.dumps(request.content, indent=2, ensure_ascii=False) + '\n')
    return {'ok': True}

async def execute(command, job_id):
    global proc
    try:
        proc = await asyncio.create_subprocess_exec(str(ENGINE), '--command', command,
            cwd=str(ROOT / 'engine'), stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0,
            start_new_session=os.name != 'nt')
        async for raw in proc.stdout:
            log(raw.decode('utf-8', errors='replace'))
        code = await proc.wait()
        state['exit_code'] = code
        state['status'] = 'cancelled' if state['status'] == 'cancelling' else ('completed' if code == 0 else 'failed')
    except Exception as exc:
        log(f'[ERROR] {exc}')
        state['status'] = 'failed'
    finally:
        proc = None

@app.post('/api/jobs')
async def start_job(request: RunRequest):
    if request.command not in ALLOWED:
        raise HTTPException(400, 'Unsupported command')
    if not ENGINE.is_file():
        raise HTTPException(503, 'C++ engine not built; run python launcher.py setup')
    async with lock:
        if state['status'] in ('running', 'cancelling'):
            raise HTTPException(409, 'Another job is running')
        state.update(id=uuid.uuid4().hex, command=request.command, status='running',
                     started=time.time(), exit_code=None, logs=[])
        asyncio.create_task(execute(request.command, state['id']))
    return {'job_id': state['id']}

@app.post('/api/jobs/cancel')
async def cancel_job():
    if proc and proc.returncode is None:
        state['status'] = 'cancelling'
        if os.name == 'nt':
            proc.terminate()
        else:
            os.killpg(proc.pid, signal.SIGTERM)
        return {'ok': True}
    raise HTTPException(409, 'No running process')

@app.get('/api/files')
def files():
    found = []
    for base in (DATA, ROOT / 'engine'):
        for p in base.rglob('*.ply'):
            if p.is_file() and len(found) < 200:
                found.append({'id': str(p.relative_to(ROOT)), 'name': p.name, 'size': p.stat().st_size})
    return {'files': sorted(found, key=lambda x: x['name'])}

@app.get('/api/files/{file_path:path}')
def get_file(file_path: str):
    p = (ROOT / file_path).resolve()
    allowed_roots = [DATA.resolve(), (ROOT / 'engine').resolve()]
    if p.suffix.lower() != '.ply' or not any(p.is_relative_to(root) for root in allowed_roots) or not p.is_file():
        raise HTTPException(404, 'PLY not found')
    return FileResponse(p, media_type='application/octet-stream', filename=p.name if False else None)

@app.websocket('/api/ws/logs')
async def ws_logs(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(state)
            await asyncio.sleep(0.4)
    except (WebSocketDisconnect, RuntimeError):
        pass
