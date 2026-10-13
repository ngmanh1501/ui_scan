#!/usr/bin/env python3
"""Scan3D Studio: python launcher.py setup|start|check (Windows and Ubuntu)."""
from __future__ import annotations
import argparse
import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ENGINE = ROOT / 'engine'
FRONTEND = ROOT / 'frontend'
VENV_PYTHON = ROOT / '.venv' / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')

def run(*args, cwd=ROOT):
    print('+', ' '.join(map(str,args)), flush=True)
    subprocess.run(list(map(str,args)), cwd=cwd, check=True)

def npm():
    command = shutil.which('npm.cmd' if os.name == 'nt' else 'npm')
    if not command:
        raise RuntimeError('Node.js/npm not found. Install Node.js 22 LTS.')
    return command

def engine_path():
    name = 'scan3d_engine.exe' if os.name == 'nt' else 'scan3d_engine'
    paths = [ENGINE/'build'/'Release'/name, ENGINE/'build'/name]
    return next((path for path in paths if path.is_file()), paths[-1])

def setup():
    run(sys.executable, '-m', 'venv', ROOT/'.venv')
    run(VENV_PYTHON, '-m', 'pip', 'install', '-r', ROOT/'backend/requirements.txt')
    run('cmake', '-S', ENGINE, '-B', ENGINE/'build', *([] if os.name=='nt' else ['-DCMAKE_BUILD_TYPE=Release']))
    run('cmake', '--build', ENGINE/'build', '--config', 'Release', '--parallel', '2')
    run(npm(), 'ci', cwd=FRONTEND)
    run(npm(), 'run', 'build', cwd=FRONTEND)
    print('Setup complete. Start with: python launcher.py start')

def check():
    for label, value in [('OS',sys.platform), ('Python',sys.version.split()[0]),
        ('CMake',shutil.which('cmake') or 'MISSING'),
        ('npm',shutil.which('npm.cmd' if os.name=='nt' else 'npm') or 'MISSING'),
        ('venv',str(VENV_PYTHON) if VENV_PYTHON.is_file() else 'MISSING'),
        ('engine',str(engine_path()) if engine_path().is_file() else 'MISSING')]:
        print(f'{label}: {value}')

def free_port(port):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',port)) == 0:
            raise RuntimeError(f'Port {port} already in use. Stop the previous server.')

def stop(proc):
    if proc is not None and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

def start():
    if not VENV_PYTHON.is_file() or not (FRONTEND/'node_modules').exists():
        raise RuntimeError('Dependencies missing: run python launcher.py setup')
    for port in (8000,5173):
        free_port(port)
    backend = subprocess.Popen([str(VENV_PYTHON),'-m','uvicorn','backend.server:app',
        '--host','127.0.0.1','--port','8000'],cwd=ROOT)
    frontend = None
    try:
        frontend = subprocess.Popen([npm(),'run','dev','--','--strictPort'],cwd=FRONTEND)
        url='http://127.0.0.1:5173/'
        for _ in range(60):
            if backend.poll() is not None or frontend.poll() is not None:
                raise RuntimeError('A server exited. Inspect logs above.')
            try:
                with urllib.request.urlopen(url,timeout=.3) as response:
                    if response.status==200:
                        print('Scan3D Studio:',url,flush=True)
                        webbrowser.open(url)
                        break
            except Exception:
                pass
            time.sleep(.2)
        else:
            raise RuntimeError('Frontend failed to start.')
        while backend.poll() is None and frontend.poll() is None:
            time.sleep(.3)
        raise RuntimeError('A server stopped unexpectedly.')
    finally:
        stop(frontend)
        stop(backend)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['setup','check','start'])
    args=parser.parse_args()
    try:
        globals()[args.action]()
    except KeyboardInterrupt:
        print('Stopped.')
    except (RuntimeError,OSError,subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        sys.exit(1)
