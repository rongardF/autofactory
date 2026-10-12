#!/usr/bin/env python3
"""Generate machine-specific Compose settings before container creation."""
import json
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[2]
GENERATED = ROOT / '.devcontainer' / '.generated'


def run(args, timeout=15):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None


def configure():
    GENERATED.mkdir(exist_ok=True)
    mode = os.environ.get('AUTOFACTORY_DISPLAY', 'auto')
    gpu = os.environ.get('AUTOFACTORY_GPU', 'auto')
    if mode not in ('auto', 'native', 'browser') or gpu not in ('auto', 'cpu'):
        raise SystemExit('Use AUTOFACTORY_DISPLAY=auto|native|browser and AUTOFACTORY_GPU=auto|cpu')
    # Bind mounts and devices refer to the Docker host, so reject remote daemons.
    context = run(['docker', 'context', 'inspect'])
    if context is None or context.returncode:
        raise SystemExit('Docker is unavailable; start Docker and retry Reopen in Container.')
    endpoint = os.environ.get('DOCKER_HOST') or json.loads(context.stdout)[0]['Endpoints']['docker']['Host']
    if endpoint.startswith(('ssh:', 'tcp:')):
        raise SystemExit('This Dev Container requires a local Docker daemon.')
    info = run(['docker', 'info', '--format', '{{json .Runtimes}}'])
    if info is None or info.returncode:
        raise SystemExit('Cannot access Docker; start Docker and check socket permissions.')
    linux = platform.system() == 'Linux'
    wsl = linux and 'microsoft' in platform.release().lower()
    display = os.environ.get('DISPLAY', '')
    native = linux and bool(display) and Path('/tmp/.X11-unix').exists()
    if mode == 'native' and not native:
        raise SystemExit('Native mode requires Linux/WSLg with DISPLAY and /tmp/.X11-unix; use browser mode here.')
    native = native and mode != 'browser'
    env = {'AUTOFACTORY_DISPLAY_MODE': 'native' if native else 'browser',
           'AUTOFACTORY_GPU': gpu, 'DISPLAY': display if native else ':99',
           'NVIDIA_VISIBLE_DEVICES': 'void'}
    # Keep personal Codex sessions and credentials on the host across rebuilds.
    codex_home = Path.home() / '.codex'
    codex_home.mkdir(mode=0o700, exist_ok=True)
    service = {'environment': env, 'volumes': [
        {'type': 'bind', 'source': str(codex_home), 'target': '/home/ros/.codex'}
    ]}
    if platform.system() == 'Darwin':
        # The existing Pylon SDK setup downloads x86_64 packages.
        service['platform'] = 'linux/amd64'
    if native:
        service['volumes'].append({'type': 'bind', 'source': '/tmp/.X11-unix',
                                   'target': '/tmp/.X11-unix', 'read_only': True})
        # Copy only this display's cookie and make its address family portable.
        authority = os.environ.get('XAUTHORITY', str(Path.home() / '.Xauthority'))
        cookies = run(['xauth', '-f', authority, 'nlist', display])
        cookie_file = GENERATED / 'Xauthority'
        cookie_file.unlink(missing_ok=True)
        if cookies and cookies.returncode == 0 and cookies.stdout.strip():
            data = '\n'.join('ffff' + line[4:] for line in cookies.stdout.splitlines()) + '\n'
            result = subprocess.run(['xauth', '-f', str(cookie_file), 'nmerge', '-'],
                                    input=data, text=True, capture_output=True)
            if result.returncode:
                raise SystemExit('Could not prepare Xauthority cookie.')
            cookie_file.chmod(0o600)
            env['XAUTHORITY'] = '/run/host.Xauthority'
            service['volumes'].append({'type': 'bind', 'source': str(cookie_file),
                                       'target': '/run/host.Xauthority', 'read_only': True})
        if gpu != 'cpu' and wsl and Path('/dev/dxg').exists() and Path('/usr/lib/wsl').exists():
            service['devices'] = ['/dev/dxg:/dev/dxg']
            service['volumes'].append({'type': 'bind', 'source': '/usr/lib/wsl',
                                       'target': '/usr/lib/wsl', 'read_only': True})
            env['LD_LIBRARY_PATH'] = '/usr/lib/wsl/lib'
            env['GALLIUM_DRIVER'] = 'd3d12'
        elif gpu != 'cpu' and not wsl:
            # Test actual device injection; the small probe image is cached after first use.
            runtimes = json.loads(info.stdout)
            if 'nvidia' in runtimes:
                probe = run(['docker', 'run', '--rm', '--pull=missing', '--network=none',
                             '--gpus=all', '--entrypoint', 'nvidia-smi',
                             'ubuntu:24.04', '-L'], timeout=45)
                if probe and probe.returncode == 0:
                    service['deploy'] = {'resources': {'reservations': {'devices': [
                        {'driver': 'nvidia', 'count': 'all', 'capabilities': ['gpu']}]}}}
                    env['NVIDIA_VISIBLE_DEVICES'] = 'all'
                    env['NVIDIA_DRIVER_CAPABILITIES'] = 'graphics,utility,compute,display'
            if 'deploy' not in service and Path('/dev/dri').exists():
                service['devices'] = ['/dev/dri:/dev/dri']
    target = GENERATED / 'compose.json'
    target.write_text(json.dumps({'services': {'devcontainer': service}}, indent=2) + '\n')
    print('Dev Container display: ' + env['AUTOFACTORY_DISPLAY_MODE'] +
          '; rendering will be checked inside the container.')


if __name__ == '__main__':
    configure()
