"""Host selection regressions; no Docker daemon or display is needed."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('configure', Path(__file__).with_name('configure.py'))
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)


class HostSelection(unittest.TestCase):
    def select(self, system='Linux', release='generic', paths=(), env=None,
               nvidia=False, probe_ok=True):
        original_exists = Path.exists
        def exists(path):
            return str(path) in paths or original_exists(path)
        def run(args, timeout=15):
            if args[:3] == ['docker', 'context', 'inspect']:
                out = json.dumps([{'Endpoints': {'docker': {'Host': 'unix:///var/run/docker.sock'}}}])
            elif args[1] == 'info':
                out = json.dumps({'nvidia': {}} if nvidia else {})
            elif args[1] == 'run':
                return subprocess.CompletedProcess(args, 0 if probe_ok else 1, 'GPU', '')
            else:
                return subprocess.CompletedProcess(args, 1, '', '')
            return subprocess.CompletedProcess(args, 0, out, '')
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(config, 'GENERATED', Path(temp)), \
             patch.object(Path, 'home', return_value=Path(temp)), \
             patch.object(config, 'run', side_effect=run), \
             patch.object(config.platform, 'system', return_value=system), \
             patch.object(config.platform, 'release', return_value=release), \
             patch.object(Path, 'exists', exists), \
             patch.dict(os.environ, env or {}, clear=True):
            config.configure()
            return json.loads((Path(temp) / 'compose.json').read_text())['services']['devcontainer']

    def test_mac_browser_has_no_host_devices(self):
        value = self.select(system='Darwin', env={'DISPLAY': '/private/tmp/xquartz:0'})
        self.assertEqual(value['environment']['DISPLAY'], ':99')
        self.assertEqual(len(value['volumes']), 1)
        mount = value['volumes'][0]
        self.assertEqual(mount['type'], 'bind')
        self.assertEqual(mount['target'], '/home/ros/.codex')
        self.assertTrue(mount['source'].endswith('/.codex'))
        self.assertNotIn('devices', value)
        self.assertNotIn('deploy', value)

    def test_headless_linux_browser(self):
        self.assertEqual(self.select()['environment']['AUTOFACTORY_DISPLAY_MODE'], 'browser')

    def test_linux_cpu_preserves_native_display(self):
        value = self.select(paths=['/tmp/.X11-unix', '/dev/dri'],
                            env={'DISPLAY': ':1', 'AUTOFACTORY_GPU': 'cpu'})
        self.assertEqual(value['environment']['DISPLAY'], ':1')
        self.assertNotIn('devices', value)

    def test_linux_dri(self):
        value = self.select(paths=['/tmp/.X11-unix', '/dev/dri'], env={'DISPLAY': ':0'})
        self.assertEqual(value['devices'], ['/dev/dri:/dev/dri'])

    def test_nvidia_requires_successful_container_probe(self):
        options = dict(paths=['/tmp/.X11-unix'], env={'DISPLAY': ':0'}, nvidia=True)
        self.assertIn('deploy', self.select(**options))
        self.assertNotIn('deploy', self.select(**options, probe_ok=False))

    def test_wsl_uses_dxg_without_nvidia_toolkit(self):
        value = self.select(release='microsoft-standard-WSL2',
                            paths=['/tmp/.X11-unix', '/dev/dxg', '/usr/lib/wsl'], env={'DISPLAY': ':0'})
        self.assertEqual(value['devices'], ['/dev/dxg:/dev/dxg'])
        self.assertEqual(value['environment']['GALLIUM_DRIVER'], 'd3d12')
        self.assertNotIn('deploy', value)

    def test_explicit_browser_skips_gpu(self):
        value = self.select(paths=['/tmp/.X11-unix', '/dev/dri'], nvidia=True,
                            env={'DISPLAY': ':0', 'AUTOFACTORY_DISPLAY': 'browser'})
        self.assertEqual(value['environment']['DISPLAY'], ':99')
        self.assertNotIn('devices', value)
        self.assertNotIn('deploy', value)

    def test_invalid_mode_rejected(self):
        with self.assertRaises(SystemExit):
            self.select(env={'AUTOFACTORY_DISPLAY': 'typo'})

    def test_native_on_mac_rejected(self):
        with self.assertRaises(SystemExit):
            self.select(system='Darwin', env={'AUTOFACTORY_DISPLAY': 'native'})


if __name__ == '__main__':
    unittest.main()
