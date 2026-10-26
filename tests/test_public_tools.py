"""Public setup regression tests: no network, Pi, card or motor access."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def fake_tools(tmp_path, names):
    log = tmp_path / "calls.jsonl"
    for name in names:
        p = tmp_path / name
        p.write_text(
            f"#!{sys.executable}\nimport sys,json,os\n"
            "body=sys.stdin.read() if sys.argv[-1].endswith('bash -s') else ''\n"
            "with open(os.environ['MOCK_LOG'],'a') as f:\n"
            " f.write(json.dumps({'argv':sys.argv,'stdin':body})+'\\n')\n"
        )
        p.chmod(0o755)
    return {**os.environ, "PATH": str(tmp_path) + os.pathsep + os.environ['PATH'],
            "MOCK_LOG": str(log)}, log


def test_deploy_uses_selected_login_and_remote_home(tmp_path):
    env, log = fake_tools(tmp_path, ['ssh', 'scp'])
    env.update(FROGGY_PI_USER='bench_user', FROGGY_HOST='example.test')
    r = subprocess.run(['bash', str(ROOT/'motor/deploy.sh')], env=env,
                       text=True, capture_output=True, timeout=10)
    assert r.returncode == 0, r.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert all(any('bench_user@example.test' in a for a in c['argv']) for c in calls)
    assert any('cd ~/froggy-motor' in c['stdin'] for c in calls)
    assert any(a.endswith(':~/froggy-motor/') for c in calls for a in c['argv'])


@pytest.mark.parametrize('script,shell', [
    ('motor/deploy.sh', 'bash'), ('bin/froggy-pi-flash', 'bash'),
    ('bin/froggy-pi-net', 'bash'), ('bin/froggy-motor', 'zsh'),
    ('bin/froggy-cam-live', 'zsh')])
def test_invalid_account_rejected_before_side_effects(script, shell):
    env = {**os.environ, 'FROGGY_PI_USER': 'bad;account'}
    r = subprocess.run([shell, str(ROOT/script)], env=env,
                       text=True, capture_output=True, timeout=5)
    assert r.returncode == 2
    assert 'Invalid FROGGY_PI_USER' in r.stderr


def test_main_pi_phonehome_sends_nothing_without_target(tmp_path):
    source = (ROOT/'bin/froggy-pi-flash').read_text()
    script = re.search(r'cat > "\$B/froggy-phonehome.sh" <<\'EOF\'\n(.*?)\nEOF',
                       source, re.S).group(1)
    script = script.replace('/boot/firmware', str(tmp_path))
    env, log = fake_tools(tmp_path, ['curl', 'systemctl'])
    r = subprocess.run(['sh', '-c', script], env=env, text=True,
                       capture_output=True, timeout=5)
    assert r.returncode == 0
    assert not log.exists()


def test_tone_dry_run_preserves_explicit_host_and_login(tmp_path):
    env, log = fake_tools(tmp_path, ['ssh'])
    env.update(FROGGY_PI_USER='bench_user', FROGGY_PI='example.test')
    r = subprocess.run([sys.executable, str(ROOT/'bin/froggy-motor-song'),
                        '--dry-run', 'scale'], env=env, text=True,
                       capture_output=True, timeout=5)
    assert r.returncode == 0, r.stderr
    assert 'bench_user@example.test' in r.stdout
    assert not log.exists()
