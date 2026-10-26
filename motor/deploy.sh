#!/bin/bash
export FROGGY_PI_USER="${FROGGY_PI_USER:-froggy}"
[[ "$FROGGY_PI_USER" =~ ^[a-z_][a-z0-9_-]*$ ]] || { echo "Invalid FROGGY_PI_USER" >&2; exit 2; }
# deploy.sh: copy the motor code to froggy-pi and make sure pigpio is installed and running.
# Idempotent. Works with no internet on the Pi: if apt cannot reach the archive, it installs the
# pigpio .debs vendored in ./debs (Raspberry Pi bookworm arm64, sha256-checked) with dpkg -i.
# Host: $FROGGY_HOST (set by froggy-motor), else `froggy-motor --which-host` does the lookup
# (froggy-pi.local, then $FROGGY_PI).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
QUIET=""; [[ "${1:-}" == "--quiet" ]] && QUIET=1
say() { [[ -z "$QUIET" ]] && echo "$@"; return 0; }
HOST="${FROGGY_HOST:-$("$HERE/../bin/froggy-motor" --which-host 2>/dev/null)}"
if [[ -z "$HOST" ]]; then
  echo "deploy: froggy-pi unreachable (tried froggy-pi.local, \$FROGGY_PI): connect the Ethernet cable or hotspot" >&2
  exit 1
fi
SSH=(ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -o ControlMaster=auto
     -o "ControlPath=/tmp/froggy-ssh-%C" -o ControlPersist=60 "${FROGGY_PI_USER}@$HOST")
SCP=(scp -q -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -o "ControlPath=/tmp/froggy-ssh-%C")
"${SSH[@]}" true 2>/dev/null || { echo "deploy: ssh to ${FROGGY_PI_USER}@$HOST failed (ConnectTimeout 10)" >&2; exit 1; }
"${SSH[@]}" 'mkdir -p ~/froggy-motor/debs ~/froggy-motor/logs' || exit 1
FILES=("$HERE/froggy_motor.py")
[[ -f "$HERE/motor.toml" ]] && FILES+=("$HERE/motor.toml")
"${SCP[@]}" "${FILES[@]}" "${FROGGY_PI_USER}@$HOST:~/froggy-motor/" || { echo "deploy: copy failed" >&2; exit 1; }
"${SSH[@]}" 'dpkg -s pigpiod python3-pigpio >/dev/null 2>&1' || \
  "${SCP[@]}" "$HERE"/debs/*.deb "$HERE/debs/SHA256SUMS" "${FROGGY_PI_USER}@$HOST:~/froggy-motor/debs/" || { echo "deploy: copy of debs failed" >&2; exit 1; }
say "deploy: code copied to $HOST:~/froggy-motor/"
"${SSH[@]}" "QUIET='$QUIET' bash -s" <<'REMOTE'
set -u
say() { [[ -z "$QUIET" ]] && echo "$@"; return 0; }
cd ~/froggy-motor
if dpkg -s pigpiod python3-pigpio >/dev/null 2>&1; then
  say "pi: pigpio already installed ($(dpkg-query -W -f='${Version}' pigpiod))"
else
  ok=""
  if timeout 5 getent hosts archive.raspberrypi.com >/dev/null 2>&1; then
    echo "pi: network up, trying apt-get install pigpio python3-pigpio"
    sudo -n timeout 300 apt-get install -y pigpio python3-pigpio && ok=apt
  fi
  if [[ -z "$ok" ]]; then
    echo "pi: no apt, installing vendored debs with dpkg -i"
    (cd debs && sha256sum -c --quiet SHA256SUMS) || { echo "pi: vendored deb checksum mismatch" >&2; exit 1; }
    sudo -n dpkg -i debs/libpigpio1_*.deb debs/pigpiod_*.deb debs/pigpio-tools_*.deb debs/python3-pigpio_*.deb \
      && ok=dpkg || { echo "pi: dpkg -i failed (sudo -n needs passwordless sudo)" >&2; exit 1; }
  fi
  echo "pi: pigpio installed via $ok"
fi
sudo -n systemctl enable --now pigpiod >/dev/null 2>&1 || { echo "pi: could not start pigpiod" >&2; exit 1; }
for i in 1 2 3 4 5 6 7 8 9 10; do systemctl is-active --quiet pigpiod && break; sleep 0.5; done
systemctl is-active --quiet pigpiod || { echo "pi: pigpiod not active" >&2; exit 1; }
if [[ -z "$QUIET" ]]; then python3 froggy_motor.py status; fi
REMOTE
rc=$?
(( rc == 0 )) && say "deploy: ok"
exit $rc
