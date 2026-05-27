#!/usr/bin/env bash
set -euo pipefail

official_root="${1:-/tmp/unitree_rl_mjlab_official_baseline}"
device="${JOYSTICK_DEVICE:-/dev/input/js0}"
jstest="${official_root}/simulate/build/jstest"

printf 'joystick_device: %s\n' "${device}"

if [[ -d /dev/input ]]; then
  printf 'dev_input: present\n'
  find /dev/input -maxdepth 1 -type c -printf 'input_device: %p\n' 2>/dev/null \
    | sort || true
else
  printf 'dev_input: missing\n'
fi

if [[ -e "${device}" ]]; then
  printf 'device_exists: yes\n'
  if [[ -c "${device}" ]]; then
    printf 'device_char: yes\n'
  else
    printf 'device_char: no\n'
  fi
  if [[ -r "${device}" ]]; then
    printf 'device_readable: yes\n'
  else
    printf 'device_readable: no\n'
  fi
else
  printf 'device_exists: no\n'
fi

printf 'groups: %s\n' "$(id -nG)"
if id -nG | tr ' ' '\n' | grep -qx input; then
  printf 'in_input_group: yes\n'
else
  printf 'in_input_group: no\n'
fi

if [[ -x "${jstest}" ]]; then
  printf 'jstest_binary: %s\n' "${jstest}"
  set +e
  output="$("${jstest}" 2>&1)"
  status=$?
  set -e
  printf 'jstest_exit: %s\n' "${status}"
  if [[ -n "${output}" ]]; then
    printf 'jstest_output: %s\n' "${output}"
  fi
else
  printf 'jstest_binary: missing_or_not_executable\n'
fi

printf 'next_gate: scripts/agent/official_baseline_preflight.sh\n'
