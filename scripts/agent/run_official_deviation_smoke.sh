#!/usr/bin/env bash
set -euo pipefail

root="${1:-/tmp/g1_official_automation_deviation}"
stamp="$(date +%Y%m%d-%H%M%S)"
logdir="logs/g1_tracking_phase1/${stamp}-official-deviation-smoke"
mkdir -p "${logdir}"

sim_bin="${G1_SIM_BIN:-${root}/simulate/build/unitree_mujoco}"
ctrl_bin="${G1_CTRL_BIN:-${root}/deploy/robots/g1/build/g1_ctrl}"
manifest="${G1_DEVIATION_MANIFEST:-}"
network="${G1_DEVIATION_NETWORK:-lo}"
xvfb_bin="${G1_XVFB_BIN:-}"
xvfb_display="${G1_XVFB_DISPLAY:-:99}"
ctrl_start_delay="${G1_CTRL_START_DELAY:-3}"
xvfb_pid=""

if [[ -z "${manifest}" ]]; then
  for candidate in \
    "${root}/UNITREE_SIM2SIM_MANIFEST.json" \
    "${root}/AUTOMATION_DEVIATION_MANIFEST.json"; do
    if [[ -f "${candidate}" ]]; then
      manifest="${candidate}"
      break
    fi
  done
fi
manifest="${manifest:-${root}/UNITREE_SIM2SIM_MANIFEST.json}"

if [[ ! -x "${sim_bin}" ]]; then
  printf 'missing simulator binary: %s\n' "${sim_bin}" >&2
  exit 2
fi
if [[ ! -x "${ctrl_bin}" ]]; then
  printf 'missing controller binary: %s\n' "${ctrl_bin}" >&2
  exit 2
fi
if [[ ! -f "${manifest}" ]] || \
  ! grep -q '"claim": "not_clean_official_baseline"' "${manifest}"; then
  printf 'missing explicit automation-deviation manifest: %s\n' "${manifest}" >&2
  exit 2
fi

printf 'lane: official_source_plus_automation_deviation\n' >"${logdir}/summary.txt"
printf 'claim: not_clean_official_baseline\n' >>"${logdir}/summary.txt"
printf 'root: %s\n' "${root}" >>"${logdir}/summary.txt"
printf 'manifest: %s\n' "${manifest}" >>"${logdir}/summary.txt"
printf 'logdir: %s\n' "${logdir}" >>"${logdir}/summary.txt"
printf 'display: %s\n' "${DISPLAY:-<unset>}" >>"${logdir}/summary.txt"
printf 'network: %s\n' "${network}" >>"${logdir}/summary.txt"
printf 'xvfb_bin: %s\n' "${xvfb_bin:-<unset>}" >>"${logdir}/summary.txt"
printf 'ctrl_start_delay: %s\n' "${ctrl_start_delay}" >>"${logdir}/summary.txt"

sim_pid=""
cleanup() {
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    kill "${sim_pid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
  fi
  if [[ -n "${xvfb_pid}" ]] && kill -0 "${xvfb_pid}" 2>/dev/null; then
    kill "${xvfb_pid}" 2>/dev/null || true
    wait "${xvfb_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ -n "${xvfb_bin}" ]]; then
  if [[ ! -x "${xvfb_bin}" ]]; then
    printf 'xvfb binary is not executable: %s\n' "${xvfb_bin}" >&2
    exit 2
  fi
  "${xvfb_bin}" "${xvfb_display}" -screen 0 1280x720x24 -nolisten tcp \
    >"${logdir}/xvfb.log" 2>&1 &
  xvfb_pid=$!
  export DISPLAY="${xvfb_display}"
  sleep 1
  if ! kill -0 "${xvfb_pid}" 2>/dev/null; then
    printf 'xvfb failed to start; see %s\n' "${logdir}/xvfb.log" >&2
    exit 2
  fi
  printf 'display_after_xvfb: %s\n' "${DISPLAY}" >>"${logdir}/summary.txt"
fi

(
  cd "${root}"
  timeout 30s "${sim_bin}" --network="${network}"
) >"${logdir}/unitree_mujoco.log" 2>&1 &
sim_pid=$!

sleep "${ctrl_start_delay}"

set +e
(
  cd "${root}/deploy/robots/g1/build"
  timeout 22s "${ctrl_bin}" --network="${network}"
) >"${logdir}/g1_ctrl.log" 2>&1
ctrl_status=$?
set -e

if kill -0 "${sim_pid}" 2>/dev/null; then
  sim_status=124
else
  wait "${sim_pid}" || sim_status=$?
  sim_status="${sim_status:-0}"
fi

printf 'sim_status: %s\n' "${sim_status}" >>"${logdir}/summary.txt"
printf 'ctrl_status: %s\n' "${ctrl_status}" >>"${logdir}/summary.txt"

if rg -n 'error|failed|cannot|No such|DISPLAY|glfw|joystick|open failed' "${logdir}" \
  >"${logdir}/error_extract.txt" 2>/dev/null; then
  printf 'error_extract: %s\n' "${logdir}/error_extract.txt" >>"${logdir}/summary.txt"
fi

if rg -n 'DEVIATION:|FSM:' "${logdir}" >"${logdir}/fsm_extract.txt" 2>/dev/null; then
  printf 'fsm_extract: %s\n' "${logdir}/fsm_extract.txt" >>"${logdir}/summary.txt"
fi

printf 'official deviation smoke logdir: %s\n' "${logdir}"
cat "${logdir}/summary.txt"
