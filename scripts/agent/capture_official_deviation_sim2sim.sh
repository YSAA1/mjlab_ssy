#!/usr/bin/env bash
set -euo pipefail

root="${1:-/tmp/g1_official_automation_deviation}"
stamp="$(date +%Y%m%d-%H%M%S)"
logdir="logs/g1_tracking_phase1/${stamp}-official-deviation-capture"
mkdir -p "${logdir}"

sim_bin="${G1_SIM_BIN:-${root}/simulate/build/unitree_mujoco}"
ctrl_bin="${G1_CTRL_BIN:-${root}/deploy/robots/g1/build/g1_ctrl}"
manifest="${G1_DEVIATION_MANIFEST:-}"
network="${G1_DEVIATION_NETWORK:-wlo1}"
xvfb_bin="${G1_XVFB_BIN:-/tmp/xvfb-local/usr/bin/Xvfb}"
xvfb_display="${G1_XVFB_DISPLAY:-:104}"
video_size="${G1_CAPTURE_SIZE:-1280x720}"
fps="${G1_CAPTURE_FPS:-15}"
capture_seconds="${G1_CAPTURE_SECONDS:-22}"
ctrl_start_delay="${G1_CTRL_START_DELAY:-3}"

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
if [[ ! -x "${xvfb_bin}" ]]; then
  printf 'missing Xvfb binary: %s\n' "${xvfb_bin}" >&2
  exit 2
fi

video="${logdir}/sim2sim_capture.mp4"
summary="${logdir}/summary.txt"

cat >"${summary}" <<EOF
lane: official_source_plus_automation_deviation
claim: not_clean_official_baseline
root: ${root}
manifest: ${manifest}
logdir: ${logdir}
sim_bin: ${sim_bin}
ctrl_bin: ${ctrl_bin}
network: ${network}
xvfb_bin: ${xvfb_bin}
xvfb_display: ${xvfb_display}
video: ${video}
capture_seconds: ${capture_seconds}
ctrl_start_delay: ${ctrl_start_delay}
EOF

xvfb_pid=""
ffmpeg_pid=""
sim_pid=""

cleanup() {
  for pid in "${ffmpeg_pid}" "${sim_pid}" "${xvfb_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

"${xvfb_bin}" "${xvfb_display}" -screen 0 "${video_size}x24" -nolisten tcp \
  >"${logdir}/xvfb.log" 2>&1 &
xvfb_pid=$!
sleep 1
if ! kill -0 "${xvfb_pid}" 2>/dev/null; then
  printf 'xvfb failed to start; see %s\n' "${logdir}/xvfb.log" >&2
  exit 2
fi

ffmpeg -y -hide_banner -loglevel warning \
  -video_size "${video_size}" \
  -framerate "${fps}" \
  -f x11grab \
  -i "${xvfb_display}.0" \
  -t "${capture_seconds}" \
  -pix_fmt yuv420p \
  "${video}" >"${logdir}/ffmpeg.log" 2>&1 &
ffmpeg_pid=$!

(
  cd "${root}"
  DISPLAY="${xvfb_display}" timeout "${capture_seconds}s" "${sim_bin}" --network="${network}"
) >"${logdir}/unitree_mujoco.log" 2>&1 &
sim_pid=$!

sleep "${ctrl_start_delay}"

set +e
(
  cd "${root}/deploy/robots/g1/build"
  ctrl_seconds="$(python3 -c "import math; print(max(1, math.ceil(float('${capture_seconds}') - float('${ctrl_start_delay}'))))")"
  timeout "${ctrl_seconds}s" "${ctrl_bin}" --network="${network}"
) >"${logdir}/g1_ctrl.log" 2>&1
ctrl_status=$?
set -e

wait "${ffmpeg_pid}" || true
ffmpeg_pid=""

if kill -0 "${sim_pid}" 2>/dev/null; then
  sim_status=124
else
  wait "${sim_pid}" || sim_status=$?
  sim_status="${sim_status:-0}"
fi

printf 'sim_status: %s\n' "${sim_status}" >>"${summary}"
printf 'ctrl_status: %s\n' "${ctrl_status}" >>"${summary}"

if rg -n 'FSM:|Change state|DEVIATION|Connected|Policy directory|Loaded motion' "${logdir}" \
  >"${logdir}/fsm_extract.txt" 2>/dev/null; then
  printf 'fsm_extract: %s\n' "${logdir}/fsm_extract.txt" >>"${summary}"
fi

if [[ -s "${video}" ]]; then
  printf 'video_status: written\n' >>"${summary}"
else
  printf 'video_status: missing_or_empty\n' >>"${summary}"
fi

printf 'official deviation capture logdir: %s\n' "${logdir}"
cat "${summary}"
