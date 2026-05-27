G1 Phase-1 Tracking Evidence
============================

This page documents the phase-1 log evidence contract for G1 tracking
sim2sim2 and real-deploy triage. It is intentionally local-runtime aware:
instrumentation under ``.external/unitree_rl_mjlab`` is a machine-local deploy
patch and must not be committed as mjlab source.

For durable G1 Unitree sim2sim lane preparation, use
:doc:`g1_unitree_sim2sim`. This page remains a phase-1 diagnostic and
historical evidence reference. The old phase-1 wrappers are compatibility
entry points; new reproducible preparation should go through
``uv run unitree-sim2sim prepare-g1``.

Log Schema
----------

The parser consumes timestamped controller logs and the following minimal
``[PHASE1]`` events:

.. code-block:: text

   [PHASE1] event=trigger action=flying_kick command=RB+X
   [PHASE1] event=motion_frame action=flying_kick frame=0 motion_t=0.00
   [PHASE1] event=policy_step action=flying_kick step=0
   [PHASE1] event=lowcmd_write action=flying_kick step=0
   [PHASE1] event=q_response action=flying_kick q_err_l2=0.20 dq_err_l2=0.30 base_vel_x=0.00 command_vel_x=0.00 gravity_b=(0.000,0.000,-1.000)
   [PHASE1] event=stable_sample state=Velocity stable=1 q_err_l2=0.10 base_vel_x=0.00 command_vel_x=0.00 gravity_b=(0.000,0.000,-1.000)

FSM lines define action episodes:

.. code-block:: text

   FSM: Change state from Velocity to Mimic_FlyingKick
   FSM: Change state from Mimic_FlyingKick to Velocity

The required timing offsets are:

- ``trigger_to_fsm_s``
- ``fsm_to_motion_s``
- ``motion_to_policy_s``
- ``policy_to_lowcmd_s``
- ``lowcmd_to_q_response_s``

The post-action stability window starts at the action-end FSM transition and
stops before the next mimic action starts. Passing requires return to
``Velocity`` or another explicitly approved stable policy state and at least
5 seconds of stable samples. ``FixStand`` alone is not a phase-1 standing
acceptance state because it is only a PD pose-transition state in the Unitree
FSM.

Parser Usage
------------

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --fixtures tests/fixtures/g1_phase1/pass
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --fixtures tests/fixtures/g1_phase1/fail --expect-failure
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir logs/<phase1-run>

Missing required timing fields are a blocking ``insufficient_timing_evidence``
result. They must never be treated as a pass.

Local Runtime Instrumentation
-----------------------------

If existing logs only contain FSM and ``[GETUP-DIAG]`` lines, add temporary
``[PHASE1]`` logging in these local runtime files, rebuild the Unitree deploy
binary, and rerun sim2sim2:

- ``/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_Mimic.cpp``
- ``/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp``
- ``/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_FixStand.h``

The parser reports these paths in ``instrumentation_needed`` when logs are
insufficient. Keep those deploy edits local; commit only the mjlab parser,
fixtures, and evidence summary paths.

2026-05-22 Sim2sim2 Classification
----------------------------------

Phase-1 preflight passed against the latest manifest:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json

Preflight evidence:

- ``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/contract_report.json``
- ``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/sim2sim2_preflight.json``

The selected simulator config remained simulation-only:

- ``interface: lo``
- ``robot: g1``
- ``robot_scene: src/assets/robots/unitree_g1/xmls/scene_g1.xml``
- ``use_joystick: 0``

Two start attempts were run and then stopped/restored:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action flying_kick --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir logs/flying_kick_sim2sim/20260522-140745 --expect-failure
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh restore --action flying_kick logs/flying_kick_sim2sim/20260522-140745

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action roundhouse_leading_right --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir logs/roundhouse_leading_right_sim2sim/20260522-140906 --expect-failure
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh restore --action roundhouse_leading_right logs/roundhouse_leading_right_sim2sim/20260522-140906

Both attempts produced simulator/controller logs and selected config snapshots:

- ``logs/flying_kick_sim2sim/20260522-140745/``
- ``logs/roundhouse_leading_right_sim2sim/20260522-140906/``

Both attempts are classified as ``insufficient_timing_evidence``. The controller
connected to the simulator and loaded both mimic states, but each log only
reached ``FSM: Start FixStand``. No Mimic episode was entered, and no
``[PHASE1]`` timing events were present. This is not a passing sim2sim2
demonstration.

Root cause for the no-episode result: ``start`` used the default
``stand`` mode with ``start_paused=1``. That path starts in ``FixStand`` and
requires manual Run/trigger input, so it cannot produce deterministic Mimic
evidence by itself.

For a deterministic direct-Mimic diagnostic run, use:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action flying_kick --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json --mode play_parity --start-paused 0

The first direct-Mimic runs showed that action execution was observable, but
both actions still failed. Returning from Mimic to ``FixStand`` was then
changed to return to ``Velocity`` for sim2sim diagnosis. That falsified the
``FixStand``-only handoff hypothesis: flying kick still failed after returning
to ``Velocity``.

After adding local ``[PHASE1]`` instrumentation and rebuilding
``.external/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl``, direct-Mimic
runs produced complete timing offsets and no longer reported insufficient
timing evidence:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action flying_kick --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json --mode play_parity --start-paused 0
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir logs/flying_kick_sim2sim/20260522-143349 --expect-failure
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh restore --action flying_kick logs/flying_kick_sim2sim/20260522-143349

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action roundhouse_leading_right --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json --mode play_parity --start-paused 0
   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir logs/roundhouse_leading_right_sim2sim/20260522-143755 --expect-failure
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh restore --action roundhouse_leading_right logs/roundhouse_leading_right_sim2sim/20260522-143755

This is the current root cause for the direct-Mimic sim2sim diagnosis: action
execution is observable, timing evidence is complete, but both actions become
unstable during Mimic before the return-state controller can recover them. Both
latest reports fail with ``policy_action_to_joint_response_mismatch`` and empty
``missing_evidence``.

The flying-kick report first crosses the bad-gravity threshold during the action
at line 112, before the line 528 transition back to ``Velocity``. The roundhouse
report first crosses the same threshold during the action at line 206, before
the line 636 transition back to ``Velocity``.

Latest classified evidence:

- ``logs/flying_kick_sim2sim/20260522-143349/phase1_log_analysis.json``
- ``logs/roundhouse_leading_right_sim2sim/20260522-143755/phase1_log_analysis.json``

Entry-Pose Diagnosis
--------------------

The narrower diagnosis under ``policy_action_to_joint_response_mismatch`` is an
entry-state pose mismatch. The mjlab tracking task resets the simulator to the
reference motion state at frame 0: ``sampling_mode: start``, empty pose/velocity
ranges, and ``joint_position_range: (0.0, 0.0)``. The Unitree deploy path cannot
teleport the robot into that reference state; the direct-Mimic sim2sim run starts
from the deploy standing/default pose and immediately switches to Mimic.

Run the entry-gap check with:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_entry_gap.py \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --log logs/flying_kick_sim2sim/20260522-143349/g1_ctrl.log \
     --log logs/roundhouse_leading_right_sim2sim/20260522-143755/g1_ctrl.log \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_gap_report.json \
     --expect-failure

Current evidence:

- flying kick frame-0 default-vs-reference gap: ``1.914`` L2, max joint gap
  ``1.048``; the first logged q error is ``1.915``.
- roundhouse frame-0 default-vs-reference gap: ``1.936`` L2, max joint gap
  ``0.980``; the first logged q error is ``2.027``.
- the best natural frame for the flying-kick motion is still ``1.116`` L2 from
  deploy default pose; for roundhouse it is still ``1.732`` L2. There is no
  simple ``time_start`` frame that makes direct entry from default standing look
  like the trained reset condition.

This keeps the phase-1 verdict blocked and does not unlock any real-robot
trial. The next fix direction is an explicit entry-state solution: first test
the controller-reproducible prepose candidate in sim2sim2; if that fails, use a
stronger transition controller or a phase-2 training variant that includes
deployment entry-state perturbations and handoff robustness.

Entry-State Handoff Gate
------------------------

Use the handoff gate to distinguish a real deploy entry from a sim-only
teleport:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_handoff_gate.py \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_handoff_gate.json \
     --expect-blocked

The dual-action wrapper exposes the same gate without launching the simulator:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh entry-gate \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --expect-blocked

Current handoff-gate evidence:

- report path:
  ``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_handoff_gate.json``
- flying kick: the active sim ``initial_qpos`` matches the reference frame-0
  qpos exactly, so it is classified as ``sim_teleport_only`` and
  ``deploy_acceptance_candidate: false``.
- roundhouse: the active sim ``initial_qpos`` does not match the roundhouse
  reference frame-0 qpos; it is the wrong action's sim initial state for this
  check.
- both actions still fail ``deploy_default_entry`` with
  ``entry_state_pose_mismatch``.
- both actions report ``deploy_safe_transition_entry.available: false`` because
  no post-entry sim2sim2 evidence has passed yet. The wrapper provides a
  ``sim2sim_prepose_mode`` candidate, but it is only a candidate until the
  action passes the sim2sim2 gate.

The direct work item 6 handoff result is therefore blocked, not accepted. A
matching ``initial_qpos`` is useful for cause isolation, but it cannot unlock
real hardware unless the controller can reproduce the same entry without
teleporting state.

For the next sim2sim2 run, the wrapper now has a deploy-style prepose mode:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode prepose \
     --start-paused 1

``prepose`` keeps the simulator starting from the stand qpos, sets ``FixStand``
to interpolate to the action's motion frame-0 joint pose, and then waits for the
Mimic trigger. That makes it a controller-reproducible candidate to test in
Unitree MuJoCo. It is still not real-robot acceptance until both actions pass
the 5-second sim2sim2 gate with logs and video/screenshot evidence.

2026-05-22 Prepose Sim2sim2 Evidence
------------------------------------

Both actions were rerun through Unitree MuJoCo with the deploy-style prepose
entry. The first auto-run attempts were kept as diagnostic-only because the
MuJoCo screenshots still showed ``PAUSE`` even though the helper printed an
auto-start message:

- ``logs/flying_kick_sim2sim/20260522-153100/``
- ``logs/roundhouse_leading_right_sim2sim/20260522-153225/``
- ``logs/flying_kick_sim2sim/20260522-154417/``
- ``logs/roundhouse_leading_right_sim2sim/20260522-154539/``

The final runs below use the hardened auto-run path, which clicks MuJoCo's
``Run`` radio button after ``FixStand`` is ready:

.. code-block:: bash

   MJLAB_AUTO_RUN_AFTER_READY=1 \
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode prepose \
     --start-paused 1

   MJLAB_AUTO_RUN_AFTER_READY=1 \
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action roundhouse_leading_right \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode prepose \
     --start-paused 1

Final evidence directories:

- ``logs/flying_kick_sim2sim/20260522-155756/``
- ``logs/roundhouse_leading_right_sim2sim/20260522-155950/``

Each directory contains:

- ``g1_ctrl.log``
- ``unitree_mujoco.log``
- ``selected/config.yaml`` and ``selected/simulate_config.yaml``
- ``hash_selected.txt``
- ``phase1_log_analysis.json``
- ``mujoco_scene.png`` at ``1706x960``

The roundhouse directory also includes ``mujoco_scene_aligned.png`` because the
robot moved out of the initial camera view after falling.

Both runs passed the new-G1 / loopback preflight before launch:
``interface=lo``, ``robot=g1``,
``robot_scene=src/assets/robots/unitree_g1/xmls/scene_g1.xml``, and
``use_joystick=0``.

The prepose result is not a pass:

- flying kick started in ``FixStand`` and did not enter ``Mimic_FlyingKick``.
  ``mujoco_scene.png`` shows MuJoCo in ``Run`` with the robot fallen. The log
  transitions from upright at line 26 to unstable at line 27:
  ``q_err_l2=3.662``, ``q_err_max=2.246``, and
  ``gravity_b=(-0.247,-0.762,-0.598)``.
- roundhouse started in ``FixStand`` and did not enter
  ``Mimic_RoundhouseLeadingRight``. ``mujoco_scene.png`` shows MuJoCo in
  ``Run`` and ``mujoco_scene_aligned.png`` shows the fallen robot after camera
  alignment. The log transitions from upright at line 26 to unstable at line
  27: ``q_err_l2=4.013``, ``q_err_max=2.411``, and
  ``gravity_b=(-0.090,-0.935,-0.343)``.
- the phase-1 parser reports ``primary_reason: insufficient_timing_evidence``
  for both final evidence directories because no Mimic episode exists:
  ``event_counts.phase1=163`` for flying kick and ``269`` for roundhouse.
- the entry-gap checker report
  ``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_gap_after_prepose.json``
  still reports ``primary_reason: entry_state_pose_mismatch``. The latest logs
  have no first Mimic ``q_response`` because the controller never transitioned
  out of ``FixStand``.

This closes work item 7 as blocked. The prepose candidate is
controller-reproducible enough to test, and the final evidence confirms MuJoCo
was actually running. It is not stable enough to become a deploy-safe entry
contract. Work item 8 remains locked. The next route is diagnosis of the
prepose entry controller/transition, or a phase-2 training variant with
deployment entry-state perturbations and handoff robustness.

Official Elastic-Band Bootstrap Mode
------------------------------------

The public Unitree G1/H1 sim2sim flow uses ``unitree_mujoco`` plus
``g1_ctrl`` with the simulator elastic band enabled. In that flow, MuJoCo key
``8`` lowers the elastic-band-supported robot toward the ground, and key ``9``
releases the band only after the policy state is stable. A free-standing
``FixStand`` run with ``enable_elastic_band=0`` is therefore a stricter
diagnostic than the documented sim bootstrap path.

The phase-1 wrapper now exposes that bootstrap path explicitly:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode official_bootstrap

``official_bootstrap`` maps the controller config to the deploy stand qpos and
defaults the simulator to ``enable_elastic_band=1`` and ``start_paused=1``. It
does not auto-run by default.

The first live smoke attempt,
``logs/flying_kick_sim2sim/20260522-163437/``, proved why that matters: the
local Unitree elastic band starts enabled with anchor ``(0, 0, 3)`` and
``length=0``, so clicking ``Run`` before pre-tensioning the band pulls the
robot upward before any policy state can run. The corrected automation only
prepares and clicks ``Run`` when ``MJLAB_AUTO_RUN_AFTER_READY=1`` is explicitly
set. In that explicit auto-run path, the helper sends key ``8`` before
clicking ``Run`` to lengthen the band first; it no longer sends the ``space``
toggle before pre-tensioning. The default pre-tension is 24 key-8 steps, which
matches the local anchor height much more closely than the failed length-zero
startup.

The paused-only smoke after the safety fix is
``logs/flying_kick_sim2sim/20260522-164258/``. It confirms:

- wrapper output: ``Mode: official_bootstrap; config mode: stand; elastic band:
  1; start paused: 1; auto run: 0``;
- selected sim config: ``enable_elastic_band: 1`` and ``start_paused: 1``;
- selected controller config: ``FixStand`` targets the deploy stand qpos;
- controller log stayed in stable ``FixStand`` samples with ``q_err_l2=0`` and
  ``gravity_b=(0,0,-1)`` while MuJoCo stayed paused;
- screenshot evidence:
  ``logs/flying_kick_sim2sim/20260522-164258/mujoco_paused_safe.png``.

It does not press key ``9`` automatically; releasing the elastic band is part
of the next evidence run and should happen only after the policy state is
stable enough to judge. This mode is sim bootstrap evidence only and does not
unlock real-robot work without a subsequent 5-second gate pass.

The explicit auto-run smoke with 24 key-8 pre-tension steps is
``logs/flying_kick_sim2sim/20260522-165042/``. It no longer shows the
length-zero elastic-band launch failure, but the robot falls after entering
MuJoCo ``Run`` while still in ``FixStand``. The first unstable sample appears
around ``2026-05-22 16:50:48.585`` with ``q_err_l2=4.246`` and
``gravity_b=(-0.018,-0.019,-1.000)``, followed by a clear tip at
``2026-05-22 16:50:49.085`` with
``gravity_b=(0.866,-0.374,0.332)``. Screenshot evidence is
``logs/flying_kick_sim2sim/20260522-165042/mujoco_run_fall_after_pretension.png``.

That run changes the diagnosis boundary: elastic pre-tension fixed the
upward-pull startup failure, but ``FixStand`` itself is not a deploy standing
gate. In the Unitree FSM it is a PD pose-transition state, while sustained
balance should be judged in ``Velocity`` or in the tracking policy state. The
next sim2sim2 diagnostic should therefore use a Velocity-first bootstrap before
triggering Mimic, not a free-standing ``FixStand`` hold as acceptance evidence.

Velocity-First Bootstrap Mode
-----------------------------

Use ``velocity_bootstrap`` to test the deploy Velocity controller before any
Mimic trigger:

.. code-block:: bash

   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode velocity_bootstrap

This mode starts from the deploy stand qpos, keeps ``Passive`` and
``FixStand`` available in the generated FSM config, but sets
``FSM.initial_state`` to ``Velocity`` for the sim diagnostic. It defaults to
``enable_elastic_band=1``, ``start_paused=1``, and no auto-run. If
``MJLAB_AUTO_RUN_AFTER_READY=1`` is explicitly set, the helper waits for
``FSM: Start Velocity``, sends key ``8`` to pre-tension the elastic band, and
only then clicks MuJoCo ``Run``.

The action transitions from ``Velocity`` are kept action-specific:
``Mimic_FlyingKick`` uses ``RB + X`` and
``Mimic_RoundhouseLeadingRight`` uses ``RB + Y``. This is still phase-1
sim2sim diagnostic evidence; real-robot work remains locked until both actions
complete and pass the 5-second stable-state gate.

The first paused-only Velocity bootstrap smoke is
``logs/flying_kick_sim2sim/20260522-170654/``. It confirms:

- selected controller config has ``FSM.initial_state: Velocity`` and
  ``Velocity.policy_dir: config/policy/velocity``;
- selected sim config has ``enable_elastic_band: 1``, ``start_paused: 1``, and
  the deploy stand qpos;
- the controller resolves the actual Velocity policy directory to
  ``config/policy/velocity/v0``;
- the controller log reaches ``FSM: Start Velocity`` and records 117
  ``[PHASE1] event=stable_sample state=Velocity stable=1`` samples while
  paused;
- screenshot evidence is
  ``logs/flying_kick_sim2sim/20260522-170654/mujoco_velocity_bootstrap_paused.png``;
- after restore, no ``g1_ctrl`` or ``unitree_mujoco`` process remains active,
  and the external sim config returns to ``enable_elastic_band: 0``.

The first explicit Velocity ``Run`` smoke is
``logs/flying_kick_sim2sim/20260522-171001/``. The helper waited for
``FSM: Start Velocity``, sent 24 key-8 pre-tension steps, clicked MuJoCo
``Run``, and did not release the elastic band with key ``9``. This run fails
before any Mimic trigger:

- the log is stable in ``Velocity`` from ``17:10:03.941`` through
  ``17:10:06.921``;
- the first failing sample is line 33 at ``17:10:07.421`` with
  ``stable=0``, ``q_err_l2=5.614``,
  ``gravity_b=(0.150,-0.963,-0.223)``, and
  ``root_ang_vel_l2=6.364``;
- screenshot evidence is
  ``logs/flying_kick_sim2sim/20260522-171001/mujoco_velocity_bootstrap_run_fall.png``;
- after restore, no ``g1_ctrl`` or ``unitree_mujoco`` process remains active.

This is now a Velocity deploy/sim contract failure, not a FixStand failure and
not a tracking-policy Mimic failure. The next diagnosis should inspect the
Velocity policy asset, stand qpos versus Velocity policy default offset, action
target scale/gains, and whether the new G1 mode-15 sim asset matches the
Velocity policy's training/deploy robot contract.

Run the Velocity contract report with:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py \
     --evidence-dir logs/flying_kick_sim2sim/20260522-171001 \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_bootstrap_report.json \
     --expect-failure

Current report:

- ``primary_reason: velocity_default_pose_mismatch``;
- all deploy dimensions are 29: ``joint_ids_map``, ``default_joint_pos``,
  action offset/scale, stiffness, damping, and selected sim joint qpos;
- selected ``Velocity.policy_dir`` resolves to
  ``config/policy/velocity/v0`` with ``policy.onnx`` present;
- stand qpos versus Velocity ``default_joint_pos`` / action offset:
  ``gap_l2=0.779069`` and ``gap_max=0.369``;
- largest mismatches are both knee joints at indices 3 and 9
  (``0.669`` initial versus ``0.3`` policy default), followed by arm joints
  18 and 25 (``0.6`` initial versus ``0.87`` policy default);
- runtime stability confirms the pose mismatch is not merely cosmetic:
  first unstable sample occurs at line 33 with ``q_err_l2=5.614`` and
  ``root_ang_vel_l2=6.364``.

The immediate remediation should test a Velocity bootstrap that starts from
the Velocity policy default/action-offset pose, or retrain/export a Velocity
policy whose deploy offset matches the new mode-15 stand qpos. Do not trigger
the kick Mimic policies until this upstream Velocity gate is stable.

The policy-default Velocity bootstrap check uses the same root qpos and
new-G1 mode-15 scene, but initializes the 29 joint positions from the resolved
Velocity policy ``default_joint_pos``:

.. code-block:: bash

   MJLAB_AUTO_RUN_AFTER_READY=1 \
   MJLAB_ELASTIC_PRETENSION_STEPS=24 \
   MJLAB_ELASTIC_DROP_STEPS=0 \
   MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default \
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode velocity_bootstrap

Evidence directory:
``logs/flying_kick_sim2sim/20260522-172725/``.

The initial follow-up contract report was later extended to include ONNX
metadata, deploy observation dimensions, selected-root contact geometry, and
current mjlab G1 source init-state. The report is
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_default_bootstrap_report.json``.
It falsifies the narrower default-pose-only hypothesis and narrows the failure
to an initial root/contact mismatch:

- ``primary_reason: velocity_initial_contact_mismatch``;
- selected sim ``initial_qpos[7:]`` exactly matches Velocity
  ``default_joint_pos`` / action offset: ``gap_l2=0.0`` and ``gap_max=0.0``;
- all 29-DoF deploy dimension checks still pass;
- ONNX input/output are ``obs[1,98] -> actions[1,29]`` and deploy observation
  terms also total 98 dims;
- ONNX metadata reports ``run_path: 2026-03-18_18-40-20`` and observation names
  ``base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
  actions``;
- current mjlab G1 source init-state is ``KNEES_BENT_KEYFRAME`` with root z
  ``0.76`` while the selected policy-default run used root z ``0.765781``;
- the selected root z is ``0.017894`` below the current ``HOME_KEYFRAME`` root z
  ``0.783675``;
- MuJoCo geometry check on the selected initial qpos finds the lowest foot
  collision surface at ``-0.018422``, requiring about ``0.018422`` m of root
  lift to clear the floor;
- the controller reaches ``FSM: Start Velocity`` and initially logs
  ``q_err_l2=0.000``;
- the first unstable sample is line 34 at ``17:27:31.859`` with
  ``q_err_l2=6.547``, ``q_err_max=5.002``,
  ``gravity_b=(-0.141,-0.822,0.552)``, and
  ``root_ang_vel_l2=3.656``;
- screenshot evidence is
  ``logs/flying_kick_sim2sim/20260522-172725/mujoco_velocity_policy_default_run_fall.png``;
- after stop/restore, no ``g1_ctrl`` or ``unitree_mujoco`` process remained
  active, and the external sim config returned to ``enable_elastic_band: 0``.

So the stand qpos versus policy default mismatch is a real bug in the first
Velocity run, but it is not sufficient by itself. The policy-default run then
combines Velocity policy joints with a knees-bent/root-low initial height, which
starts the policy-default pose with foot collision geoms below the floor. The
next diagnostic should test a grounded policy-default root height before any
Mimic trigger or real-robot trial.

The grounded follow-up keeps ``MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default``
and adds ``MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`` so the root height matches the
current G1 ``HOME_KEYFRAME``:

.. code-block:: bash

   MJLAB_AUTO_RUN_AFTER_READY=1 \
   MJLAB_ELASTIC_PRETENSION_STEPS=24 \
   MJLAB_ELASTIC_DROP_STEPS=0 \
   MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default \
   MJLAB_VELOCITY_BOOTSTRAP_ROOT=home \
   scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start \
     --action flying_kick \
     --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json \
     --mode velocity_bootstrap

Evidence directory:
``logs/flying_kick_sim2sim/20260522-175113/``.

The grounded report is
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_bootstrap_report.json``.
It falsifies initial floor penetration as the sufficient root cause:

- ``primary_reason: velocity_runtime_instability``;
- selected root z is ``0.783675`` and exactly matches ``HOME_KEYFRAME``;
- selected joints still exactly match Velocity ``default_joint_pos`` /
  action offset;
- ONNX/deploy observations still match at 98 dims;
- lowest foot collision surface improves from ``-0.018422`` to ``-0.000527``,
  passing the initial-contact clearance gate;
- the first unstable sample is line 33 at ``17:51:20.049`` with
  ``q_err_l2=6.767``, ``q_err_max=3.929``,
  ``gravity_b=(-0.365,-0.391,-0.845)``, and
  ``root_ang_vel_l2=7.765``;
- screenshot evidence is
  ``logs/flying_kick_sim2sim/20260522-175113/mujoco_velocity_home_root_run_fall.png``;
- after stop/restore, no ``g1_ctrl`` or ``unitree_mujoco`` process remained
  active, and the external sim config returned to ``enable_elastic_band: 0``.

The current blocker is therefore not ``FixStand``, not Mimic tracking, not
ONNX/deploy observation dimension mismatch, not policy-default joint mismatch,
and not initial foot-floor penetration. It is now the Velocity policy runtime
contract itself: likely stale Velocity policy/export, training robot mismatch,
gain/action scale mismatch, observation timing/value mismatch, or dynamics
mismatch against the new G1 mode-15 asset.

The no-elastic follow-up is
``logs/flying_kick_sim2sim/20260522-175822/`` with report
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_no_elastic_bootstrap_report.json``.
It keeps the grounded policy-default Velocity pose but sets
``MJLAB_ENABLE_ELASTIC_BAND=0``. This also fails in ``Velocity`` before any
Mimic trigger:

- selected sim config has ``enable_elastic_band: 0`` and ``start_paused: 1``;
- initial contact still passes with selected root z ``0.783675`` and lowest
  foot collision surface ``-0.000527``;
- ``primary_reason: velocity_runtime_instability``;
- the first unstable sample is line 30 at ``17:58:26.868`` with
  ``q_err_l2=5.438``, ``q_err_max=3.809``,
  ``gravity_b=(-0.579,-0.071,-0.812)``, and
  ``root_ang_vel_l2=4.790``;
- screenshot evidence is
  ``logs/flying_kick_sim2sim/20260522-175822/mujoco_velocity_home_root_no_elastic_run_fall.png``;
- after stop/restore, no ``g1_ctrl`` or ``unitree_mujoco`` process remained
  active, and the external sim config returned to ``enable_elastic_band: 0``.

This falsifies the elastic band as the sole cause. The sim2sim wrapper now
prints the actual ``elastic band=<0|1>`` value in its flow text so no-elastic
diagnostics are not mislabeled as elastic-band-enabled runs.

The extended report also checks policy provenance and current-source deltas:

- ONNX metadata reports ``run_path: 2026-03-18_18-40-20``;
- no matching local ``logs/rsl_rl`` source run is found
  (``source_run_found: false``);
- current source G1 init is ``KNEES_BENT_KEYFRAME`` and differs from the
  deploy ``v0`` default/action offset by L2 ``0.779069`` and max ``0.369``;
- selected no-elastic bootstrap qpos intentionally matches the deploy
  default/action offset instead of current source init, so this source-init
  mismatch is already controlled in the latest smoke;
- current-source action scale, stiffness, and damping are close to deploy
  values, with max gaps ``0.004501``, ``0.049377``, and ``0.042110``.

That makes a gross gain/action-scale mismatch less likely. The remaining
Velocity-only failure should now be treated as stale/missing-provenance
Velocity policy, observation/value timing mismatch in ``g1_ctrl``, or dynamics
mismatch against the current new-G1 mode-15 asset until proven otherwise.

The local Velocity policy inventory report is
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_inventory.json``.
It scanned 125 local ``g1_velocity*`` ONNX files under
``/home/ssy/ssy_files/mjlab/logs/rsl_rl`` and found 0 directly compatible
candidates for the active deploy ``v0`` policy:

- active deploy reference: ``obs[1,98] -> actions[1,29]`` with observations
  ``base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
  actions``;
- dominant local flat Velocity candidates: ``obs[1,99] -> actions[1,29]`` with
  ``base_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel,
  actions, command``;
- rough-terrain candidates include height scan and can reach ``obs[1,286]``;
- common incompatibility reasons are ``input_dim 99 != 98`` and
  ``observation_names differ``.

Therefore copying a newer local ONNX into ``velocity/v0`` is not a safe fix.
Either instrument the current ``g1_ctrl`` Velocity observations/actions, or
generate a complete deploy package from a known-compatible Velocity training
configuration.

The Velocity runtime trace patch is applied through:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_runtime_trace_patch.py --apply
   cmake --build /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/build -j8

The patch writes a sibling backup before editing the local runtime file:
``/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp.phase1_velocity_runtime_trace_v1.bak``.
It is intentionally a local deploy instrumentation step and is not committed
as mjlab source.

The runtime trace evidence run is
``logs/flying_kick_sim2sim/20260522-183512/`` with report
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_trace_report.json``
and screenshot ``mujoco_velocity_runtime_trace.png``. It starts in
``Velocity`` from policy-default joints, home root, no elastic band, and zero
command. The first unstable sample appears at line 29 / ``18:35:17.211``:

- ``policy_step=75``;
- ``command_norm=0.0``;
- ``raw_action_l2=14.76`` and ``raw_action_max=7.827``;
- ``processed_action_l2=4.874`` and ``processed_action_max=2.241``;
- ``joint_vel_l2=673.649`` and ``joint_vel_max=576.357``;
- ``gravity_b=(-0.052,-0.643,-0.764)`` and ``root_ang_vel_l2=8.491``.

This falsifies delayed or nonzero velocity command as the primary explanation
for the no-command Velocity fall. The remaining blocker is the active
Velocity policy / observation closed-loop contract: the policy is producing
large destabilizing actions under its zero-command runtime observation stream.

The zero-command ONNX replay report is
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_zero_command_replay.json``.
It builds the active deploy ``v0`` observation offline with zero base angular
velocity, projected gravity ``(0,0,-1)``, zero command, zero gait-phase output,
zero joint position/velocity error, and recurrent ``last_action``. The active
``policy.onnx`` still outputs nonzero targets:

- input/output contract: ``obs[1,98] -> actions[1,29]``;
- step 0: raw action L2 ``1.228195`` and processed target gap L2 ``0.507543``;
- replay maximum: processed target gap L2 ``0.660817`` and max joint gap
  ``0.357822``;
- top gaps are mainly ankle/arm joints, including indices ``10``, ``4``,
  ``16``, and ``23``.

This matches the early runtime trace scale before the fall and confirms the
active ``velocity/v0`` zero-command closed loop is not a default-pose hold.
The next remediation should replace or regenerate the complete Velocity deploy
package, then rerun this replay before another GUI smoke.

Velocity Deploy Candidate Triage
--------------------------------

Before replacing ``velocity/v0``, run the deploy-candidate triage:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy_candidate_triage.py \
     --limit 300 \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy_candidate_triage.json \
     --summary-only \
     --expect-no-direct-ready

Current report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy_candidate_triage.json``.

The result keeps direct ONNX swapping locked:

- scanned candidates: ``125``;
- ``active_v0_contract: 0``;
- ``direct_swap_ready: 0``;
- ``complete_unitree_deploy_package: 0``;
- current-source flat Velocity actors: ``122``;
- current-source actor re-export candidates with params/checkpoints: ``122``.

The reusable local candidates are not deploy-ready packages. They are
``obs[1,99] -> actions[1,29]`` actors with observations
``base_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel, actions,
command``. The active Unitree ``velocity/v0`` runtime is still the 98-dim
contract ``base_ang_vel, projected_gravity, command, phase, joint_pos,
joint_vel, actions``. Therefore each current-source candidate is blocked by:

- ``requires_99_dim_runtime_observation_support``;
- ``requires_unitree_deploy_yaml_generation``;
- ``missing_complete_unitree_deploy_package``.

The decision field is:

- ``safe_to_swap_local_onnx_into_active_v0: false``;
- ``remediation: generate_99_dim_deploy_package_or_retrain_98_dim_velocity``;
- ``real_robot_gate: locked``.

The next remediation must choose one path explicitly: either build a complete
99-dim Unitree deploy package and add correct runtime support for
``base_lin_vel``, or train/export a compatible 98-dim Velocity package. In both
cases, rerun zero-command ONNX replay before any GUI smoke, Mimic trigger, or
real-robot trial.

Runtime Observation Support Audit
---------------------------------

Do not generate a 99-dim deploy YAML until the C++ runtime can actually provide
the 99-dim observation contract. Audit it with:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_runtime_observation_audit.py \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_observation_audit.json \
     --expect-runtime-missing-base-lin-vel

Current report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_observation_audit.json``.

The contract split is now explicit:

- current worktree flat Velocity actor: 99 dims, with
  ``base_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel,
  actions, command``;
- external Unitree source flat Velocity actor: 98 dims, with
  ``base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
  actions``;
- active deploy ``velocity/v0`` YAML: 98 dims, with
  ``base_ang_vel, projected_gravity, velocity_commands, gait_phase,
  joint_pos_rel, joint_vel_rel, last_action``.

The active C++ deploy runtime cannot serve ``base_lin_vel`` today:

- registered observations do not include ``base_lin_vel``;
- ``ArticulationData`` has no root/base linear velocity field;
- ``BaseArticulation::update()`` does not populate linear velocity.

Therefore ``safe_to_generate_99_dim_deploy_yaml_only`` is ``false``. A 99-dim
deploy route requires a real runtime base-linear-velocity source first. The
alternative is a 98-dim Velocity training/export path matched to the active
deploy runtime contract. Real-robot and Mimic gates remain locked.

Deploy98 Velocity Task Contract
-------------------------------

The repo now exposes a separate 98-dim G1 Velocity task for the active Unitree
runtime contract:

``Mjlab-Velocity-Flat-Unitree-G1-Deploy98``

This does not modify the existing ``Mjlab-Velocity-Flat-Unitree-G1`` task. The
existing flat task remains the current-source 99-dim actor with
``base_lin_vel``. The deploy98 actor terms are:

``base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
actions``

Validate the task against the active deploy YAML with:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_task_contract.json \
     --expect-compatible

Current report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_task_contract.json``.

Current result:

- actor contract is ``98`` dims;
- active deploy YAML contract is ``98`` dims;
- semantic mapping is complete and ordered:
  ``command -> velocity_commands``, ``phase -> gait_phase``,
  ``joint_pos -> joint_pos_rel``, ``joint_vel -> joint_vel_rel``,
  ``actions -> last_action``;
- ``task_contract_matches_active_runtime: true``;
- ``safe_to_swap_without_training: false``;
- ``real_robot_gate: locked``.

This only unlocks a controlled 98-dim training/export route. It does not prove
that any existing policy is safe. The next required sequence is: train or locate
a checkpoint for the deploy98 task, export ONNX, generate matching deploy YAML,
run zero-command ONNX replay, then run Velocity-only Unitree MuJoCo smoke before
any Mimic trigger or real-robot trial.

Deploy98 Velocity Package Generator
-----------------------------------

Use the package generator after a deploy98 Velocity ONNX has been exported with
the expected metadata:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py \
     --policy-onnx /path/to/policy.onnx \
     --out-dir /path/to/unitree_policy_dir \
     --report-out logs/g1_tracking_phase1/<run>/velocity_deploy98_package.json \
     --expect-compatible

The generator enforces the active 98-dim runtime contract:

- ONNX shape ``obs[1,98] -> actions[1,29]``;
- actor metadata terms ``base_ang_vel, projected_gravity, command, phase,
  joint_pos, joint_vel, actions``;
- runtime YAML terms ``base_ang_vel, projected_gravity, velocity_commands,
  gait_phase, joint_pos_rel, joint_vel_rel, last_action``;
- 29-value metadata for joint names, default joint positions, stiffness,
  damping, and action scale.

It writes:

- ``exported/policy.onnx``;
- optional ``exported/policy.onnx.data`` when the ONNX external-data sidecar
  exists;
- ``params/deploy.yaml``.

Current generator smoke evidence:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_smoke.json``
and
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_write_smoke.json``.

The smoke used the stale active ``velocity/v0`` ONNX only to prove the generator
can parse a compatible deploy98 package and write the expected Unitree layout.
It does not re-accept that policy, because earlier zero-command replay already
showed the active ``v0`` policy does not hold the default pose. The write-smoke
report therefore keeps:

- ``safe_to_run_zero_command_replay: true``;
- ``safe_to_use_for_sim2sim: false``;
- ``safe_to_swap_without_zero_command_replay: false``;
- ``real_robot_gate: locked``.

The next policy package must still pass zero-command replay and Velocity-only
Unitree MuJoCo GUI smoke before any Mimic trigger or real-robot trial.

Actor ONNX Export Metadata
--------------------------

Post-hoc checkpoint export must preserve deploy metadata. Use:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/export_rsl_rl_actor_onnx.py \
     Mjlab-Velocity-Flat-Unitree-G1-Deploy98 \
     --checkpoint-file /path/to/model_N.pt \
     --output-file /path/to/policy.onnx \
     --device cpu

The export script now attaches base metadata after actor export:

- ``run_path``;
- ``joint_names``;
- ``joint_stiffness``;
- ``joint_damping``;
- ``default_joint_pos``;
- ``command_names``;
- ``observation_names``;
- ``action_scale``.

If the restored run directory name is not the provenance string you want in
the ONNX, pass ``--metadata-run-path`` explicitly. A deploy98 export is not
eligible for packaging unless the resulting metadata still reports the 98-dim
terms ``base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
actions``.

Deploy98 Training/Export Smoke
------------------------------

A minimal deploy98 training/export smoke was run to prove the non-model
plumbing:

.. code-block:: bash

   uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98 \
     --env.scene.num-envs 8 \
     --agent.max-iterations 1 \
     --agent.save-interval 1 \
     --agent.logger tensorboard \
     --agent.upload-model False \
     --agent.experiment-name g1_velocity_deploy98_smoke \
     --agent.run-name deploy98_smoke_20260522 \
     --gpu-ids None

Training output:
``logs/rsl_rl/g1_velocity_deploy98_smoke/2026-05-22_19-40-43_deploy98_smoke_20260522/``.

The smoke produced:

- ``model_0.pt``;
- ``2026-05-22_19-40-43_deploy98_smoke_20260522.onnx``;
- actor shape ``98`` and action shape ``29``.

The exported ONNX was packaged with:

.. code-block:: bash

   uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py \
     --policy-onnx logs/rsl_rl/g1_velocity_deploy98_smoke/2026-05-22_19-40-43_deploy98_smoke_20260522/2026-05-22_19-40-43_deploy98_smoke_20260522.onnx \
     --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package/policy_dir \
     --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package.json \
     --expect-compatible

Package result:

- ``compatible: true``;
- ``package_written: true``;
- ``safe_to_run_zero_command_replay: true``;
- ``safe_to_use_for_sim2sim: false``;
- ``real_robot_gate: locked``.

Zero-command replay report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_zero_command_replay.json``.

Current replay result:

- ``max_processed_target_gap_l2: 0.13366``;
- ``zero_command_target_is_default: false``.

This is expected for a 1-iteration smoke policy. It proves the training/export
and packaging chain, not policy quality. Do not run Unitree MuJoCo GUI smoke,
Mimic, or hardware with this smoke policy.

Deploy98 300-Iteration Pilot
----------------------------

A first real deploy98 Velocity pilot was run on GPU:

.. code-block:: bash

   uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98 \
     --env.scene.num-envs 4096 \
     --agent.max-iterations 300 \
     --agent.save-interval 50 \
     --agent.logger tensorboard \
     --agent.upload-model False \
     --agent.experiment-name g1_velocity_deploy98_candidates \
     --agent.run-name deploy98_v1_300iter_20260522 \
     --gpu-ids "[0]"

Training output:
``logs/rsl_rl/g1_velocity_deploy98_candidates/2026-05-22_19-45-22_deploy98_v1_300iter_20260522/``.

Produced files include:

- ``model_0.pt``;
- ``model_50.pt``;
- ``model_100.pt``;
- ``model_150.pt``;
- ``model_200.pt``;
- ``model_250.pt``;
- ``model_299.pt``;
- ``2026-05-22_19-45-22_deploy98_v1_300iter_20260522.onnx``.

Final TensorBoard scalars at step 299:

- ``Train/mean_reward: 2.785632``;
- ``Train/mean_episode_length: 134.479996``;
- ``Episode/length_seconds: 2.692022``;
- ``Episode_Termination/fell_over: 29.083334``;
- ``Metrics/twist/error_vel_xy: 0.239295``;
- ``Metrics/twist/error_vel_yaw: 0.306789``.

The final ONNX packages successfully:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_package.json``.

Package result:

- ``compatible: true``;
- ``package_written: true``;
- ``safe_to_run_zero_command_replay: true``;
- ``safe_to_use_for_sim2sim: false``;
- ``real_robot_gate: locked``.

Zero-command replay report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_zero_command_replay.json``.

Replay result:

- ``max_processed_target_gap_l2: 0.618108``;
- ``zero_command_target_is_default: false``.

This rejects the 300-iteration pilot before Unitree MuJoCo GUI smoke. The next
work should diagnose the deploy98 Velocity training objective/reset/reward
contract rather than only increasing iteration count.

Deploy98 Stand-First Velocity Task
----------------------------------

The next remediation is a separate task, not a mutation of the deploy98
baseline:

``Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst``

It keeps the same 98-dim actor contract used by the active Unitree runtime:

- ``base_ang_vel``;
- ``projected_gravity``;
- ``command``;
- ``phase``;
- ``joint_pos``;
- ``joint_vel``;
- ``actions``.

The task is intentionally standing-first:

- sampled velocity ranges are all zero;
- ``rel_standing_envs`` is ``1.0``;
- heading, forward-only, world-frame, and init-velocity command variants are
  disabled;
- ``push_robot`` is disabled;
- ``command_vel`` curriculum is disabled;
- training horizon is ``5.0`` seconds;
- ``alive`` and ``termination_penalty`` rewards are added.

This is a zero-command Velocity stability entry, not final nonzero command
tracking acceptance. The next policy candidate from this task must still pass:

1. package generation;
2. zero-command replay;
3. Velocity-only Unitree MuJoCo GUI smoke;
4. only then Mimic/action tracking evidence.

Real-robot work remains locked.

Deploy98 Stand-First 300-Iteration Candidate
--------------------------------------------

The first StandFirst GPU candidate is:

``logs/rsl_rl/g1_velocity_deploy98_standfirst_candidates/2026-05-22_20-06-38_standfirst_v1_300iter_20260522/``

Training command:

.. code-block:: bash

   uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst \
     --env.scene.num-envs 4096 \
     --agent.max-iterations 300 \
     --agent.save-interval 50 \
     --agent.logger tensorboard \
     --agent.upload-model False \
     --agent.experiment-name g1_velocity_deploy98_standfirst_candidates \
     --agent.run-name standfirst_v1_300iter_20260522 \
     --gpu-ids "[0]"

Final TensorBoard scalars at step 299:

- ``Episode/length_seconds: 5.0``;
- ``Episode_Termination/fell_over: 0.0``;
- ``Metrics/twist/error_vel_xy: 0.046835``;
- ``Metrics/twist/error_vel_yaw: 0.159242``;
- ``Episode_Reward/total: 30.056957``.

Package report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package.json``.

Package result:

- ``compatible: true``;
- ``package_written: true``;
- ``safe_to_run_zero_command_replay: true``;
- ``real_robot_gate: locked``.

Zero-command replay report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_zero_command_replay.json``.

Replay result:

- ``max_processed_target_gap_l2: 0.595646``;
- ``zero_command_target_is_default: false``.

This candidate passes the mjlab standing training gate and packaging gate, but it
is not a default-pose hold controller. Replay is therefore diagnostic evidence,
not a substitute for physics. The next allowed check is a sim-only Velocity
bootstrap in Unitree MuJoCo with this exact packaged policy. Do not judge this
candidate from ``FixStand`` and do not trigger Mimic or hardware before that
Velocity-only smoke passes.

Explicit Velocity Policy Root For Sim2Sim
-----------------------------------------

The single-action sim2sim wrappers accept ``MJLAB_VELOCITY_POLICY_ROOT`` for
``velocity_bootstrap``. This lets a GUI smoke use an explicit packaged policy
directory, for example:

.. code-block:: bash

   MJLAB_SIM2SIM_MODE=velocity_bootstrap \
   MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default \
   MJLAB_VELOCITY_BOOTSTRAP_ROOT=home \
   MJLAB_VELOCITY_POLICY_ROOT=logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package/policy_dir \
   bash scripts/tools/run_flying_kick_sim2sim.sh start

Without this override, the wrapper resolves the machine-local active
``config/policy/velocity`` tree, which can select the stale ``v0`` policy. This
override is sim-only; it does not accept hardware or Mimic gates by itself.

StandFirst Velocity-Only Unitree MuJoCo Smoke
---------------------------------------------

The first decisive StandFirst GUI smoke is:

``logs/flying_kick_sim2sim/20260522-202752/``

It used:

- ``MJLAB_SIM2SIM_MODE=velocity_bootstrap``;
- ``MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default``;
- ``MJLAB_VELOCITY_BOOTSTRAP_ROOT=home``;
- ``MJLAB_VELOCITY_POLICY_ROOT`` pointing to the StandFirst packaged policy;
- ``MJLAB_ENABLE_ELASTIC_BAND=0``;
- no Mimic trigger.

Screenshot evidence:
``logs/flying_kick_sim2sim/20260522-202752/mujoco_standfirst_velocity_run_fall.png``.

Velocity contract report:
``logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_standfirst_bootstrap_report.json``.

The report confirms:

- ``Velocity.policy_dir`` resolves to the StandFirst packaged policy directory;
- selected initial joints match deploy ``default_joint_pos`` and action offset
  with ``gap_l2=0.0`` and ``gap_max=0.0``;
- selected root z is ``0.783675``;
- initial foot clearance passes with ``min_foot_surface_z=0.027211``;
- ``enable_elastic_band=0``.

The run still fails in ``Velocity`` before any Mimic trigger. First unstable
sample:

- timestamp ``2026-05-22 20:27:56.728``;
- line ``29``;
- stable duration before failure ``1.5`` seconds;
- ``policy_step=75``;
- ``command_norm=0.0``;
- ``raw_action_l2=19.253``;
- ``processed_action_l2=7.182``;
- ``joint_vel_l2=328.134``;
- ``q_err_l2=4.855``;
- ``root_ang_vel_l2=10.997``.

This rejects the StandFirst candidate for sim2sim. The failure is not a
``FixStand`` fall, not stale ``velocity/v0``, and not an initial contact
mismatch. Real-robot work remains locked.

Deploy98 StandFirst Damped Continuation Candidate
-------------------------------------------------

``Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped`` was trained as a
separate zero-command deploy98 entry to reduce the previously observed
joint-velocity/action amplification loop. The first 300-iteration pilot was
rejected before packaging because it missed the strict training gate:
``Episode/length_seconds=4.888494`` and
``Episode_Termination/fell_over=2.166667`` at step 299.

The same run was then resumed from ``model_299.pt`` for 300 more iterations:

``logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522/``

Final TensorBoard scalars at step 598 pass the mjlab zero-command standing gate:

- ``Episode/length_seconds: 5.0``;
- ``Episode_Termination/fell_over: 0.0``;
- ``Episode_Reward/total: 38.259392``;
- ``Episode_Metrics/mean_action_acc: 0.065160``;
- ``Metrics/slip_velocity_mean: 0.003066``;
- ``Metrics/twist/error_vel_xy: 0.026822``;
- ``Metrics/twist/error_vel_yaw: 0.070388``.

The generated package report is:

``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package.json``

Package result:

- ``compatible: true``;
- ``package_written: true``;
- ONNX/deploy shape ``obs[1,98] -> actions[1,29]``;
- ``safe_to_run_zero_command_replay: true``;
- ``safe_to_use_for_sim2sim: false`` until replay and GUI smoke pass;
- ``real_robot_gate: locked``.

Zero-command replay report:

``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_zero_command_replay.json``

Replay result:

- ``max_processed_target_gap_l2: 0.385498``;
- ``zero_command_target_is_default: false``.

This improves over the rejected StandFirst package
(``max_processed_target_gap_l2: 0.595646``), but it is still not proof of
physics stability.

Sensitivity reports:

- zero-command-focused stress report:
  ``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity.json``;
- default-magnitude report:
  ``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity_default.json``.

The stress report keeps ``velocity_commands`` and ``gait_phase`` perturbations
at zero and finds ``joint_vel_rel`` remains the strongest large-perturbation
hazard: baseline processed target gap ``0.391274`` rises to ``3.296666`` under
a single-axis ``100 rad/s`` joint-velocity perturbation.

The default-magnitude report finds ``gait_phase`` and ``velocity_commands`` can
produce large actions if runtime command/phase masking is wrong. This is a
runtime contract hazard, not evidence that the zero-command deploy runtime must
provide nonzero phase: the Unitree deploy ``gait_phase`` implementation zeros the
phase output when ``command_norm < 0.1``.

The next allowed check is therefore a Velocity-only Unitree MuJoCo smoke using
this exact package through ``MJLAB_VELOCITY_POLICY_ROOT``. Do not judge this
candidate from ``FixStand`` alone, do not trigger Mimic, and do not touch
hardware before the Velocity-only smoke passes.

Damped Continuation Velocity-Only Smoke
---------------------------------------

The Velocity-only Unitree MuJoCo smoke for the damped continuation package is:

``logs/flying_kick_sim2sim/20260522-212636/``

It used:

- ``MJLAB_SIM2SIM_MODE=velocity_bootstrap``;
- ``MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default``;
- ``MJLAB_VELOCITY_BOOTSTRAP_ROOT=home``;
- ``MJLAB_VELOCITY_POLICY_ROOT`` pointing to
  ``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir``;
- ``MJLAB_ENABLE_ELASTIC_BAND=0``;
- no Mimic trigger.

Screenshot evidence:

``logs/flying_kick_sim2sim/20260522-212636/mujoco_damped_resume300_velocity_run_fall.png``

Velocity contract report:

``logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_bootstrap_report.json``

The report confirms:

- ``FSM: Start Velocity`` was reached;
- ``Velocity.policy_dir`` resolved to the damped continuation package;
- ONNX/deploy shape is ``obs[1,98] -> actions[1,29]``;
- selected initial joints match deploy ``default_joint_pos`` and action offset
  with ``gap_l2=0.0`` and ``gap_max=0.0``;
- selected root z is ``0.783675``;
- initial foot clearance passes with ``min_foot_surface_z=0.027211``;
- ``enable_elastic_band=0``.

The run still fails in ``Velocity`` before any Mimic trigger. First unstable
sample:

- timestamp ``2026-05-22 21:26:40.990``;
- line ``30``;
- stable duration before failure ``2.0`` seconds;
- ``policy_step=100``;
- ``command_norm=0.0``;
- ``raw_action_l2=38.483``;
- ``processed_action_l2=15.654``;
- ``joint_vel_l2=269.793``;
- ``q_err_l2=13.591``;
- ``root_ang_vel_l2=19.108``.

This rejects the damped continuation package for sim2sim. The failure is a
``Velocity`` closed-loop/runtime mismatch, not a ``FixStand`` failure, not stale
``velocity/v0``, not a deploy default/action-offset mismatch, and not initial
foot-floor contact. Real-robot and Mimic work remain locked.
