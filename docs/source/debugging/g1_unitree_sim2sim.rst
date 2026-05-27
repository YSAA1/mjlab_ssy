G1 Unitree Sim2sim Lanes
========================

This page describes the repo-owned G1 Unitree sim2sim preparation path. It
replaces ad-hoc ``.external`` patching with a generated output lane and a
machine-readable manifest.

The feature prepares an ``official_source_plus_automation_deviation`` lane. It
does not certify a ``clean_official`` Unitree baseline, real robot readiness, or
policy quality.

Command
-------

Prepare a lane from a clean official Unitree checkout and a selected action
bundle:

.. code-block:: bash

   uv run unitree-sim2sim prepare-g1 \
     --official-root <clean-unitree-root> \
     --out-root <scratch-lane> \
     --action flying_kick \
     --policy-root <policy-root>

Supported actions:

- ``flying_kick``
- ``roundhouse_leading_right``

The ``--policy-root`` must contain:

.. code-block:: text

   exported/policy.onnx
   params/deploy.yaml
   params/<action-motion>.npz

Optional deviations:

.. code-block:: bash

   --automation-sequence full
   --automation-sequence fixstand_only
   --use-mjlab-mode15-model --mjlab-model-xml <mjlab-g1.xml>
   --align-mjlab-actuator-limits
   --apply-official-joint-passive-defaults
   --diagnostic-trace
   --evidence-dir <runtime-evidence-dir>
   --force

``--force`` is required before replacing an existing output root. The command
refuses to generate inside the official source checkout.

Compatibility Wrappers
----------------------

The old sim2sim wrapper names remain available as compatibility entry points.
Use ``prepare-lane`` to reach the new productized generator:

.. code-block:: bash

   bash scripts/tools/run_flying_kick_sim2sim.sh prepare-lane \
     --official-root <clean-unitree-root> \
     --out-root <scratch-lane> \
     --policy-root <flying-kick-policy-root>

   bash scripts/tools/run_roundhouse_leading_right_sim2sim.sh prepare-lane \
     --official-root <clean-unitree-root> \
     --out-root <scratch-lane> \
     --policy-root <roundhouse-policy-root>

   bash scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh prepare-lane \
     --action flying_kick \
     --official-root <clean-unitree-root> \
     --out-root <scratch-lane> \
     --policy-root <flying-kick-policy-root>

The legacy ``start`` commands still operate on local runtime state and may
mutate ``.external/unitree_rl_mjlab`` during simulator/controller launch. Use
the generated lane and manifest for reproducible preparation evidence.

Manifest
--------

Each generated lane writes:

.. code-block:: text

   <scratch-lane>/UNITREE_SIM2SIM_MANIFEST.json

The manifest records:

- source checkout path and source identity;
- output root and manifest path;
- selected action, state name, policy subdir, and trigger;
- automation sequence and unpause delay;
- deviation labels;
- changed paths;
- policy asset hashes;
- optional model asset hashes;
- optional evidence directory;
- forbidden claims.

Every path in ``changed_paths`` is relative to the generated output root. The
manifest is part of the changed path list.

Evidence Labels
---------------

Use these labels consistently:

``clean_official``
  Unchanged official Unitree source and runtime behavior. Do not use this label
  unless a capable host has run the clean official path with the required
  input/display/runtime capability.

``official_source_plus_deviation``
  Official source copied into a fresh lane, then changed by declared and
  reviewed deviations.

``official_source_plus_automation_deviation``
  The current productized path. It disables physical joystick input, injects
  synthetic automation, copies selected policy assets, and may apply declared
  model-alignment deviations.

``diagnostic_trace``
  Optional instrumentation for debugging. It is disabled by default and must
  not be treated as core product behavior.

Runtime Prerequisites
---------------------

The productized tests do not require MuJoCo runtime, DDS, Unitree SDK, joystick
devices, display access, or robot hardware. They verify generation, manifests,
labels, path handling, and wrapper delegation.

Visual/controller smoke is a separate runtime gate. Run it only on a host with:

- a buildable Unitree simulator and controller;
- MuJoCo display or capture path;
- DDS loopback or the required network interface;
- joystick/input automation or an approved no-joystick automation path.

If runtime capability is missing, record the blocker as a runtime limitation.
Do not downgrade or relabel it as a productization failure when static and
golden tests pass.

Local Runtime State
-------------------

``.external/unitree_rl_mjlab`` is local runtime storage. Do not commit it, use
it as canonical source, or copy it wholesale into ``mjlab``. Generated lanes
must come from a caller-supplied official checkout plus declared deviations.

Historical Evidence
-------------------

The prior research lane is archived here:

.. code-block:: text

   docs/research/archive/g1-sim2sim-2026-05-25.md

That archive is historical evidence for the original
``official_source_plus_automation_deviation`` experiments. It is not the
current runbook for preparing new lanes.

The old agent utility
``scripts/agent/prepare_official_deviation_lane.py`` is a predecessor of the
productized generator. New work should use ``uv run unitree-sim2sim
prepare-g1`` instead.
