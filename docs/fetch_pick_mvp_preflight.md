# FetchPick Phase 1: hardware preflight blocked

Audit date: 2026-10-07. Decision: **NO-GO for FetchPick training**.
This branch contains a reproducible preflight audit, CPU regression tests and
this report. It does **not** contain a FetchPick environment or trained policy.
The user explicitly required stopping if implementation needed an invented
hardware mechanism or a broken 61D/14D policy contract.

## Sources and scope

- RL develop: `273afe0b31c4ab365b9ff806a927b63ac92b5ddd`.
- Runtime main reviewed: `77005fdeb330de64247fa4cc940ba90e2be3ea85`.
- [RL robot model](https://github.com/thankfull120/microduck_rl/blob/273afe0b31c4ab365b9ff806a927b63ac92b5ddd/src/mjlab_microduck/robot/microduck/robot_groundcontact.xml).
- [Runtime joint mapping](https://github.com/pollen-robotics/microduck/blob/77005fdeb330de64247fa4cc940ba90e2be3ea85/duck-control/src/model.rs)
  and [hardware IDs](https://github.com/pollen-robotics/microduck/blob/77005fdeb330de64247fa4cc940ba90e2be3ea85/duck-ipc-proto/src/lib.rs).
- [Maintainer confirmation that the mouth DoF is excluded from the policy and independently controlled](https://github.com/pollen-robotics/microduck_rl/issues/27#issuecomment-5514573901).

No existing task, XML, runtime, policy or checkpoint was modified. No training,
hardware deployment, ONNX conversion or Hugging Face publication occurred.

## 1. Actual grasp hardware and the missing simulation mechanism

Runtime code describes **15 hardware joints**. Fourteen are policy joints;
`mouth` is separately controlled at hardware array index 9, Dynamixel ID **34**.
`MOUTH_CLOSED` and `MOUTH_OPEN` are -5 and +30 degrees in runtime coordinates.
These values alone do not specify the physical jaw rotation axis or linkage.
The user's particular assembled hardware has not been physically inspected.

The current RL groundcontact XML has **14 actuators and no mouth joint or mouth
actuator**. Its `jaw`, `jaw_soft`, `soft_mouth_top` and `bottom_head_shell` meshes
are attached to the same `jaw_soft` rigid body. `head_roll` moves that body; it is
not an opening/closing jaw axis. The upper soft-mouth mesh is visual geometry,
not a calibrated opposing gripping surface. A mouth-tip site is only a marker.

CPU MuJoCo compilation confirms this ownership. Moving head/neck joints leaves
the separation between the two selected mesh origins invariant (variation
1.39e-17 m). This measures rigidity, **not** the physical beak aperture.
Therefore a jaw actuator, contact attachment, weld, teleport or distance latch
would not establish a physically valid grasp on this model.

Unblocking requires verified moving-link membership, jaw pivot and axis in the
MJCF frame, zero/sign/range calibration, servo transmission/linkage, mass/inertia
and opposing collision surfaces for the actual hardware revision. The public
CAD source pointer and runtime mouth angles are insufficient to infer these.

## 2. model_61447.pt warm-start decision

**Tensor structure compatible; direct FetchPick warm start NOT APPROVED.**

Checkpoint SHA-256:
`387458346862f7244659df4f264127fcbf1fbd5bd028474db3d960090eff2aa4`.
It was opened locally with `torch.load(..., weights_only=True, map_location="cpu")`.
It was not uploaded or overwritten.

| Check | Observed result | Interpretation |
|---|---|---|
| Actor | 61 -> 512 -> 256 -> 128 -> 14 | Required tensor dimensions match |
| Critic | 76 -> 512 -> 256 -> 128 -> 1 | A future object-aware critic may differ |
| Strict rsl_rl MLP load | Actor and critic: all keys matched | Existing architecture loads on CPU |
| Synthetic zero-input inference | Actor (2,14), critic (2,1), finite | No rollout or walking-quality claim |
| Actor normalizer | 61 entries; finite, positive std | Present, but task semantics must match |
| Joint ordering | Stock XML and supplied training configuration are consistent | Checkpoint itself has no joint-name provenance |
| Metadata | iter=61447; infos only env_state.common_step_counter=1486848 | No embedded task ID, source SHA or ordered joint names |
| Optimizer | saved LR 1e-5 | Blind reuse also carries training state |

Policy order is left hip_yaw, hip_roll, hip_pitch, knee, ankle; neck_pitch,
head_pitch, head_yaw, head_roll; right hip_yaw, hip_roll, hip_pitch, knee, ankle.
The separately controlled mouth is excluded. The supplied saved env YAML has
actor term order `base_ang_vel, projected_gravity, joint_pos, joint_vel, actions,
command, head_command, body_command`: 48 proprioceptive + 3 twist + 4 head + 6 body.
The saved action selector is regex-based; the checkpoint alone cannot prove
its historical XML order. A filename is not task or model provenance.

Saved twist std is approximately `[0.19549, 0.12343, 0.49861]`. If GroundPick's
phase command `[1,0,0]` replaces velocity, its first slot is about **4.92** after
this saved normalizer. Phase and velocity are different meanings despite the
same 61D shape. `MICRODUCK_WARM_START=1` resets counters, not that semantic
mismatch, normalizer or optimizer. Future options are to preserve velocity
semantics and reuse this as an unchanged walking expert, or explicitly evaluate
selective weight transfer with a target-task normalizer/critic. Neither option
has been validated for FetchPick here.

## 3. Files added

- `scripts/fetch_pick_preflight.py`: read-only stock MJCF / optional checkpoint /
  optional CPU MuJoCo audit. Exit 2 intentionally means hardware gate blocked.
- `tests/test_fetch_pick_preflight.py`: seven CPU regression tests.
- `docs/fetch_pick_mvp_preflight.md`: this report and implementation prerequisites.

No `microduck_fetch_env_cfg.py`, FetchPick MDP functions or task registration
were added: an apparently runnable grasp environment would hide the missing
physical mechanism. The preflight is a snapshot diagnostic, not a general
hardware certification tool; adding a joint named mouth never grants GO.

## 4. Tests performed

| Test | Result |
|---|---|
| Stdlib CPU test run, MuJoCo absent from default Python path | 6 passed, 1 explicitly skipped |
| CPU run with MuJoCo 3.10.0 available | **7/7 passed** |
| Actual stock MJCF compilation and kinematic sanity | 14 actuators; correct policy order; fixed jaw confirmed |
| XML before/after CLI checksum | Unchanged |
| Checkpoint finite values, dimensions and normalizer | Passed structure checks; warm-start approval withheld |
| rsl_rl 5.0.1 strict actor/critic load and synthetic inference | Passed; not environment resume |
| Available torch runtime | 2.9.1+cpu; CUDA unavailable |

Reproduce from a checkout root (unit tests use stdlib unittest):

```bash
python -m unittest discover -s tests -p test_fetch_pick_preflight.py -v
# With the repository dependencies installed, includes CPU MuJoCo test:
uv run python -m unittest discover -s tests -p test_fetch_pick_preflight.py -v
uv run python scripts/fetch_pick_preflight.py --physics --checkpoint /path/to/model_61447.pt
# Expected audit exit status: 2 (NO-GO), not a successful training gate.
```

Local checks used existing CPU torch/MuJoCo packages and the rsl_rl 5.0.1 source.
A fresh full `uv sync` and full mjlab environment build were not validated.

## 5. Tests not run

- FetchPick cfg/MDP tests: **NOT RUN**; task intentionally not implemented.
- Dynamic contact, real grasp, gravity lift and timed hold: **NOT RUN**.
- Noisy-init 3-second stable-pose test under the FetchPick robot/object scene:
  **NOT RUN**; no verified scene exists.
- 64 environments x 5 PPO iterations: **NOT RUN**; no CUDA training runtime and
  no valid FetchPick task. Runtime obs/action, reward, termination and rollout
  NaN checks consequently remain unverified.
- FetchPick runner checkpoint resume: **NOT RUN**; strict tensor loading is not
  a substitute for task/critic/normalizer compatibility.
- Existing locomotion and trick rollout regression: **NOT RUN**; existing source
  and policies are untouched, but no new policy's walking ability is established.
- Official ONNX export: **NOT RUN** because the smoke gate has not passed.
- Real robot and Hugging Face publication: **NOT RUN**, outside scope.

Commands prepared for AFTER hardware validation, task implementation and CPU
physics acceptance; the task ID below is a proposal and currently unregistered:

```bash
uv run train Mjlab-FetchPick-Flat-MicroDuck --env.scene.num-envs 64 --agent.max_iterations 5
# Only after a passing smoke run, with its actual checkpoint path:
uv run scripts/export.py Mjlab-FetchPick-Flat-MicroDuck --checkpoint-file /path/to/smoke/model_N.pt --onnx-file /tmp/fetchpick-smoke.onnx --num-envs 1
```

Do not use the old checkpoint as a resume argument in this command. No manual
checkpoint-to-ONNX conversion is proposed. Official export must bake the actual
new policy normalizer and be checked against its 61D input and 14D output.

## 6. What is currently possible

Existing GroundPick supplies reaching/posture structure, not object grasping.
Velocity supplies the walking base and its noise/DR/BAM/NaN protections;
BallKick supplies a simple object-scene pattern. StandUp/VelStand supply recovery
and warm-start curriculum patterns, not a grasp mechanism. Those tasks do not
supply verified approach-contact-grasp-lift-hold success.

The current branch can reproduce the blocking hardware and checkpoint evidence.
It cannot train or demonstrate a physical FetchPick policy. All five requested
success stages remain **unvalidated**, with no success percentage asserted.

## 7. Implementation design after the hardware blocker is resolved

Keep the existing walking policy and tasks intact. Start a dedicated
`microduck_fetch_env_cfg.py` from the Velocity base and GroundPick groundcontact
posture setup; borrow only object/reset patterns from BallKick. Add all custom
MDP functions to `tasks/mdp.py`, a distinct runner experiment and registration to
`tasks/__init__.py`. A verified articulated-mouth robot/scene must be FetchPick
specific. Do not change shared selectors or robot constants for existing tasks.

A physical mouth actuator can be simulated as an **independently scripted**
controller, matching the runtime's separate mouth control, while the policy
still outputs 14 body/head actions. This requires explicit policy joint/action
selectors: current broad non-passive selectors would accidentally include the
new mouth DoF and silently break 61D/14D. Do not disguise a powered mouth as a
passive joint. This controller design needs calibrated hardware parameters; it
is not implemented or approved for deployment here. No robotd change is proposed
in this phase, and future mouth/policy timing must be tested before real use.

Start on flat ground with one lightweight object at a fixed reachable location.
Object state may be used in reward/termination and privileged critic inputs, not
silently appended to the actor. Retaining true velocity slots permits use of an
unchanged walking expert; GroundPick-style phase slots preserve shape but change
semantics and require a separately validated policy/normalizer. A fixed-location
MVP cannot establish general object localization or robust closed-loop grasping.

Use separately logged stages; numerical thresholds must come from calibrated
geometry and physics, not arbitrary presumed aperture or standing height:

| Stage | Required evidence | Explicit rejection |
|---|---|---|
| Approach | Mouth-to-object proximity and suitable alignment, stable support | Body reaching the object alone |
| Contact | Contact pair between object and allowed jaw gripping geometry | Trunk/leg/head-shell-only contact |
| Grasp/hold | Physically closed jaw with calibrated opposing contact/force and stable object-relative pose over time | Proximity latch, weld, teleport, pushing |
| Lift | Object lowest collision point clears floor while grasp remains valid; no floor or other robot support | COM increase during roll, bounce, dragging, body support |
| Timed hold | Continuous grasp + clearance + bounded relative motion for duration, timer reset on loss | Accumulated disconnected contacts or ballistic toss |

MDP implementation needs approach potential shaping, contact classification,
per-env grasp/hold state and reset, lift/hold gated rewards, drop/fall/nonfinite
termination and timeout. State updates belong in an always-executed hook, not a
zero-weight reward (mjlab skips those). Negative tests must include push-only,
body carry, drag, bounce/toss, reset leakage and loss of contact on the last step.
Stable target pose and gripping physics must pass CPU checks before GPU training.

## 8. Risks and GO / NO-GO

**NO-GO for long training.** The missing articulated model prevents validating
force closure and actual lift. Broad selectors threaten the actor/action
contract if a fifteenth joint is naively added. Shape-compatible transfer can
still destroy behavior through command/normalizer changes; fine-tuning does not
prove walking preservation. Privileged object state also changes critic shape
and must not leak into an unobservable runtime actor input.

GPU smoke, reward/termination regressions, checkpoint runner loading and official
ONNX export remain outstanding. No grant of GO follows from passing this audit's
negative regression tests. An unchanged old walking policy remains available,
but new FetchPick and policy-transition quality require explicit evaluation.

## 9. GitHub delivery

Work branch: `feature/fetch-pick-mvp`, based on the develop commit above.
Only the three added audit/test/report files belong in its commit. The feature
branch is an incomplete Phase 1 investigation; do not merge it as a working
FetchPick implementation. Commit and any draft PR links are reported separately
once created. Develop is not a commit target.

## Required input to resume

Provide the manufacturer/validated MJCF or URDF with the moving mouth, or
kinematic drawings and calibration data for the actual jaw mechanism. The
specific request to avoid invented hardware is the reason implementation pauses
here; permission to invent an unverified mechanism would not establish physical
validity.
