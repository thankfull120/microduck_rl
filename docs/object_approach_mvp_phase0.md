# MicroDuck Object-Approach MVP — Phase 0 / Architecture Baseline

Status: PLANNING ONLY
Branch: `feature/object-approach-mvp`
Base: `develop`

## 1. Goal

Add a new high-level capability that is not currently present as a completed official MicroDuck behavior:

`detect a specified object -> estimate target direction/distance -> approach it autonomously -> stop at pickup distance -> optionally hand off to an existing official skill such as GroundPick`.

This project must **reuse existing official skills instead of retraining them**.

## 2. What is already official and must not be rebuilt

From the official MicroDuck repositories:

- walking / VelStand policies already exist;
- GroundPick already exists;
- StandUp / SitStand / BallKick / Roulade / roller skills already exist;
- camera capture exists;
- ToF/depth exists;
- odometry and gaze infrastructure exist;
- NPU/CPU detector infrastructure exists;
- `mediad` can publish detection boxes;
- the current official detector is a **single-class MicroDuck detector**, not a generic arbitrary-object detector.

Therefore the new value is not another walking policy or another GroundPick policy.

## 3. Confirmed perception baseline

Official runtime:
- camera frames are available from `mediad`;
- `duck-detect` supports RKNN NPU inference and ONNX CPU fallback;
- the official detector is YOLO11n, 320x320, single class `duck`;
- detection is integrated in `mediad` and emitted as `media.detections`;
- the detector model can be updated independently from daemon releases;
- current official NPU pipeline is suitable as a reference architecture for a custom single-class target detector.

Official detector training pipeline:
- capture;
- triage;
- autolabel;
- review;
- YOLO dataset build;
- YOLO11n fine-tune;
- ONNX export;
- RKNN INT8 conversion;
- live/offline evaluation.

## 4. Important conclusion

There is no reason to train a new locomotion policy just to approach a target.

The lowest-risk architecture is:

`target detector -> target tracker -> bearing/distance controller -> existing walking policy -> stop condition -> optional GroundPick trigger`

Reinforcement learning should only be introduced later if a conventional controller proves insufficient.

## 5. MVP contract

The first MVP is intentionally narrow:

- one target class only;
- one target visible at a time;
- flat indoor floor;
- no object grasp re-training;
- no generic open-vocabulary detection;
- no multi-object semantic planning;
- no long-horizon exploration;
- existing 61D -> 14D locomotion policy unchanged;
- existing mouth controller unchanged;
- existing GroundPick unchanged.

### Success definition

The robot starts with the target visible in the head camera and:

1. detects the target;
2. turns to center it;
3. walks toward it;
4. slows down near the target;
5. stops inside a defined approach distance;
6. remains stable for a short hold period.

GroundPick handoff is a later sub-phase after approach-only success is reliable.

## 6. Required components

### A. Target detector
Do not overwrite the official `duck` detector.

Create a separate target-detector model and dataset identity.

Minimum metadata:
- target class name;
- dataset revision;
- train/validation sessions;
- model checkpoint;
- ONNX SHA256;
- RKNN SHA256;
- precision / recall / mAP;
- target device/runtime version.

### B. Target tracker
Input:
- detection boxes;
- confidence;
- frame timestamp.

Output:
- selected target box;
- image-center error;
- lost-target state.

### C. Bearing estimate
For the first MVP, image-plane horizontal error may be enough for turning.

Do not claim metric bearing calibration until camera intrinsics are verified.

### D. Distance estimate
Priority order:
1. ToF/depth fusion if target geometry can be associated reliably;
2. validated monocular size estimate for a fixed-size target;
3. conservative stop rule based on bounding-box scale.

Do not invent metric distance from a bounding box without calibration.

### E. Approach controller
The controller should command the existing locomotion interface, not raw servos.

Suggested state machine:

`SEARCH -> ALIGN -> APPROACH -> SLOW -> STOP -> HOLD`

Emergency exits:
- target lost;
- obstacle too close;
- fall state;
- detector stale;
- command timeout.

## 7. Why Phase 1 should not be RL yet

A deterministic controller is preferred first because:
- existing walking policy already accepts velocity commands;
- detector output naturally maps to heading and forward-speed commands;
- controller behavior is easier to validate and debug;
- it preserves `model_61447.pt` and official locomotion;
- saleable value can still exist in the detector dataset, tuned detector, controller, and validation package.

Only if oscillation, occlusion recovery, complex obstacle negotiation, or multi-stage behavior cannot be solved robustly should RL be evaluated.

## 8. Repo impact

This `microduck_rl` branch is documentation/planning only for now.

Likely implementation ownership:

### Official-runtime-derived fork/worktree
Future files will likely live around:
- `mediad` detector integration;
- detection message/state;
- high-level autonomous behavior/controller;
- camera/ToF fusion.

### RL repo
Only add a new RL task if later evidence shows RL is actually needed.

Do **not** modify:
- current 61D/14D locomotion contract;
- `model_61447.pt`;
- official GroundPick;
- official walking tasks;
- FetchPick branch/PR.

## 9. Next gating decision

Before coding the target detector, choose the first physical target object.

The first target should be:
- visually distinctive;
- rigid and consistent in size/shape;
- safe for the robot;
- easy to photograph from many angles;
- large enough to detect from room scale;
- usable later with GroundPick if desired.

Once the target class is chosen, the next concrete work item is:
`dataset specification + capture plan + detector fork/adaptation plan`.

## 10. Asset / sales archive rule

Every new object-approach asset must be stored separately from official Pollen assets:

- dataset;
- labels;
- training config;
- detector weights;
- ONNX;
- RKNN;
- controller version;
- evaluation logs;
- videos;
- license/source record.

This capability is classified as:
`NEW HIGH-LEVEL AUTONOMOUS BEHAVIOR`,
not as a new walking or GroundPick skill.

FINAL PHASE-0 DECISION: GO FOR TARGET-DETECTOR + CONTROLLER MVP; DO NOT START NEW LOCOMOTION RL.
