"""Read-only FetchPick hardware/checkpoint audit; this does not train a policy.

Run from any directory. Exit 2 means the FetchPick hardware gate is blocked,
not that the audit crashed. No simulator joint, attachment or policy is created.
The optional CPU physics check imports mujoco; the checkpoint check imports torch.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

POLICY_JOINTS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = REPO_ROOT / "src/mjlab_microduck/robot/microduck/robot_groundcontact.xml"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_model(path: Path) -> dict:
    """Inspect an expanded MJCF; includes need expansion before this audit."""
    root = ET.parse(path).getroot()
    if root.findall(".//include"):
        raise ValueError("Pass an expanded MJCF so no included mouth mechanism is missed")
    actuator = root.find("actuator")
    actuators = list(actuator) if actuator is not None else []
    driven = [a.get("joint") for a in actuators]
    mesh_bodies: dict[str, list[dict]] = {}
    joints = []
    for body in root.findall(".//worldbody//body"):
        joints.extend(j.get("name") for j in body.findall("joint"))
        for geom in body.findall("geom"):
            mesh = geom.get("mesh", "")
            if mesh in {"jaw", "jaw_soft", "soft_mouth_top", "bottom_head_shell"}:
                mesh_bodies.setdefault(mesh, []).append({
                    "body": body.get("name"), "geom_class": geom.get("class"),
                })
    body_sets = {k: {g["body"] for g in v} for k, v in mesh_bodies.items()}
    lower = body_sets.get("jaw", set())
    upper = body_sets.get("soft_mouth_top", set())
    rigid_together = bool(lower and upper and len(lower | upper) == 1)
    blockers = []
    if "mouth" not in joints or "mouth" not in driven:
        blockers.append("No independently actuated mouth joint in the expanded MJCF")
    if rigid_together:
        blockers.append("Jaw and upper mouth meshes belong to the same rigid body")
    if tuple(driven) != POLICY_JOINTS:
        blockers.append("Model actuator order differs from the stock 14-policy-joint contract")
    blockers.append("Verified jaw kinematics/linkage and contact calibration are required before a grasp task")
    return {
        "model": path.name, "sha256": sha256(path), "actuated_joints": driven,
        "policy_joint_order_matches": tuple(driven) == POLICY_JOINTS,
        "mouth_joint_present": "mouth" in joints,
        "mouth_actuator_present": "mouth" in driven,
        "mouth_meshes": mesh_bodies, "jaw_and_upper_mouth_rigid_together": rigid_together,
        "decision": "NO-GO", "blockers": blockers,
    }


def inspect_checkpoint(path: Path) -> dict:
    """Weights-only inspection. Shape compatibility never grants warm-start permission."""
    import torch

    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    actor = checkpoint["actor_state_dict"]
    critic = checkpoint["critic_state_dict"]
    expected = {
        "mlp.0.weight": (512, 61), "mlp.0.bias": (512,),
        "mlp.2.weight": (256, 512), "mlp.2.bias": (256,),
        "mlp.4.weight": (128, 256), "mlp.4.bias": (128,),
        "mlp.6.weight": (14, 128), "mlp.6.bias": (14,),
        "obs_normalizer._mean": (1, 61), "obs_normalizer._var": (1, 61),
        "obs_normalizer._std": (1, 61), "obs_normalizer.count": (),
        "distribution.std_param": (14,),
    }
    mismatch = {name: list(actor[name].shape) if name in actor else None
                for name, shape in expected.items()
                if name not in actor or tuple(actor[name].shape) != shape}
    finite = all(bool(torch.isfinite(v).all()) for group in (actor, critic)
                 for v in group.values() if isinstance(v, torch.Tensor))
    std = actor["obs_normalizer._std"]
    mean = actor["obs_normalizer._mean"]
    normalizer_ok = tuple(std.shape) == (1, 61) and bool((std > 0).all()) and finite
    normalized_phase0 = None
    if normalizer_ok:
        normalized_phase0 = ((torch.tensor([1., 0., 0.]) - mean[0, 48:51]) / std[0, 48:51]).tolist()
    return {
        "file": path.name, "sha256": sha256(path), "keys": sorted(checkpoint),
        "iteration": checkpoint.get("iter"), "infos": checkpoint.get("infos"),
        "actor_shape_mismatches": mismatch,
        "actor_unexpected_keys": sorted(set(actor) - set(expected)),
        "weights_finite": finite, "actor_normalizer_finite_positive": normalizer_ok,
        "critic_input_dim": int(critic["mlp.0.weight"].shape[1]),
        "twist_mean": mean[0, 48:51].tolist(), "twist_std": std[0, 48:51].tolist(),
        "phase0_through_velocity_normalizer": normalized_phase0,
        "optimizer_learning_rates": [g["lr"] for g in checkpoint["optimizer_state_dict"]["param_groups"]],
        "cuda_available": torch.cuda.is_available(),
        "direct_fetchpick_warm_start": "NOT APPROVED",
        "reason": "61D/14D shapes do not establish phase-command semantics, joint-order provenance, or target-critic compatibility",
    }


def physics_check(path: Path) -> dict:
    """Compile the actual MJCF and check mesh rigidity over legal head poses.

    This is a kinematic sanity check, NOT a successful contact/lift rollout.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    data = mujoco.MjData(model)
    order = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT,
                              int(model.actuator_trnid[i, 0])) for i in range(model.nu)]
    def geom_for(mesh_name: str) -> int:
        mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
        if mesh_id < 0:
            raise ValueError(f"Missing mesh {mesh_name}")
        return next(i for i in range(model.ngeom)
                    if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                    and model.geom_dataid[i] == mesh_id)
    lower, upper = geom_for("jaw"), geom_for("soft_mouth_top")
    distances = []
    for angle in (-0.2, 0., 0.2):
        mujoco.mj_resetData(model, data)
        for name in ("neck_pitch", "head_pitch", "head_roll"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = angle
        mujoco.mj_forward(model, data)
        distances.append(math.dist(data.geom_xpos[lower], data.geom_xpos[upper]))
    return {
        "mujoco_version": mujoco.__version__, "compiled": True,
        "actuator_count": model.nu, "policy_joint_order_matches": tuple(order) == POLICY_JOINTS,
        "jaw_upper_same_body": bool(model.geom_bodyid[lower] == model.geom_bodyid[upper]),
        "mesh_origin_distance_variation_m": max(distances) - min(distances),
        "measured_quantity": "rigid mesh-origin separation, not a calibrated beak aperture",
        "grasp_lift_rollout": "NOT RUN: no verified articulated mouth model",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--physics", action="store_true")
    args = parser.parse_args()
    result = {"hardware": inspect_model(args.model)}
    if args.checkpoint:
        result["checkpoint"] = inspect_checkpoint(args.checkpoint)
    if args.physics:
        result["physics"] = physics_check(args.model)
    result["readiness"] = "NO-GO: preflight evidence only; no FetchPick task registered"
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
