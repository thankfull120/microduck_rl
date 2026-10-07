"""CPU regression tests for the real stock-model hardware gate.

Uses unittest so the XML gate is runnable without the GPU/mjlab stack.
The physics test is explicitly skipped if MuJoCo is unavailable.
"""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fetch_pick_preflight", ROOT / "scripts/fetch_pick_preflight.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class FetchPickHardwareGate(unittest.TestCase):
    def test_stock_actuator_order_is_the_policy_contract(self):
        result = audit.inspect_model(audit.DEFAULT_MODEL)
        self.assertTrue(result["policy_joint_order_matches"])
        self.assertEqual(len(result["actuated_joints"]), 14)

    def test_stock_head_meshes_are_not_a_moving_gripper(self):
        result = audit.inspect_model(audit.DEFAULT_MODEL)
        self.assertTrue(result["jaw_and_upper_mouth_rigid_together"])
        self.assertFalse(result["mouth_joint_present"])
        self.assertFalse(result["mouth_actuator_present"])
        self.assertEqual(result["decision"], "NO-GO")

    def test_surface_marker_is_not_an_actuator(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "marker.xml"
            path.write_text('<mujoco><worldbody><body name="head"><site name="mouth_tip"/></body></worldbody></mujoco>')
            result = audit.inspect_model(path)
            self.assertFalse(result["mouth_actuator_present"])
            self.assertEqual(result["decision"], "NO-GO")

    def test_xml_with_unresolved_include_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "included.xml"
            path.write_text('<mujoco><include file="unseen_jaw.xml"/></mujoco>')
            with self.assertRaisesRegex(ValueError, "expanded MJCF"):
                audit.inspect_model(path)

    def test_adding_a_name_does_not_certify_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "name_only.xml"
            path.write_text('<mujoco><worldbody><body name="jaw"><joint name="mouth"/></body></worldbody><actuator><position joint="mouth"/></actuator></mujoco>')
            result = audit.inspect_model(path)
            self.assertTrue(result["mouth_actuator_present"])
            self.assertFalse(result["policy_joint_order_matches"])
            self.assertEqual(result["decision"], "NO-GO")
            self.assertTrue(any("kinematics" in b for b in result["blockers"]))

    def test_cli_signals_blocked_without_mutating_model(self):
        before = audit.sha256(audit.DEFAULT_MODEL)
        result = subprocess.run([sys.executable, str(ROOT / "scripts/fetch_pick_preflight.py")],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("NO-GO", result.stdout)
        self.assertEqual(audit.sha256(audit.DEFAULT_MODEL), before)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo unavailable; physics check NOT RUN")
    def test_compiled_model_has_no_relative_jaw_aperture_motion(self):
        result = audit.physics_check(audit.DEFAULT_MODEL)
        self.assertTrue(result["compiled"])
        self.assertTrue(result["policy_joint_order_matches"])
        self.assertEqual(result["actuator_count"], 14)
        self.assertTrue(result["jaw_upper_same_body"])
        self.assertLess(result["mesh_origin_distance_variation_m"], 1e-12)


if __name__ == "__main__":
    unittest.main()
