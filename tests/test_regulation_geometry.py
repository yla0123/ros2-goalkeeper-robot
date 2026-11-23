"""Behavior and asset-consistency checks for the regulation-size model."""
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/goalkeeper_control"))

from goalkeeper_control.model import CrossingTarget, Design, RobotState
from goalkeeper_control.planner import covers, keeper_corners, plan, swept_clearance, trajectory
from goalkeeper_control.protocol import decode_move, encode_plan


class RegulationGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = Design.load(ROOT / "config/keeper.design.json")
        cls.world = ET.parse(ROOT / "src/goalkeeper_sim/worlds/goalkeeper_world.sdf").getroot().find("world")
        cls.robot = ET.parse(ROOT / "src/goalkeeper_description/urdf/goalkeeper.urdf").getroot()

    def test_clear_goal_aperture_uses_inside_edges(self):
        goal = self.world.find("model[@name='goal']")
        left = goal.find("link[@name='left_post']")
        right = goal.find("link[@name='right_post']")
        radius = float(left.find("collision/geometry/cylinder/radius").text)
        y_left = float(left.find("pose").text.split()[1])
        y_right = float(right.find("pose").text.split()[1])
        self.assertAlmostEqual(y_left - y_right - 2 * radius, 7.32)
        bar = goal.find("link[@name='crossbar']")
        underside = float(bar.find("pose").text.split()[2]) - float(bar.find("collision/geometry/cylinder/radius").text)
        self.assertAlmostEqual(underside, 2.44)
        goal_x = float(goal.find("pose").text.split()[0])
        self.assertAlmostEqual(goal_x + radius, self.design.goal_plane_world_x_m)

    def test_penalty_mark_and_initial_ball_are_eleven_meters_away(self):
        spot = self.world.find("model[@name='field_marking']/link/visual[@name='penalty_spot']/pose")
        spot_x = float(spot.text.split()[0])
        self.assertAlmostEqual(self.design.goal_plane_world_x_m - spot_x, 11.0)
        ball = self.world.find("include[name='soccer_ball']/pose")
        self.assertAlmostEqual(float(ball.text.split()[0]), spot_x)

    def test_robot_and_planner_share_each_blocking_part(self):
        body = self.robot.find("link[@name='keeper_body']")
        collisions = {node.attrib["name"]: node for node in body.findall("collision")}
        self.assertEqual(set(collisions), {part.name for part in self.design.footprint})
        for part in self.design.footprint:
            xyz = list(map(float, collisions[part.name].find("origin").attrib["xyz"].split()))
            dimensions = list(map(float, collisions[part.name].find("geometry/box").attrib["size"].split()))
            self.assertEqual(xyz, [0.0, part.center_y_m, part.center_z_m])
            self.assertEqual(dimensions, [self.design.keeper_depth_m, part.width_m, part.height_m])

    def test_original_slide_and_tilt_axes_are_preserved(self):
        slide = self.robot.find("joint[@name='slide_joint']")
        tilt = self.robot.find("joint[@name='tilt_joint']")
        self.assertEqual(slide.attrib["type"], "prismatic")
        self.assertEqual(slide.find("axis").attrib["xyz"], "0 1 0")
        self.assertEqual(tilt.attrib["type"], "revolute")
        self.assertEqual(tilt.find("axis").attrib["xyz"], "1 0 0")
        self.assertIsNone(self.robot.find("joint[@name='vertical_joint']"))
        for node, axis in ((slide, self.design.slide), (tilt, self.design.tilt)):
            self.assertAlmostEqual(float(node.find("limit").attrib["lower"]), axis.minimum)
            self.assertAlmostEqual(float(node.find("limit").attrib["upper"]), axis.maximum)

    def test_height_and_arm_span_are_adult_scale(self):
        corners = keeper_corners(0, 0, self.design)
        self.assertAlmostEqual(max(p[0] for p in corners) - min(p[0] for p in corners), 1.90)
        self.assertAlmostEqual(max(p[1] for p in corners) - min(p[1] for p in corners), 1.90)
        self.assertAlmostEqual(min(p[1] for p in corners), 0.05)
        self.assertAlmostEqual(max(p[1] for p in corners), 1.95)

    def test_empty_space_between_head_and_glove_is_not_solid(self):
        self.assertFalse(covers(CrossingTarget(0.55, 1.80), 0, 0, self.design))
        self.assertTrue(covers(CrossingTarget(0.875, 1.43), 0, 0, self.design))

    def test_full_size_goal_does_not_reject_valid_wide_shot_as_outside(self):
        result = plan(CrossingTarget(3.2, 1.2), self.design)
        self.assertNotEqual(result["reason"], "TARGET_OUTSIDE_GOAL")
        self.assertFalse(result["reachable"])
        self.assertIsNone(result["command"])

    def test_outside_goal_is_rejected(self):
        for y, z in ((3.67, 1.0), (0.0, 2.45), (0.0, -0.01)):
            self.assertEqual(plan(CrossingTarget(y, z), self.design)["reason"], "TARGET_OUTSIDE_GOAL")

    def test_center_hold_can_be_encoded_without_zero_duration_failure(self):
        result = plan(CrossingTarget(0.0, 1.3), self.design)
        self.assertTrue(result["reachable"])
        self.assertAlmostEqual(result["command"]["slide_m"], 0.0)
        self.assertAlmostEqual(result["command"]["tilt_rad"], 0.0)
        self.assertGreater(decode_move(encode_plan(result, self.design), self.design).motion_ms, 0)

    def test_demo_respects_deadline_clearance_and_motion_limits(self):
        result = plan(CrossingTarget(1.3, 1.4), self.design)
        self.assertTrue(result["reachable"])
        command = result["command"]
        self.assertLessEqual(command["motion_finishes_at_s"] + self.design.settling_margin_s, 0.55 + 1e-9)
        self.assertTrue(covers(CrossingTarget(1.3, 1.4), command["slide_m"], command["tilt_rad"], self.design))
        rows = trajectory(result, self.design)
        for row in rows:
            self.assertLessEqual(abs(row["slide_velocity_mps"]), self.design.slide.max_speed + 1e-9)
            self.assertLessEqual(abs(row["tilt_velocity_radps"]), self.design.tilt.max_speed + 1e-9)
            self.assertLessEqual(abs(row["slide_acceleration_mps2"]), self.design.slide.max_acceleration + 1e-9)
            self.assertLessEqual(abs(row["tilt_acceleration_radps2"]), self.design.tilt.max_acceleration + 1e-9)
            for y, z in keeper_corners(row["slide_m"], row["tilt_rad"], self.design):
                self.assertLessEqual(abs(y), self.design.goal_width_m / 2 - self.design.frame_clearance_m + 1e-9)
                self.assertGreaterEqual(z, self.design.frame_clearance_m - 1e-9)
                self.assertLessEqual(z, self.design.goal_height_m - self.design.frame_clearance_m + 1e-9)
        self.assertAlmostEqual(rows[-1]["slide_m"], command["slide_m"])
        self.assertAlmostEqual(rows[-1]["tilt_rad"], command["tilt_rad"])
        self.assertAlmostEqual(rows[-1]["time_s"], 0.55)

    def test_stale_prediction_produces_no_move(self):
        result = plan(CrossingTarget(1.3, 1.4, elapsed_since_kick_s=0.56), self.design)
        self.assertFalse(result["reachable"])
        self.assertIsNone(result["command"])

    def test_footprint_dimensions_must_agree_with_declared_height(self):
        with self.assertRaises(ValueError):
            replace(self.design, keeper_height_m=0.65).validate()

    def test_tilted_body_must_clear_the_frame(self):
        self.assertTrue(swept_clearance(RobotState(), 0, -math.pi / 4, self.design))
        self.assertFalse(swept_clearance(RobotState(), 3.2, 0, self.design))

    def test_cli_demo_and_csv_work_without_an_examples_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "path.csv"
            process = subprocess.run([sys.executable, str(ROOT / "run_keeper.py"), "demo", "--path-csv", str(destination)],
                                     capture_output=True, text=True, check=True)
            output = json.loads(process.stdout)
            self.assertTrue(output["plan"]["reachable"])
            self.assertEqual(output["coordinate_input"]["crossing_y_m"], 1.3)
            self.assertGreater(len(destination.read_text().splitlines()), 10)


if __name__ == "__main__":
    unittest.main()
