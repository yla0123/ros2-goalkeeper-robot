"""Offline checks of generated geometry, fixed-panel motion and scoring."""
from collections import Counter
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/goalkeeper_control"))
from goalkeeper_control.model import CrossingTarget, Design, JOINT_NAMES, Pose
from goalkeeper_control.planner import covers, footprint, motion_samples, move_duration, plan, swept_clearance
from goalkeeper_control.protocol import goal_crossing, state_pose


class DiamondKeeperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = Design.load(ROOT / "config/keeper.design.json")
        cls.world = ET.parse(ROOT / "src/goalkeeper_sim/worlds/goalkeeper_world.sdf").getroot().find("world")
        cls.robot = ET.parse(ROOT / "src/goalkeeper_description/urdf/goalkeeper.urdf").getroot()
        cls.mesh = ROOT / "src/goalkeeper_description/meshes/diamond_panel.stl"

    def test_goal_visual_and_collision_dimensions(self):
        goal = self.world.find("model[@name='goal']")
        left, right, bar = (goal.find(f"link[@name='{name}']") for name in ("left_post", "right_post", "crossbar"))
        yl, yr = (float(p.findtext("pose").split()[1]) for p in (left, right))
        self.assertEqual((yl, yr), (3.72, -3.72))
        for mode in ("visual", "collision"):
            radii = [float(node.findtext(f"{mode}/geometry/cylinder/radius")) for node in (left, right, bar)]
            self.assertEqual(radii, [0.06]*3)
            self.assertAlmostEqual(yl - yr - sum(radii[:2]), 7.32)
            self.assertAlmostEqual(yl - yr + sum(radii[:2]), 7.56)
            z = float(bar.findtext("pose").split()[2])
            self.assertAlmostEqual(z - radii[2], 2.44)
            self.assertAlmostEqual(z + radii[2], 2.56)

    def test_penalty_distance(self):
        frame = self.world.find("model[@name='goal']")
        x = float(frame.findtext("pose").split()[0]) + self.d.geometry["post_radius_m"]
        spot = self.world.findtext("model[@name='field_marking']/link/visual[@name='penalty_spot']/pose")
        self.assertAlmostEqual(x - float(spot.split()[0]), 11.0)

    def test_panels_and_chassis_exact_front_dimensions(self):
        parts = dict(footprint(self.d.neutral, self.d))
        diamond = parts["diamond_panel"]
        self.assertEqual(len(diamond), 4)
        for actual, expected in zip(diamond, ((0, 1.25), (0.725, 1.70), (0, 2.15), (-0.725, 1.70))):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])
        rect = parts["rectangle_panel"]
        self.assertAlmostEqual(max(y for y, _ in rect) - min(y for y, _ in rect), 0.50)
        self.assertAlmostEqual(min(z for _, z in rect), 0.22)
        self.assertAlmostEqual(max(z for _, z in rect), 1.35)
        base = next(p for p in self.d.fixed_parts() if p.name == "chassis")
        self.assertEqual((base.width_m, base.depth_m), (0.85, 0.75))
        self.assertAlmostEqual(max(z for _, z in parts["chassis"]), 0.22)
        self.assertAlmostEqual(min(z for _, z in parts["front_left_wheel"]), 0)

    def test_same_mesh_for_visual_and_collision_and_matching_planner_projection(self):
        link = self.robot.find("link[@name='keeper_chassis']")
        mesh_names = [link.find(f"{kind}[@name='diamond_panel']/geometry/mesh").get("filename")
                      for kind in ("visual", "collision")]
        self.assertEqual(mesh_names, ["package://goalkeeper_description/meshes/diamond_panel.stl"]*2)
        points = [tuple(map(float, line.strip().split()[1:]))
                  for line in self.mesh.read_text().splitlines() if line.strip().startswith("vertex ")]
        origin = tuple(map(float, link.find("collision[@name='diamond_panel']/origin").get("xyz").split()))
        projected = {(round(y + origin[1], 6), round(z + origin[2], 6)) for _, y, z in points}
        expected = {(round(y, 6), round(z, 6)) for y, z in dict(footprint(self.d.neutral, self.d))["diamond_panel"]}
        self.assertEqual(projected, expected)
        self.assertAlmostEqual(max(p[0] for p in points) - min(p[0] for p in points), 0.04)

    def test_diamond_mesh_is_closed_and_outward(self):
        vertices = [tuple(map(float, line.strip().split()[1:]))
                    for line in self.mesh.read_text().splitlines() if line.strip().startswith("vertex ")]
        faces = list(zip(vertices[0::3], vertices[1::3], vertices[2::3]))
        self.assertEqual(len(faces), 12)
        edges = Counter(tuple(sorted((a, b))) for face in faces for a, b in zip(face, face[1:] + face[:1]))
        self.assertTrue(all(n == 2 for n in edges.values()))
        volume = sum(a[0]*(b[1]*c[2]-b[2]*c[1]) + a[1]*(b[2]*c[0]-b[0]*c[2]) +
                     a[2]*(b[0]*c[1]-b[1]*c[0]) for a, b, c in faces) / 6
        self.assertAlmostEqual(volume, 1.45 * 0.90 * 0.04 / 2)

    def test_panel_area_is_solid_but_bounding_box_corners_are_empty(self):
        for y, z in ((0, 1.75), (0.55, 1.70), (0, 2.10)):
            self.assertTrue(covers(CrossingTarget(y, z, 0.55, 0.10), self.d.neutral, self.d))
        for y, z in ((0.68, 2.08), (-0.68, 2.08), (0.55, 1.30)):
            self.assertFalse(covers(CrossingTarget(y, z, 0.55, 0.10), self.d.neutral, self.d))

    def test_one_controlled_joint_and_mobile_mass(self):
        joints = [j for j in self.robot.findall("joint") if j.get("type") != "fixed"]
        self.assertEqual(tuple(j.get("name") for j in joints), JOINT_NAMES)
        self.assertEqual(joints[0].get("type"), "prismatic")
        self.assertEqual(joints[0].find("axis").get("xyz"), "0 1 0")
        plugin = self.robot.find("gazebo/plugin[@name='gz::sim::systems::JointTrajectoryController']")
        self.assertEqual([j.text for j in plugin.findall("joint_name")], ["slide_joint"])
        mass = self.robot.find("link[@name='keeper_chassis']/inertial/mass")
        self.assertEqual(float(mass.get("value")), 85.0)
        self.assertEqual(85 * self.d.slide.max_acceleration, 340.0)

    def test_slide_both_ways_within_speed_acceleration_and_frame_limits(self):
        start = self.d.neutral
        for displacement in (-2.65, 2.65):
            end = Pose(displacement)
            self.assertTrue(swept_clearance(start, end, self.d))
            samples = list(motion_samples(start, end, move_duration(start, end, self.d), self.d))
            self.assertEqual(samples[0][1][0][0], 0.0)
            self.assertEqual(samples[-1][1][0][0], displacement)
            self.assertTrue(all(abs(states[0][1]) <= self.d.slide.max_speed + 1e-8 and
                                abs(states[0][2]) <= self.d.slide.max_acceleration + 1e-8 for _, states in samples))
        self.assertFalse(swept_clearance(start, Pose(3.1), self.d))

    def test_crossbar_and_floor_block_unsafe_geometry(self):
        short = replace(self.d, geometry={**self.d.geometry, "goal_height_m": 2.12})
        self.assertFalse(swept_clearance(short.neutral, short.neutral, short))
        bad = replace(self.d, keeper={**self.d.keeper, "rectangle": {**self.d.keeper["rectangle"], "bottom_m": -0.01}})
        self.assertFalse(swept_clearance(bad.neutral, bad.neutral, bad))

    def test_cases_and_front_contact_budget(self):
        for case, expected in (("center", True), ("low", True), ("high_corner", False)):
            target = CrossingTarget(**self.d.scenarios[case], arrival_time_s=0.55, elapsed_since_kick_s=0.10)
            result = plan(target, self.d)
            self.assertAlmostEqual(result["motion_budget_s"], 0.405)
            self.assertEqual(result["reachable"], expected)
            if expected:
                command = result["command"]
                self.assertTrue(covers(target, Pose(command["slide_m"]), self.d))
                self.assertLessEqual(command["motion_duration_s"], result["motion_budget_s"])
            else:
                self.assertEqual(result["reason"], "ABOVE_PANEL_COVERAGE")
                self.assertIsNone(result["command"])

    def test_whole_ball_goal_is_separate_from_planner(self):
        for end in ((10.10, 0, 1), (10.20, 3.57, 1), (10.20, 0, 2.35), (10.20, 0, 0.107)):
            start = (10.05, end[1], end[2])
            self.assertIsNone(goal_crossing(start, end, self.d))
        self.assertIsNotNone(goal_crossing((10.05, 0, 0.109), (10.20, 0, 0.109), self.d))

    def test_controller_rejects_missing_or_moving_slide_feedback(self):
        self.assertIsNone(state_pose((), (), (), JOINT_NAMES, (self.d.slide,), (0.02,)))
        self.assertIsNone(state_pose(JOINT_NAMES, [0], [0.2], JOINT_NAMES, (self.d.slide,), (0.02,)))
        self.assertEqual(state_pose(JOINT_NAMES, [0.08], [0], JOINT_NAMES, (self.d.slide,), (0.02,)), [0.08])


if __name__ == "__main__":
    unittest.main()
