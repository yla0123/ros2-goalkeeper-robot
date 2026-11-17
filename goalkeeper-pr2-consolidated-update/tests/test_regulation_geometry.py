"""Offline checks of generated geometry, fixed-panel motion and scoring."""
from collections import Counter
from dataclasses import replace
import math
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
        for actual, expected in zip(diamond, ((0, 1.50), (0.725, 1.95), (0, 2.40), (-0.725, 1.95))):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])
        rect = parts["rectangle_panel"]
        self.assertAlmostEqual(max(y for y, _ in rect) - min(y for y, _ in rect), 0.50)
        self.assertAlmostEqual(min(z for _, z in rect), 0.43)
        self.assertAlmostEqual(max(z for _, z in rect), 1.64)
        base = next(p for p in self.d.fixed_parts() if p.name == "chassis")
        self.assertEqual((base.width_m, base.depth_m), (0.85, 0.75))
        self.assertAlmostEqual(max(z for _, z in parts["chassis"]), 0.22)
        self.assertAlmostEqual(min(z for _, z in parts["front_left_wheel"]), 0)

    def test_same_mesh_for_visual_and_collision_and_matching_planner_projection(self):
        link = self.robot.find("link[@name='keeper_board']")
        mesh_names = [link.find(f"{kind}[@name='diamond_panel']/geometry/mesh").get("filename")
                      for kind in ("visual", "collision")]
        self.assertEqual(mesh_names, ["package://goalkeeper_description/meshes/diamond_panel.stl"]*2)
        points = [tuple(map(float, line.strip().split()[1:]))
                  for line in self.mesh.read_text().splitlines() if line.strip().startswith("vertex ")]
        origin = tuple(map(float, link.find("collision[@name='diamond_panel']/origin").get("xyz").split()))
        pivot = self.d.pivot_height_m
        projected = {(round(y + origin[1], 6), round(z + origin[2] + pivot, 6)) for _, y, z in points}
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
        for y, z in ((0, 1.75), (0.55, 1.95), (0, 2.35)):
            self.assertTrue(covers(CrossingTarget(y, z, 0.55, 0.10), self.d.neutral, self.d))
        for y, z in ((0.68, 2.33), (-0.68, 2.33), (0.55, 1.30)):
            self.assertFalse(covers(CrossingTarget(y, z, 0.55, 0.10), self.d.neutral, self.d))

    def test_controlled_joints_and_mobile_mass(self):
        joints = [j for j in self.robot.findall("joint") if j.get("type") != "fixed"]
        self.assertEqual(tuple(j.get("name") for j in joints), JOINT_NAMES)
        self.assertEqual(joints[0].get("type"), "prismatic")
        self.assertEqual(joints[0].find("axis").get("xyz"), "0 1 0")
        self.assertEqual(joints[1].get("type"), "revolute")
        self.assertEqual(joints[1].find("axis").get("xyz"), "1 0 0")
        self.assertEqual(joints[1].find("parent").get("link"), "keeper_chassis")
        self.assertEqual(joints[1].find("child").get("link"), "keeper_board")
        self.assertAlmostEqual(float(joints[1].find("origin").get("xyz").split()[2]), self.d.pivot_height_m)
        plugin = self.robot.find("gazebo/plugin[@name='gz::sim::systems::JointTrajectoryController']")
        self.assertEqual(tuple(j.text for j in plugin.findall("joint_name")), JOINT_NAMES)
        mass = sum(float(self.robot.find(f"link[@name='{name}']/inertial/mass").get("value"))
                   for name in ("keeper_chassis", "keeper_board"))
        self.assertEqual(mass, 85.0)
        self.assertEqual(85 * self.d.slide.max_acceleration, 6375.0)

    def test_slide_both_ways_within_speed_acceleration_and_frame_limits(self):
        start = self.d.neutral
        for displacement in (self.d.slide.minimum, self.d.slide.maximum):
            end = Pose(displacement)
            self.assertTrue(swept_clearance(start, end, self.d))
            samples = list(motion_samples(start, end, move_duration(start, end, self.d), self.d))
            self.assertLess(samples[-1][0], 0.45)
            self.assertEqual(samples[0][1][0][0], 0.0)
            self.assertEqual(samples[-1][1][0][0], displacement)
            self.assertTrue(all(abs(states[0][1]) <= self.d.slide.max_speed + 1e-8 and
                                abs(states[0][2]) <= self.d.slide.max_acceleration + 1e-8 for _, states in samples))
        self.assertFalse(swept_clearance(start, Pose(3.56), self.d))

    def test_tilt_and_slide_use_same_trajectory_and_avoid_posts(self):
        for sign in (-1, 1):
            end = Pose(sign * self.d.slide.maximum, -sign * math.pi/2)
            samples = list(motion_samples(self.d.neutral, end, move_duration(self.d.neutral, end, self.d), self.d))
            self.assertTrue(swept_clearance(self.d.neutral, end, self.d))
            self.assertTrue(any(all(abs(state[1]) > 0.01 for state in states) for _, states in samples))
            self.assertEqual(tuple(state[0] for state in samples[-1][1]), end.values())
            self.assertEqual(max(z for _, z in dict(footprint(end, self.d))["chassis"]), 0.22)
            board = dict(footprint(end, self.d))["diamond_panel"]
            self.assertAlmostEqual(min(z for _, z in board), 0.035)
            aligned = replace(self.d, geometry={**self.d.geometry, "keeper_offset_from_goal_m": 0})
            self.assertFalse(swept_clearance(aligned.neutral, end, aligned))
        self.assertTrue(swept_clearance(Pose(3.55, -math.pi/2), Pose(-3.55, math.pi/2), self.d))
        low_hinge = replace(self.d, keeper={**self.d.keeper, "pivot_height_m": 0.43})
        self.assertFalse(swept_clearance(low_hinge.neutral, Pose(0.9, -math.pi/2), low_hinge))

    def test_planner_uses_tilt_for_an_offset_shot(self):
        target = CrossingTarget(0.85, 1.95, 0.55, 0.10)
        result = plan(target, self.d)
        self.assertTrue(result["reachable"])
        self.assertLess(result["command"]["tilt_rad"], 0)
        self.assertTrue(covers(target, Pose(result["command"]["slide_m"], result["command"]["tilt_rad"]), self.d))

    def test_crossbar_and_floor_block_unsafe_geometry(self):
        short = replace(self.d, geometry={**self.d.geometry, "goal_height_m": 2.12,
                                          "keeper_offset_from_goal_m": 0})
        self.assertFalse(swept_clearance(short.neutral, short.neutral, short))
        bad = replace(self.d, keeper={**self.d.keeper, "rectangle": {**self.d.keeper["rectangle"], "bottom_m": -0.01}})
        self.assertFalse(swept_clearance(bad.neutral, bad.neutral, bad))

    def test_cases_and_front_contact_budget(self):
        for case in ("center", "low", "high_corner"):
            target = CrossingTarget(**self.d.scenarios[case], arrival_time_s=0.55, elapsed_since_kick_s=0.10)
            result = plan(target, self.d)
            self.assertAlmostEqual(result["motion_budget_s"], 0.379)
            self.assertTrue(result["reachable"])
            command = result["command"]
            self.assertTrue(covers(target, Pose(command["slide_m"], command["tilt_rad"]), self.d))
            self.assertLessEqual(command["motion_duration_s"], result["motion_budget_s"])
        late = CrossingTarget(**self.d.scenarios["high_corner"], arrival_time_s=0.55,
                              elapsed_since_kick_s=0.20)
        self.assertEqual(plan(late, self.d)["reason"], "NO_SAFE_INTERCEPT_IN_TIME")

    def test_upright_poses_cover_every_legal_ball_center_in_projection(self):
        # Align the centreline with any legal lateral centre. Verify continuous
        # vertical coverage, including the interval between chassis and board.
        lateral = self.d.goal_width_m / 2 - self.d.radius
        self.assertAlmostEqual(self.d.slide.minimum, -lateral)
        self.assertAlmostEqual(self.d.slide.maximum, lateral)
        intervals = sorted((min(z for _, z in part.corners()), max(z for _, z in part.corners()))
                           for part in self.d.fixed_parts() if abs(part.y_m) < 1e-9)
        covered_to = self.d.radius
        for low, high in intervals:
            self.assertLessEqual(low, covered_to + 1e-9)
            covered_to = max(covered_to, high)
        self.assertGreaterEqual(covered_to, self.d.goal_height_m - self.d.radius)
        for y in (-lateral, lateral):
            for z in (self.d.radius, self.d.goal_height_m - self.d.radius):
                self.assertTrue(covers(CrossingTarget(y, z, 0.55, 0.10), Pose(y), self.d))
        goal_x = float(self.world.findtext("model[@name='goal']/pose").split()[0])
        frame_front_x = goal_x - self.d.geometry["post_radius_m"]
        chassis = next(self.d.chassis_parts())
        rear_x = self.d.geometry["goal_plane_world_x_m"] - self.d.keeper_offset_from_goal_m + chassis.depth_m / 2
        self.assertAlmostEqual(frame_front_x - rear_x, 0.025)

    def test_whole_ball_goal_is_separate_from_planner(self):
        for end in ((10.10, 0, 1), (10.20, 3.57, 1), (10.20, 0, 2.35), (10.20, 0, 0.107)):
            start = (10.05, end[1], end[2])
            self.assertIsNone(goal_crossing(start, end, self.d))
        self.assertIsNotNone(goal_crossing((10.05, 0, 0.109), (10.20, 0, 0.109), self.d))

    def test_controller_rejects_missing_or_moving_joint_feedback(self):
        limits = (0.02, 0.035)
        self.assertIsNone(state_pose((), (), (), JOINT_NAMES, self.d.axes, limits))
        self.assertIsNone(state_pose(JOINT_NAMES, [0, 0], [0, 0.2], JOINT_NAMES, self.d.axes, limits))
        self.assertEqual(state_pose(JOINT_NAMES, [0.08, -0.1], [0, 0], JOINT_NAMES, self.d.axes, limits),
                         [0.08, -0.1])


if __name__ == "__main__":
    unittest.main()
