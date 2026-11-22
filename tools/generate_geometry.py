"""Generate the Gazebo geometry and scale drawing from keeper.design.json."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, {key: str(value) for key, value in attrs.items()})
    if text is not None:
        node.text = str(text)
    return node


def write_xml(root, path):
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    with path.open("ab") as stream:
        stream.write(b"\n")


def generate_robot(config):
    geometry, keeper, motion = config["geometry"], config["keeper"], config["motion"]
    robot = ET.Element("robot", name="goalkeeper_robot")
    robot.append(ET.Comment("Generated from config/keeper.design.json; run tools/generate_geometry.py."))
    element(robot, "link", name="world")
    for name in ("base_link", "slider_carriage"):
        link = element(robot, "link", name=name)
        inertia = element(link, "inertial")
        element(inertia, "mass", value="0.001")
        element(inertia, "inertia", ixx="0.000001", iyy="0.000001", izz="0.000001",
                ixy="0", ixz="0", iyz="0")
    fixed = element(robot, "joint", name="world_to_base", type="fixed")
    element(fixed, "parent", link="world")
    element(fixed, "child", link="base_link")
    body = element(robot, "link", name="keeper_body")
    parts, depth = keeper["footprint"], keeper["depth_m"]
    area = sum(part["width_m"] * part["height_m"] for part in parts)
    masses = [keeper["mass_kg"] * part["width_m"] * part["height_m"] / area for part in parts]
    cy = sum(mass * part["center_y_m"] for part, mass in zip(parts, masses)) / keeper["mass_kg"]
    cz = sum(mass * part["center_z_m"] for part, mass in zip(parts, masses)) / keeper["mass_kg"]
    ixx = iyy = izz = 0.0
    for part, mass in zip(parts, masses):
        width, height = part["width_m"], part["height_m"]
        dy, dz = part["center_y_m"] - cy, part["center_z_m"] - cz
        ixx += mass * ((width ** 2 + height ** 2) / 12 + dy ** 2 + dz ** 2)
        iyy += mass * ((depth ** 2 + height ** 2) / 12 + dz ** 2)
        izz += mass * ((depth ** 2 + width ** 2) / 12 + dy ** 2)
        if part["name"] in ("head", "neck"):
            color = "0.82 0.61 0.43 1"
        elif "glove" in part["name"]:
            color = "0.95 0.95 0.95 1"
        elif "leg" in part["name"] or "foot" in part["name"]:
            color = "0.08 0.13 0.20 1"
        else:
            color = "0.08 0.65 0.34 1"
        for kind in ("visual", "collision"):
            shape = element(body, kind, name=part["name"])
            element(shape, "origin", xyz=f'0 {part["center_y_m"]:.9g} {part["center_z_m"]:.9g}', rpy="0 0 0")
            box_geometry = element(shape, "geometry")
            element(box_geometry, "box", size=f"{depth:.9g} {width:.9g} {height:.9g}")
            if kind == "visual":
                material = element(shape, "material", name=part["name"] + "_material")
                element(material, "color", rgba=color)
    inertia = element(body, "inertial")
    element(inertia, "origin", xyz=f"0 {cy:.9g} {cz:.9g}", rpy="0 0 0")
    element(inertia, "mass", value=keeper["mass_kg"])
    element(inertia, "inertia", ixx=f"{ixx:.9g}", iyy=f"{iyy:.9g}", izz=f"{izz:.9g}",
            ixy="0", ixz="0", iyz="0")
    for name, kind, parent, child, axis, z, capability in (
        ("slide_joint", "prismatic", "base_link", "slider_carriage", "0 1 0", 0, motion["slide"]),
        ("tilt_joint", "revolute", "slider_carriage", "keeper_body", "1 0 0", geometry["pivot_height_m"], motion["tilt"]),
    ):
        joint = element(robot, "joint", name=name, type=kind)
        element(joint, "parent", link=parent)
        element(joint, "child", link=child)
        element(joint, "origin", xyz=f"0 0 {z:.9g}", rpy="0 0 0")
        element(joint, "axis", xyz=axis)
        element(joint, "limit", lower=capability["minimum"], upper=capability["maximum"],
                velocity=capability["max_speed"], effort="2000")
    write_xml(robot, ROOT / "src/goalkeeper_description/urdf/goalkeeper.urdf")


def update_world(config):
    path = ROOT / "src/goalkeeper_sim/worlds/goalkeeper_world.sdf"
    tree = ET.parse(path)
    world = tree.getroot().find("world")
    geometry = config["geometry"]
    width, height, radius = (geometry[key] for key in ("goal_width_m", "goal_height_m", "post_radius_m"))
    goal = world.find("model[@name='goal']")
    # The back edge of each 0.12 m goal member is at the x=10 goal plane.
    goal.find("pose").text = f'{geometry["goal_plane_world_x_m"] - radius:.9g} 0 0 0 0 0'
    for name, sign in (("left_post", 1), ("right_post", -1)):
        post = goal.find(f"link[@name='{name}']")
        post.find("pose").text = f"0 {sign * (width / 2 + radius):.9g} {(height + 2 * radius) / 2:.9g} 0 0 0"
        for cylinder in post.findall("./*/geometry/cylinder"):
            cylinder.find("radius").text = str(radius)
            cylinder.find("length").text = f"{height + 2 * radius:.9g}"
    bar = goal.find("link[@name='crossbar']")
    bar.find("pose").text = f"0 0 {height + radius:.9g} {math.pi / 2:.12g} 0 0"
    for cylinder in bar.findall("./*/geometry/cylinder"):
        cylinder.find("radius").text = str(radius)
        cylinder.find("length").text = f"{width + 4 * radius:.9g}"
    for color in goal.findall("./link/visual/material/*"):
        if color.tag in ("ambient", "diffuse"):
            color.text = "1 1 1 1"
    spot = world.find("model[@name='field_marking']/link/visual[@name='penalty_spot']/pose")
    spot.text = f'{geometry["goal_plane_world_x_m"] - geometry["penalty_distance_m"]:.9g} 0 0.006 0 0 0'
    ball = world.find("include[name='soccer_ball']")
    if ball is None:
        ball = element(world, "include")
        element(ball, "uri", "model://soccer_ball")
        element(ball, "name", "soccer_ball")
        element(ball, "pose")
    ball.find("pose").text = f'{geometry["goal_plane_world_x_m"] - geometry["penalty_distance_m"]:.9g} 0 {geometry["ball_radius_m"]:.9g} 0 0 0'
    write_xml(tree.getroot(), path)


def generate_drawing(config):
    geometry, keeper = config["geometry"], config["keeper"]
    scale, center, floor = 110, 550, 405
    width, height = geometry["goal_width_m"], geometry["goal_height_m"]
    left, top = center - width * scale / 2, floor - height * scale
    shapes = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="510" viewBox="0 0 1100 510">',
        '<rect width="1100" height="510" fill="#f7fafc"/>',
        '<g font-family="Arial, sans-serif" fill="#162b3e">',
        '<text x="55" y="42" font-size="26" font-weight="bold">Regulation goal and adult goalkeeper scale</text>',
        '<text x="55" y="70" font-size="15">Front view | 1.90 m human-shaped body | original slide + tilt motion</text>',
        f'<rect x="{left:.3f}" y="{top:.3f}" width="{width * scale:.3f}" height="{height * scale:.3f}" fill="#e9f4ee"/>',
        f'<path d="M {left - geometry["post_radius_m"] * scale:.3f} {floor} V {top - geometry["post_radius_m"] * scale:.3f} H {left + (width + geometry["post_radius_m"]) * scale:.3f} V {floor}" fill="none" stroke="#8295a4" stroke-width="{2 * geometry["post_radius_m"] * scale:.3f}"/>',
        f'<line x1="80" y1="{floor}" x2="1020" y2="{floor}" stroke="#376b4c" stroke-width="2"/>',
    ]
    for part in keeper["footprint"]:
        color = ("#d4a577" if part["name"] in ("head", "neck") else
                 "#f5f7f9" if "glove" in part["name"] else
                 "#23394c" if "leg" in part["name"] or "foot" in part["name"] else "#169b58")
        x = center - (part["center_y_m"] + part["width_m"] / 2) * scale
        y = floor - (geometry["pivot_height_m"] + part["center_z_m"] + part["height_m"] / 2) * scale
        shapes.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{part["width_m"] * scale:.3f}" height="{part["height_m"] * scale:.3f}" fill="{color}" stroke="#162b3e" stroke-width="1"/>')
    shapes.extend([
        f'<text x="550" y="113" text-anchor="middle" font-size="20">{width:.2f} m clear opening</text>',
        f'<text x="975" y="265" font-size="20">{height:.2f} m</text>',
        '<text x="550" y="438" text-anchor="middle" font-size="16">Keeper: 1.90 m body height; 1.90 m hand-to-hand span</text>',
        '<text x="550" y="466" text-anchor="middle" font-size="14">Rigid silhouette on a fixed pivot; 0.05 m foot clearance in the mounted upright pose</text>',
        '<text x="550" y="490" text-anchor="middle" font-size="14">Goal dimensions: IFAB Law 1. Body proportions and motion limits: simulation assumptions.</text>',
        '</g></svg>',
    ])
    destination = ROOT / "docs/images/goalkeeper-scale.svg"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(shapes) + "\n", encoding="utf-8", newline="\n")


if __name__ == "__main__":
    design = json.loads((ROOT / "config/keeper.design.json").read_text(encoding="utf-8"))
    generate_robot(design)
    update_world(design)
    generate_drawing(design)
    print("Updated URDF, goal opening, penalty mark, and scale drawing.")
