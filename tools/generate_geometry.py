"""Regenerate the launched URDF/world and the dimensioned front view from JSON."""
from base64 import b64encode
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "src/goalkeeper_description/meshes"
FRONT_PRINT = MESH_DIR / "keeper-front-print.png"
FRONT_MESH = MESH_DIR / "keeper-front-decal.obj"
sys.path.insert(0, str(ROOT / "src/goalkeeper_control"))
from goalkeeper_control.model import Design, JOINT_NAMES
from goalkeeper_control.planner import footprint


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        node.text = str(text)
    return node


def write_xml(root, path):
    ET.indent(root, space="  ")
    with path.open("wb") as stream:
        ET.ElementTree(root).write(stream, encoding="utf-8", xml_declaration=True)
        stream.write(b"\n")


def inertial(link, parts):
    mass = sum(p.mass_kg for p in parts)
    centers = [(p.x_m, p.y_m, p.z_m) for p in parts]
    center = tuple(sum(p.mass_kg * xyz[i] for p, xyz in zip(parts, centers)) / mass for i in range(3))
    values = dict.fromkeys(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), 0.0)
    for p, xyz in zip(parts, centers):
        x, y, z = (a - b for a, b in zip(xyz, center))
        m, d, w, h = p.mass_kg, p.depth_m, p.width_m, p.height_m
        if p.wheel:
            rr = (h / 2) ** 2
            ixx = izz = m * (3 * rr + w ** 2) / 12
            iyy = m * rr / 2
        elif p.diamond:
            vy, vz, vx = w*w / 24, h*h / 24, d*d / 12
            ixx, iyy, izz = m * (vy + vz), m * (vx + vz), m * (vx + vy)
        else:
            ixx, iyy, izz = m * (w*w + h*h) / 12, m * (d*d + h*h) / 12, m * (d*d + w*w) / 12
        for k, value in (("ixx", ixx + m*(y*y + z*z)), ("iyy", iyy + m*(x*x + z*z)),
                         ("izz", izz + m*(x*x + y*y)), ("ixy", -m*x*y), ("ixz", -m*x*z), ("iyz", -m*y*z)):
            values[k] += value
    node = element(link, "inertial")
    element(node, "origin", xyz=" ".join(f"{x:.10g}" for x in center), rpy="0 0 0")
    element(node, "mass", value=f"{mass:.10g}")
    element(node, "inertia", **{k: f"{v:.10g}" for k, v in values.items()})


def rgba(name):
    if "wheel" in name:
        return "0.10 0.14 0.19 1"
    if name == "chassis":
        return "0.20 0.35 0.51 1"
    if name == "diamond_panel":
        return "0.14 0.57 0.50 1"
    return "0.08 0.60 0.40 1"


def geometry_link(robot, name, parts):
    link = element(robot, "link", name=name)
    inertial(link, parts)
    for part in parts:
        for kind in ("visual", "collision"):
            node = element(link, kind, name=part.name)
            element(node, "origin", xyz=f"{part.x_m} {part.y_m} {part.z_m}",
                    rpy=f"{math.pi/2} 0 0" if part.wheel else "0 0 0")
            geo = element(node, "geometry")
            if part.wheel:
                element(geo, "cylinder", radius=part.height_m / 2, length=part.width_m)
            elif part.diamond:
                element(geo, "mesh", filename="package://goalkeeper_description/meshes/diamond_panel.stl")
            else:
                element(geo, "box", size=f"{part.depth_m} {part.width_m} {part.height_m}")
            if kind == "visual":
                element(element(node, "material", name=part.name + "_material"), "color", rgba=rgba(part.name))
    gz = element(robot, "gazebo", reference=name)
    element(gz, "selfCollide", "true")
    if name == "keeper_chassis":
        # Locked wheel visuals/collisions represent ideal lateral rolling.
        element(gz, "mu1", 0)
        element(gz, "mu2", 0)


def joint(robot, name, kind, parent, child, xyz="0 0 0", direction=None, axis=None, effort=None):
    node = element(robot, "joint", name=name, type=kind)
    element(node, "parent", link=parent)
    element(node, "child", link=child)
    element(node, "origin", xyz=xyz, rpy="0 0 0")
    if direction:
        element(node, "axis", xyz=direction)
        element(node, "limit", lower=axis.minimum, upper=axis.maximum, velocity=axis.max_speed, effort=effort)


def generate_robot(d):
    robot = ET.Element("robot", name="goalkeeper_robot")
    robot.append(ET.Comment("Generated from config/keeper.design.json; run tools/generate_geometry.py."))
    element(robot, "link", name="world")
    anchor = element(robot, "link", name="base_link")
    anchor_mass = element(anchor, "inertial")
    element(anchor_mass, "mass", value=0.001)
    element(anchor_mass, "inertia", ixx=1e-6, iyy=1e-6, izz=1e-6, ixy=0, ixz=0, iyz=0)
    joint(robot, "world_to_base", "fixed", "world", "base_link")
    geometry_link(robot, "keeper_chassis", tuple(d.fixed_parts()))
    link = robot.find("link[@name='keeper_chassis']")
    decal = element(link, "visual", name="keeper_front_decal")
    element(element(decal, "geometry"), "mesh",
            filename="package://goalkeeper_description/meshes/keeper-front-decal.obj")
    effort = d.simulation["effort_limit"]
    joint(robot, JOINT_NAMES[0], "prismatic", "base_link", "keeper_chassis",
          direction="0 1 0", axis=d.slide, effort=effort[0])
    gz = element(robot, "gazebo")
    controller = element(gz, "plugin", filename="gz-sim-joint-trajectory-controller-system",
                         name="gz::sim::systems::JointTrajectoryController")
    element(controller, "topic", "/model/goalkeeper_robot/joint_trajectory")
    for name in JOINT_NAMES:
        element(controller, "joint_name", name)
    for position in d.neutral.values():
        element(controller, "initial_position", position)
    for key in ("position_p_gain", "position_i_gain", "position_d_gain", "position_i_max"):
        for value in d.simulation[key]:
            element(controller, key, value)
    for value in d.simulation["position_i_max"]:
        element(controller, "position_i_min", -value)
    for value in effort:
        element(controller, "position_cmd_max", value)
        element(controller, "position_cmd_min", -value)
    state = element(gz, "plugin", filename="gz-sim-joint-state-publisher-system",
                    name="gz::sim::systems::JointStatePublisher")
    for name in JOINT_NAMES:
        element(state, "joint_name", name)
    write_xml(robot, ROOT / "src/goalkeeper_description/urdf/goalkeeper.urdf")


def generate_mesh(d):
    """Write a watertight convex rhombus prism in the panel's local coordinates."""
    part = next(p for p in d.fixed_parts() if p.diamond)
    points = [(y - part.y_m, z - part.z_m) for y, z in part.corners()]
    verts = [(x, y, z) for x in (-part.depth_m / 2, part.depth_m / 2) for y, z in points]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        faces.extend(((i, j, j + 4), (i, j + 4, i + 4)))
    lines = ["solid diamond_panel"]
    for face in faces:
        a, b, c = (verts[i] for i in face)
        u, v = (tuple(q - p for p, q in zip(a, point)) for point in (b, c))
        normal = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
        norm = math.sqrt(sum(value ** 2 for value in normal))
        lines.extend(("  facet normal " + " ".join(f"{value/norm:.10g}" for value in normal), "    outer loop"))
        lines.extend("      vertex " + " ".join(f"{value:.10g}" for value in p) for p in (a, b, c))
        lines.extend(("    endloop", "  endfacet"))
    lines.append("endsolid diamond_panel")
    destination = MESH_DIR / "diamond_panel.stl"
    destination.parent.mkdir(exist_ok=True)
    destination.write_bytes(("\n".join(lines) + "\n").encode("ascii"))


def print_geometry(d):
    """Two existing front faces share one orthographic print in pitch-side view."""
    plates = tuple(p for p in d.fixed_parts() if p.name in ("rectangle_panel", "diamond_panel"))
    coordinates = [(-y, z) for p in plates for y, z in p.corners()]
    left = min(x for x, _ in coordinates)
    right = max(x for x, _ in coordinates)
    bottom = min(z for _, z in coordinates)
    top = max(z for _, z in coordinates)
    return plates, (left, right, bottom, top)


def generate_decal(d):
    """Generate a visual-only textured surface without changing any collider."""
    if not FRONT_PRINT.is_file():
        raise FileNotFoundError(FRONT_PRINT)
    plates, (left, right, bottom, top) = print_geometry(d)
    lines = ["# Generated from keeper.design.json; front faces only.",
             "mtllib keeper-front-decal.mtl", "o keeper_front_decal"]
    uv = []
    for plate in plates:
        x = plate.x_m - plate.depth_m / 2 - 0.001
        for y, z in plate.corners():
            lines.append(f"v {x:.9g} {y:.9g} {z:.9g}")
            uv.append(((-y-left)/(right-left), (z-bottom)/(top-bottom)))
    lines.extend(f"vt {u:.9g} {v:.9g}" for u, v in uv)
    lines.extend(("vn -1 0 0", "usemtl keeper_print"))
    for offset in range(0, 4*len(plates), 4):
        for face in ((1, 3, 2), (1, 4, 3)):
            lines.append("f " + " ".join(f"{offset+i}/{offset+i}/1" for i in face))
    FRONT_MESH.write_bytes(("\n".join(lines) + "\n").encode("ascii"))
    FRONT_MESH.with_suffix(".mtl").write_bytes(
        b"newmtl keeper_print\nKa 0.7 0.7 0.7\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd keeper-front-print.png\n")


def ensure_plugin(container, name, filename):
    node = container.find(f"plugin[@name='gz::sim::systems::{name}']")
    if node is None:
        node = element(container, "plugin", name=f"gz::sim::systems::{name}", filename=filename)
    return node


def update_world(d):
    path = ROOT / "src/goalkeeper_sim/worlds/goalkeeper_world.sdf"
    tree = ET.parse(path)
    world = tree.getroot().find("world")
    for name, suffix in (("Physics", "physics"), ("UserCommands", "user-commands"),
                         ("SceneBroadcaster", "scene-broadcaster"), ("ApplyLinkWrench", "apply-link-wrench")):
        ensure_plugin(world, name, f"gz-sim-{suffix}-system")
    world.find("physics/max_step_size").text = str(d.simulation["physics_step_s"])
    g = d.geometry
    w, h, r = d.goal_width_m, d.goal_height_m, g["post_radius_m"]
    goal = world.find("model[@name='goal']")
    goal.find("pose").text = f'{g["goal_plane_world_x_m"] - r:.9g} 0 0 0 0 0'
    for name, sign in (("left_post", 1), ("right_post", -1)):
        post = goal.find(f"link[@name='{name}']")
        post.find("pose").text = f"0 {sign * (w/2+r):.9g} {d.outer_height_m/2:.9g} 0 0 0"
        for cyl in post.findall("./*/geometry/cylinder"):
            cyl.find("radius").text = str(r)
            cyl.find("length").text = f"{d.outer_height_m:.9g}"
    bar = goal.find("link[@name='crossbar']")
    bar.find("pose").text = f"0 0 {h+r:.9g} {math.pi/2:.12g} 0 0"
    for cyl in bar.findall("./*/geometry/cylinder"):
        cyl.find("radius").text = str(r)
        cyl.find("length").text = f"{d.outer_width_m:.9g}"
    # Preserve the pitch-side extent; add a two-metre apron behind the scoring plane.
    field = world.find("model[@name='soccer_field']/link")
    field.find("pose").text = f'{g["goal_plane_world_x_m"]-9:.9g} 0 -0.05 0 0 0'
    for size in field.findall("./*/geometry/box/size"):
        size.text = "22 50 0.1"
    x = g["goal_plane_world_x_m"] - g["penalty_distance_m"]
    spot = world.find("model[@name='field_marking']/link/visual[@name='penalty_spot']/pose")
    spot.text = f"{x:.9g} 0 0.006 0 0 0"
    world.find("include[name='soccer_ball']/pose").text = f"{x:.9g} 0 {d.radius:.9g} 0 0 0"
    write_xml(tree.getroot(), path)


def update_ball(d):
    path = ROOT / "src/goalkeeper_sim/models/soccer_ball/model.sdf"
    tree = ET.parse(path)
    model = tree.getroot().find("model")
    for radius in model.findall("link/*/geometry/sphere/radius"):
        radius.text = str(d.radius)
    mass = d.simulation["ball_mass_kg"]
    model.find("link/inertial/mass").text = str(mass)
    for axis in ("ixx", "iyy", "izz"):
        model.find(f"link/inertial/inertia/{axis}").text = f"{2*mass*d.radius*d.radius/5:.9g}"
    plugin = ensure_plugin(model, "PosePublisher", "gz-sim-pose-publisher-system")
    for key, value in (("publish_link_pose", "false"), ("publish_model_pose", "true"),
                       ("use_pose_vector_msg", "false"), ("update_frequency", "200"),
                       ("topic", "/model/soccer_ball/pose")):
        node = plugin.find(key)
        if node is None:
            node = element(plugin, key)
        node.text = value
    write_xml(tree.getroot(), path)


def generate_drawing(d):
    svg = ET.Element("svg", xmlns="http://www.w3.org/2000/svg", width="1180", height="796", viewBox="0 0 1180 796")
    element(svg, "rect", x=0, y=0, width=1180, height=796, fill="#f8fbfc")
    defs = element(svg, "defs")
    print_uri = "data:image/png;base64," + b64encode(FRONT_PRINT.read_bytes()).decode("ascii")
    element(defs, "image", id="keeper_print_art", href=print_uri, width="1", height="1",
            preserveAspectRatio="none")
    marker = element(defs, "marker", id="arrow", viewBox="0 0 10 10", refX=5, refY=5,
                     markerWidth=6, markerHeight=6, orient="auto-start-reverse")
    element(marker, "path", d="M 0 0 L 10 5 L 0 10 Z", fill="#35566a")
    group = element(svg, "g", **{"font-family": "Arial, sans-serif", "fill": "#132c3e"})

    def text(x, y, label, size=16, middle=False, bold=False):
        return element(group, "text", label, x=x, y=y, **{
            "font-size": size, "text-anchor": "middle" if middle else "start",
            "font-weight": "700" if bold else "400"})

    def line(x1, y1, x2, y2, arrow=False, dashed=False):
        attrs = {"stroke": "#35566a", "stroke-width": "1.3"}
        if arrow:
            attrs |= {"marker-start": "url(#arrow)", "marker-end": "url(#arrow)"}
        if dashed:
            attrs["stroke-dasharray"] = "4 4"
        element(group, "line", x1=x1, y1=y1, x2=x2, y2=y2, **attrs)

    def keeper(cx, floor, scale):
        colors = {"rectangle_panel": "#28775f", "diamond_panel": "#259686", "chassis": "#486f95"}
        parts = list(footprint(d.neutral, d))
        plates, (left, right, bottom, top) = print_geometry(d)
        plate_names = {p.name for p in plates}
        clip_name = f"print_clip_{cx}"
        clip = element(defs, "clipPath", id=clip_name)
        outlines = []
        for name, points in parts:
            color = colors.get(name, "#172d43")
            coords = " ".join(f"{cx-y*scale:.4f},{floor-z*scale:.4f}" for y, z in points)
            element(group, "polygon", points=coords, fill=color, stroke="#254057", **{"stroke-width": 0.8})
            if name in plate_names:
                element(clip, "polygon", points=coords)
                if name == "diamond_panel":
                    outlines.append(coords)
        artwork = element(group, "g", **{"clip-path": f"url(#{clip_name})"})
        element(artwork, "use", href="#keeper_print_art",
                transform=f"translate({cx+left*scale:.6g} {floor-top*scale:.6g}) "
                          f"scale({(right-left)*scale:.6g} {(top-bottom)*scale:.6g})")
        for coords in outlines:
            element(group, "polygon", points=coords, fill="none", stroke="#254057", **{"stroke-width": 0.8})

    text(56, 43, "Regulation goal and fixed diamond goalkeeper", 26, bold=True)
    text(57, 69, "Front elevation | metres | goalkeeper print on two fixed panels", 16)
    cx, floor, scale = 590, 396, 99
    ol, or_ = cx-d.outer_width_m*scale/2, cx+d.outer_width_m*scale/2
    il, ir = cx-d.goal_width_m*scale/2, cx+d.goal_width_m*scale/2
    ot, it = floor-d.outer_height_m*scale, floor-d.goal_height_m*scale
    element(group, "rect", x=il, y=it, width=ir-il, height=floor-it, fill="#e5f2e9")
    for x, y, w, h in ((ol, ot, il-ol, floor-ot), (ir, ot, or_-ir, floor-ot), (il, ot, ir-il, it-ot)):
        element(group, "rect", x=x, y=y, width=w, height=h, fill="#8298a8")
    line(ol, 100, or_, 100, True)
    line(ol, 100, ol, ot)
    line(or_, 100, or_, ot)
    text(cx, 93, f"{d.outer_width_m:.2f} m outer width", middle=True)
    line(il, 176, ir, 176, True)
    text(cx, 169, f"{d.goal_width_m:.2f} m clear opening", 14, middle=True)
    line(165, ot, 165, floor, True)
    text(58, 262, f"{d.outer_height_m:.2f} m", bold=True)
    text(58, 283, "outer height", 13)
    line(1005, it, 1005, floor, True)
    text(1020, 262, f"{d.goal_height_m:.2f} m", bold=True)
    text(1020, 283, "clear height", 13)
    line(165, floor, 1015, floor)
    keeper(cx, floor, scale)
    text(cx, 423, f'Post and crossbar diameter: {2*d.geometry["post_radius_m"]:.2f} m   •   '
         f'Penalty mark: {d.geometry["penalty_distance_m"]:.2f} m from the goal line', 15, middle=True)

    cx, floor, scale = 436, 729, 119
    text(57, 465, "Keeper detail", 19, bold=True)
    keeper(cx, floor, scale)
    line(298, floor, 576, floor)
    diamond, rect = d.keeper["diamond"], d.keeper["rectangle"]
    halfw = diamond["horizontal_diagonal_m"] / 2
    halfh = diamond["vertical_diagonal_m"] / 2
    top_z, bottom_z = diamond["center_z_m"] + halfh, diamond["center_z_m"] - halfh
    top_y, bottom_y = floor-top_z*scale, floor-bottom_z*scale
    line(286, top_y, 286, floor, True)
    line(286, top_y, cx, top_y, dashed=True)
    text(125, (top_y+floor)/2, f"{top_z:.2f} m overall")
    line(582, top_y, 582, bottom_y, True)
    line(cx, bottom_y, 582, bottom_y, dashed=True)
    text(603, (top_y+bottom_y)/2, f"{2*halfh:.2f} m vertical diagonal")
    line(cx-halfw*scale, 458, cx+halfw*scale, 458, True)
    text(cx, 451, f"{2*halfw:.2f} m horizontal diagonal", 14, middle=True)
    b = d.keeper["base"]
    line(cx-b["width_m"]*scale/2, 747, cx+b["width_m"]*scale/2, 747, True)
    text(cx, 771, f"{b['width_m']:.2f} m base width", 14, middle=True)
    text(802, 499, "Model dimensions", 17, bold=True)
    for i, label in enumerate((f"Base: {b['width_m']:.2f} W × {b['depth_m']:.2f} D × {b['height_m']:.2f} H m",
                                f"Rectangle: {rect['width_m']:.2f} W × {rect['height_m']:.2f} H m",
                                f"Diamond sloping edges: {math.hypot(halfw,halfh):.3f} m each",
                                f"Diamond centre: {diamond['center_z_m']:.2f} m above ground",
                                f"Both panels: {diamond['thickness_m']*1000:g} mm thick in this model",
                                f"Panel overlap in front view: {rect['bottom_m']+rect['height_m']-bottom_z:.2f} m",
                                f"Lateral travel: {d.slide.minimum:g} to +{d.slide.maximum:g} m")):
        text(802, 528+26*i, label, 15)
    write_xml(svg, ROOT / "docs/images/goalkeeper-scale.svg")


if __name__ == "__main__":
    design = Design.load(ROOT / "config/keeper.design.json")
    generate_mesh(design)
    generate_decal(design)
    generate_robot(design)
    update_world(design)
    update_ball(design)
    generate_drawing(design)
    print("Regenerated diamond mesh, visual print, URDF, world, ball model and front elevation.")
