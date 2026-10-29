"""Build convex collision prisms around genuinely open, shaped sockets."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import Delaunay
import cv2
from peg_sorting.utils.helpers import config, project_root

SHAPES = ("square", "triangle", "circle", "hexagon")
COLORS = dict(zip(SHAPES, ("red", "green", "blue", "yellow")))


def polygon(shape):
    if shape == "square":
        return np.array([[-.022, -.022], [.022, -.022], [.022, .022], [-.022, .022]])
    count = {"triangle": 3, "hexagon": 6, "circle": 48}[shape]
    radius = {"triangle": .029, "hexagon": .026, "circle": .024}[shape]
    angle = np.arange(count) * 2 * np.pi / count
    return radius * np.column_stack((np.cos(angle), np.sin(angle)))


def fmt(value):
    return " ".join(f"{v:.9g}" for v in np.asarray(value).ravel())


def prism(asset, name, points, bottom, top):
    n = len(points)
    verts = np.vstack((np.column_stack((points, np.full(n, bottom))),
                       np.column_stack((points, np.full(n, top)))))
    # MuJoCo computes a convex hull. Every piece here is explicitly convex.
    ET.SubElement(asset, "mesh", name=name, vertex=fmt(verts))


def camera_axes(position, target):
    z = np.asarray(position, float) - target
    z /= np.linalg.norm(z)
    x = np.cross([0, 1, 0], z)
    x /= np.linalg.norm(x)
    return fmt(np.r_[x, np.cross(z, x)])


def build_scene(seed=0, destination=None):
    rootdir = project_root()
    in_memory = destination is False
    settings, colors, cameras = config("simulation"), config("colors"), config("cameras")
    upstream = rootdir / "simulation/robots/wx250s"
    tree = ET.parse(upstream / "wx250s.xml")
    root = tree.getroot()
    root.set("model", "Tri-Wizard Peg Trial / WidowX-250S")
    compiler = root.find("compiler")
    # Paths remain relative and portable in the committed scene XML.
    assetdir = str(upstream / "assets") if in_memory else "../robots/wx250s/assets"
    compiler.set("meshdir", assetdir)
    compiler.set("texturedir", assetdir)
    root.find("option").set("timestep", str(settings["timestep"]))
    root.find("option").set("gravity", "0 0 -9.81")
    root.find("option").set("iterations", "80")
    ET.SubElement(root.find("option"), "flag", multiccd="enable")
    root.remove(root.find("keyframe"))
    ET.SubElement(root, "visual")
    visual = root.find("visual")
    ET.SubElement(visual, "global", offwidth="1600", offheight="1000", azimuth="135", elevation="-35")
    ET.SubElement(visual, "map", znear="0.005", zfar="8")
    ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6")
    # Conservative, tuned simulation servos; geometry and joint limits remain upstream.
    servo = root.find("./default/default/position")
    servo.set("kp", "450")
    servo.set("forcerange", "-35 35")
    world, assets = root.find("worldbody"), root.find("asset")
    ET.SubElement(world, "light", pos="0.1 -0.3 1.6", dir="0 0 -1", diffuse="0.7 0.7 0.7", castshadow="false")
    ET.SubElement(world, "geom", name="table", type="box", pos="0.2 0 -0.035", size="0.75 0.65 0.035", rgba="0.32 0.35 0.39 1", friction="1 0.01 0.001")
    for name, cam in cameras["cameras"].items():
        ET.SubElement(world, "camera", name=name, pos=fmt(cam["position"]),
                      xyaxes=camera_axes(cam["position"], cam["target"]), fovy=str(cam["fovy"]))
    ET.SubElement(world, "camera", name="overview", pos="1.05 -1.1 0.85",
                  xyaxes=camera_axes([1.05, -1.1, .85], [.22, 0, .12]), fovy="48")
    gripper = root.find(".//body[@name='wx250s/gripper_link']")
    ET.SubElement(gripper, "site", name="grasp_site", pos="0.108 0 0", size="0.003", rgba="1 0.4 0 0")
    ET.SubElement(gripper, "site", name="official_tcp", pos="0.093575 0 0", size="0.002", rgba="0 1 0 0")
    rng = np.random.default_rng(seed)
    equality = root.find("equality")
    for shape, center in zip(SHAPES, settings["peg_centers"]):
        xy = np.asarray(center) + rng.uniform(-.006, .006, 2)
        yaw = rng.uniform(-.35, .35)
        body = ET.SubElement(world, "body", name=f"peg_{shape}", pos=fmt([*xy, .051]),
                             quat=fmt([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]))
        ET.SubElement(body, "freejoint", name=f"free_{shape}")
        prism(assets, f"mesh_{shape}", polygon(shape), -.05, .05)
        geometry = {"type": "box", "size": ".022 .022 .05"} if shape == "square" else (
            {"type": "cylinder", "size": ".024 .05"} if shape == "circle" else {"type": "mesh", "mesh": f"mesh_{shape}"})
        ET.SubElement(body, "geom", name=f"peg_{shape}_geom", **geometry,
                      mass=str(settings["peg_mass"]), rgba=fmt(colors[COLORS[shape]]["rgba"]),
                      friction="1.2 0.01 0.001", condim="4", solref="0.02 1", solimp="0.9 0.95 0.001")
        ET.SubElement(equality, "weld", name=f"grasp_{shape}", body1="wx250s/gripper_link",
                      body2=f"peg_{shape}", active="false", solref="0.004 1", solimp="0.98 0.999 0.0001")
    board = ET.SubElement(world, "body", name="socket_board")
    bc, bh = settings["board_center"], settings["board_half_size"]
    ET.SubElement(board, "geom", name="board_floor", type="box", pos=fmt([*bc, .001]),
                  size=fmt([*bh, .001]), rgba="0.10 0.10 0.12 1")
    # Each tile is triangulated outside its convex aperture. No mesh spans a hole.
    for shape, center in zip(SHAPES, settings["socket_centers"]):
        inner = polygon(shape)
        # Offset convex polygon edges by the configured normal clearance.
        radius = np.linalg.norm(inner[0])
        apothem = radius * np.cos(np.pi / len(inner))
        inner = inner * (1 + settings["socket_clearance"] / apothem)
        outer = np.array([[-.075, -.07], [.075, -.07], [.075, .07], [-.075, .07]])
        points = np.vstack((outer, inner))
        for i, tri in enumerate(Delaunay(points).simplices):
            triangle = points[tri]
            centroid = triangle.mean(axis=0)
            if cv2.pointPolygonTest(inner.astype(np.float32), tuple(centroid), False) >= 0:
                continue
            name = f"rim_{shape}_{i}"
            prism(assets, name, triangle, .002, settings["board_height"])
            ET.SubElement(board, "geom", name=name, type="mesh", mesh=name,
                          pos=fmt([*center, 0]), rgba="0.78 0.79 0.8 1", friction="0.6 0.01 0.001")
        ET.SubElement(board, "site", name=f"socket_{shape}", pos=fmt([*center, .002]), size="0.001", rgba="0 0 0 0")
    # Magenta perimeter makes the board detectable without a fixed image ROI.
    for axis in range(2):
        for sign in (-1, 1):
            pos = np.array([*bc, .031])
            pos[axis] += sign * bh[axis]
            size = [*bh, .001]
            size[axis] = .003
            ET.SubElement(board, "geom", type="box", pos=fmt(pos), size=fmt(size),
                          rgba="0.65 0.08 0.65 1", contype="0", conaffinity="0")
    if in_memory:
        return ET.tostring(root, encoding="unicode")
    destination = Path(destination or rootdir / "simulation/scenes/peg_sorting.xml")
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="unicode")
    # Standalone object assets are actual usable MJCFs, generated from the same geometry.
    objects = rootdir / "simulation/objects"
    objects.mkdir(parents=True, exist_ok=True)
    for shape in SHAPES:
        obj = ET.Element("mujoco", model=f"{shape}_peg")
        asset = ET.SubElement(obj, "asset")
        prism(asset, f"mesh_{shape}", polygon(shape), -.05, .05)
        objworld = ET.SubElement(obj, "worldbody")
        objworld.append(copy.deepcopy(world.find(f"body[@name='peg_{shape}']")))
        ET.indent(obj)
        ET.ElementTree(obj).write(objects / f"{shape}_peg.xml", encoding="unicode")
    obj = ET.Element("mujoco", model="socket_board")
    asset = ET.SubElement(obj, "asset")
    for mesh in assets.findall("mesh"):
        if mesh.get("name", "").startswith("rim_"):
            asset.append(copy.deepcopy(mesh))
    ET.SubElement(obj, "worldbody").append(copy.deepcopy(board))
    ET.indent(obj)
    ET.ElementTree(obj).write(objects / "socket_board.xml", encoding="unicode")
    return destination
