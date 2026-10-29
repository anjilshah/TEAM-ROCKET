"""Fetch the BSD-licensed, pinned upstream model; no ROS or conversion required."""
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION = "f3b6013629c7b94f8ee93fd977b161fa8350f9bc"
ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "simulation/robots/wx250s"
FILES = ["wx250s.xml", "LICENSE", "README.md", "CHANGELOG.md"]
FILES += ["assets/" + n for n in [
    "interbotix_black.png", "wx250s_1_base.stl", "wx250s_2_shoulder.stl",
    "wx250s_3_upper_arm.stl", "wx250s_4_upper_forearm.stl", "wx250s_5_lower_forearm.stl",
    "wx250s_6_wrist.stl", "wx250s_7_gripper.stl", "wx250s_8_gripper_prop.stl",
    "wx250s_9_gripper_bar.stl", "wx250s_10_gripper_finger.stl"]]


def main():
    hashes = {}
    for name in FILES:
        path = DEST / name
        url = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{REVISION}/trossen_wx250s/{name}"
        content = urllib.request.urlopen(url, timeout=45).read()
        digest = hashlib.sha256(content).hexdigest()
        if path.exists() and path.read_bytes() != content:
            raise RuntimeError(f"Refusing to replace modified upstream file: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        hashes[name] = digest
        print(f"{name}: {len(content)} bytes")
    (DEST / "provenance.json").write_text(json.dumps({
        "repository": "https://github.com/google-deepmind/mujoco_menagerie",
        "revision": REVISION, "license": "BSD-3-Clause", "sha256": hashes,
    }, indent=2) + "\n")


if __name__ == "__main__":
    main()
