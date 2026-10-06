from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

SCENE_NAME = "val_2"
SCENE_ID = "procthor-val-2"
SCENE_DATASET = "MolmoSpaces ProcTHOR 10K val"
SPAWN = (6.5, 13.8, 0.0)
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 270
PHYSICS_DT = 0.005


def scene_path() -> Path:
    return (
        Path(__file__).resolve().parent
        / "assets"
        / "scenes"
        / "procthor-10k-val"
        / "val_2_ceiling.xml"
    )


def _freeze_environment(root: ET.Element) -> None:
    for parent in root.iter():
        for child in tuple(parent):
            if child.tag in {"joint", "freejoint"}:
                parent.remove(child)


def _absolute_asset_paths(root: ET.Element, source: Path) -> None:
    asset_root = source.parents[2].resolve()
    for element in root.iter():
        filename = element.get("file")
        if not filename:
            continue
        resolved = (source.parent / filename).resolve()
        try:
            resolved.relative_to(asset_root)
        except ValueError as exc:
            raise RuntimeError(
                f"scene asset escapes the bundled asset root: {filename}"
            ) from exc
        if not resolved.is_file():
            raise FileNotFoundError(f"bundled scene asset is missing: {resolved}")
        element.set("file", str(resolved))


def load_scene_xml(source: Path | None = None) -> ET.Element:
    source = (source or scene_path()).resolve()
    if not source.is_file():
        raise FileNotFoundError(
            f"bundled MolmoSpaces scene is missing: {source}. "
            "Run scripts/vendor_molmospaces_scene.py during development."
        )
    root = ET.parse(source).getroot()
    _freeze_environment(root)
    _absolute_asset_paths(root, source)

    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(0, option)
    option.set("timestep", str(PHYSICS_DT))

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", str(CAMERA_WIDTH))
    global_visual.set("offheight", str(CAMERA_HEIGHT))
    return root


def compile_model(root: ET.Element) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
