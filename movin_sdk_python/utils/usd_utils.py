"""USD post-processing utilities.

Provides helpers that fix or augment USD files produced by Isaac Lab's
MJCF-to-USD converter.
"""

import os
import xml.etree.ElementTree as ET


def fix_mjcf_usd_materials(mjcf_path, usd_path):
    """Post-process a converted robot USD to fix material bindings.

    Isaac Lab's MJCF-to-USD converter creates materials from RGBA values but
    does not bind them to the correct mesh prims (everything gets a white
    ``DefaultMaterial``).  This function parses the source MJCF to extract
    per-geom RGBA colors, creates proper ``UsdPreviewSurface`` materials in
    the base USD sublayer, and rebinds each visual mesh prim accordingly.

    Safe to call repeatedly — existing ``RobotMat_*`` materials are replaced.

    Args:
        mjcf_path: Path to the source MJCF XML file.
        usd_path: Path to the top-level ``.usd`` output from ``MjcfConverter``.
    """
    from pxr import Usd, UsdShade, UsdGeom, Sdf, Gf

    # ---- 1. Parse MJCF for mesh_name → RGBA mapping ----
    tree = ET.parse(mjcf_path)
    mesh_to_rgba = {}
    for geom in tree.iter("geom"):
        mesh_name = geom.get("mesh")
        rgba_str = geom.get("rgba")
        if mesh_name and rgba_str:
            rgba = tuple(float(x) for x in rgba_str.split())
            mesh_to_rgba[mesh_name] = rgba

    if not mesh_to_rgba:
        return

    # Collect distinct colors → material name
    color_to_mat_name = {}
    for rgba in set(mesh_to_rgba.values()):
        r, g, b = rgba[:3]
        tag = f"{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}"
        color_to_mat_name[rgba] = f"RobotMat_{tag}"

    # ---- 2. Open the base USD sublayer (where meshes live) ----
    base_usd = os.path.join(
        os.path.dirname(usd_path), "configuration",
        os.path.splitext(os.path.basename(usd_path))[0] + "_base.usd",
    )
    if not os.path.exists(base_usd):
        print(f"[WARN] Base USD not found: {base_usd}")
        return

    stage = Usd.Stage.Open(base_usd)
    if stage is None:
        print(f"[WARN] Failed to open USD stage: {base_usd}")
        return

    # Find the root prim (e.g. /g1_mocap_29dof)
    root_prim = None
    for p in stage.GetPseudoRoot().GetChildren():
        root_prim = p
        break
    if root_prim is None:
        return

    root_path = root_prim.GetPath()
    looks_path = root_path.AppendChild("Looks")

    # ---- 3. Create materials for each distinct color ----
    mat_map = {}   # rgba → UsdShade.Material
    for rgba, mat_name in color_to_mat_name.items():
        r, g, b = rgba[:3]
        mat_path = looks_path.AppendChild(mat_name)

        # Remove existing prim if present (from a previous run)
        existing = stage.GetPrimAtPath(mat_path)
        if existing and existing.IsValid():
            stage.RemovePrim(mat_path)

        mat = UsdShade.Material.Define(stage, mat_path)
        shader_path = mat_path.AppendChild("Shader")
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(r, g, b)
        )
        # Metallic look for robot parts
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.4)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
        mat.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        mat_map[rgba] = mat

    # ---- 4. Rebind mesh prims to correct materials ----
    rebound = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        prim_name = prim.GetName()
        rgba = mesh_to_rgba.get(prim_name)
        if rgba is None:
            continue
        mat = mat_map.get(rgba)
        if mat is None:
            continue
        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(mat)
        rebound += 1

    stage.Save()
    print(f"[INFO] Fixed USD materials: {len(mat_map)} materials, "
          f"{rebound} mesh prims rebound")
