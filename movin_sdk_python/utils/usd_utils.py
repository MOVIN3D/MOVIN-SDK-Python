"""USD post-processing utilities.

Provides helpers that fix or augment USD files produced by Isaac Lab's
MJCF-to-USD converter.
"""

import os
import xml.etree.ElementTree as ET


# MuJoCo's built-in geom rgba; a geom keeping it is colored by its material.
_DEFAULT_GEOM_RGBA = (0.5, 0.5, 0.5, 1.0)


def _parse_rgba(text):
    values = tuple(float(v) for v in text.split())
    return values if len(values) == 4 else None


def mjcf_mesh_colors(mjcf_path):
    """Resolve the color each mesh is drawn with, following MJCF rules.

    A geom's attributes come from its default class (``class``, else the
    nearest ancestor body's ``childclass``, else the root default), with
    nested default classes inheriting from their parents, and its own
    attributes on top.  Its color is its ``rgba`` when that differs from
    MuJoCo's default, otherwise its material's ``rgba``.  When several geoms
    draw the same mesh, a visual-only geom (``contype`` and ``conaffinity``
    0) wins over collision geoms; geoms without any color are ignored.

    Args:
        mjcf_path: Path to an MJCF file.

    Returns:
        ``{mesh_name: (r, g, b, a)}``.
    """
    root = ET.parse(mjcf_path).getroot()

    class_geom = {"main": {}}

    def walk_default(element, inherited):
        attrs = dict(inherited)
        geom = element.find("geom")
        if geom is not None:
            attrs.update(geom.attrib)
        class_geom[element.get("class", "main")] = attrs
        for child in element.findall("default"):
            walk_default(child, attrs)

    for top in root.findall("default"):
        walk_default(top, {})

    materials = {}
    for material in root.iter("material"):
        rgba = _parse_rgba(material.get("rgba", "1 1 1 1"))
        if material.get("name") and rgba is not None:
            materials[material.get("name")] = rgba

    chosen = {}  # mesh -> (priority, rgba)

    def walk_body(element, childclass):
        for geom in element.findall("geom"):
            attrs = dict(class_geom.get(geom.get("class", childclass), class_geom["main"]))
            attrs.update(geom.attrib)
            mesh = attrs.get("mesh")
            if not mesh:
                continue
            rgba = _parse_rgba(attrs["rgba"]) if "rgba" in attrs else None
            if rgba is not None and rgba != _DEFAULT_GEOM_RGBA:
                color = rgba
            elif attrs.get("material") in materials:
                color = materials[attrs["material"]]
            elif rgba is not None:
                color = rgba
            else:
                continue
            visual = attrs.get("contype") == "0" and attrs.get("conaffinity") == "0"
            priority = 2 if visual else 1
            if mesh not in chosen or priority > chosen[mesh][0]:
                chosen[mesh] = (priority, color)
        for child in element:
            if child.tag in ("body", "frame"):
                walk_body(child, child.get("childclass", childclass))

    worldbody = root.find("worldbody")
    if worldbody is not None:
        walk_body(worldbody, "main")
    return {mesh: color for mesh, (_, color) in chosen.items()}


def fix_mjcf_usd_materials(mjcf_path, usd_path):
    """Post-process a converted robot USD to fix material bindings.

    Isaac Lab's MJCF-to-USD converter creates materials from RGBA values but
    does not bind them to the correct mesh prims (everything gets a white
    ``DefaultMaterial``).  This function parses the source MJCF to extract
    per-geom RGBA colors, creates proper ``UsdPreviewSurface`` materials in
    the base USD sublayer, and rebinds each visual mesh prim accordingly.
    Colors are resolved like MuJoCo does (see ``mjcf_mesh_colors``), so
    MJCFs that color geoms through materials and default classes work too.

    Safe to call repeatedly — existing ``RobotMat_*`` materials are replaced.

    Args:
        mjcf_path: Path to the source MJCF XML file.
        usd_path: Path to the top-level ``.usd`` output from ``MjcfConverter``.
    """
    from pxr import Usd, UsdShade, UsdGeom, Sdf, Gf

    # ---- 1. Resolve mesh_name → RGBA from the MJCF ----
    mesh_to_rgba = mjcf_mesh_colors(mjcf_path)

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
