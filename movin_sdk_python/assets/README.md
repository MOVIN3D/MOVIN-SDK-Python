# MOVINMan skeleton and mesh assets

These are the assets for the skeleton presets in
`movin_sdk_python/utils/skeleton_presets.py`. The SDK loads them from this
package directory through `SkeletonPreset.mjcf_path` and
`SkeletonPreset.mesh_npz_path`, so they are installed with the package and
nothing is read from another repository at runtime.

| File | Preset | Contents |
|------|--------|----------|
| `movinman_skeleton.xml` | `movinman` | MJCF skeleton (51 bodies) |
| `movinman_v3_skeleton.xml` | `movinman_v3` | MJCF skeleton (54 bodies), generated from `MOVINman_V3_local_rotation_identity_tpose.bvh` |
| `movinman_mesh.npz` | `movinman` | LBS mesh (38,334 vertices) |
| `movinman_v3_mesh.npz` | `movinman_v3` | LBS mesh (44,402 vertices), extracted from `MOVINman_V3_local_rotation_identity.fbx` |

All four assets use the reference T-pose proportions. To match an actor's
calibrated proportions as streamed by MOVIN Studio, use
`movin_sdk_python.utils.skeleton_fitting.fit_skeleton_mjcf()` for the MJCF and
`MOVINMeshModel.fit_to_bones()` (or `pose_from_bones()`) for the mesh.

## Provenance

The files were copied unchanged from MOVIN-Isaac-Plugin `data/` at commit
`9b5de4c` (2026-09-22). That repository holds the generators: the MJCF comes
from `scripts/generate_skeleton_mjcf.py` and the NPZ from
`scripts/extract_movinman_mesh.py`, which runs in Blender.

| File | SHA-256 |
|------|---------|
| `movinman_skeleton.xml` | `e04dd0424a5d7e7b724750172e75de1cd7f8daa26c219a3f61b1628051089afc` |
| `movinman_v3_skeleton.xml` | `82e76c99ab233f1e8f3b5023d039955a4785c9d2e5d6430281a9b075e1f4dba6` |
| `movinman_mesh.npz` | `c1d67897114a47d4c662715c5b91d386e375510a843fb0c0ff95b3d8564696e6` |
| `movinman_v3_mesh.npz` | `98535bdd76712eb1bed147a6251a788538981c2cc47c07ee0fd575fe657e0abf` |
