# MOVINMan skeleton and mesh assets

These are the assets for the skeleton presets in
`movin_sdk_python/utils/skeleton_presets.py`. The SDK loads them from this
package directory through `SkeletonPreset.mjcf_path` and
`SkeletonPreset.mesh_npz_path` and are installed with the package.

| File | Preset | Contents |
|------|--------|----------|
| `movinman_skeleton.xml` | `movinman` | MJCF skeleton (51 bodies) |
| `movinman_v3_skeleton.xml` | `movinman_v3` | MJCF skeleton (54 bodies) |
| `movinman_mesh.npz` | `movinman` | LBS mesh (38,334 vertices) |
| `movinman_v3_mesh.npz` | `movinman_v3` | LBS mesh (44,402 vertices) |

All four assets use the reference T-pose proportions. To match an actor's
calibrated proportions as streamed by MOVIN Studio, use
`movin_sdk_python.utils.skeleton_fitting.fit_skeleton_mjcf()` for the MJCF and
`MOVINMeshModel.fit_to_bones()` (or `pose_from_bones()`) for the mesh.
