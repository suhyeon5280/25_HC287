
# Collision & STL Migration (brand new workspace)

## Summary
- Copied 14 mesh files from old workspace:
  `/mnt/data/merge_work3/old_ws/ros2_ws/src/main_description/meshes`
  → `/mnt/data/merge_work3/brand_ws/espers2_ws/src/diffbot_description/meshes/legacy_from_main_description`
- Updated URDF/Xacro mesh paths using package URI: `package://diffbot_description/meshes/legacy_from_main_description/<file>`
- Ensured `<collision>` is present by cloning `<visual>` geometry **when it was missing** (backup `.bak` made).
- Ensured `CMakeLists.txt` installs the `meshes` directory.

## Rebuild
```bash
cd espers2_ws
colcon build --symlink-install
source install/setup.bash
```

## Notes
- If some links need **different collision geometry** (simplified), edit the `<collision>` blocks manually.
- If your URDF still references old paths not covered by the mapping, search for `file:///` or `package://main_description/meshes/` and replace with the new `package://diffbot_description/meshes/legacy_from_main_description/`.
