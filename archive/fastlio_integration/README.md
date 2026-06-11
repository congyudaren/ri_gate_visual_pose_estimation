# Legacy FAST-LIO Integration Archive

This directory contains non-runtime reference material from the earlier combined FAST-LIO, Livox, and ROI workflow.

The current canonical ROI package is:

```text
src/roi_lidar_corner
```

Use the standalone package for active development and runtime launch:

```bash
colcon build --symlink-install --packages-select roi_lidar_corner
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py
```

The files archived here are retained only to make legacy behavior auditable and recoverable. They are not installed as part of the standalone ROI package, and they should not be treated as maintained launch entrypoints.

Archived contents:

- `launch/fastlio_with_roi.launch.py`: previous integrated launch for ROI plus FAST-LIO and optional Livox startup.
- `config/mid360_roi.yaml`: previous FAST-LIO-style experiment config.
- `scripts/`: previous workspace wrapper scripts for the integrated flow.
- `tests/`: tests for the old integrated launch and wrapper defaults.
- `docs/`: previous nested-package README files.
- `roi_lidar_corner/`: previous Python files that loaded camera extrinsics from FAST-LIO config.

The old nested package path was:

```text
src/fast_lio_lx/roi_lidar_corner
```

That path is no longer the source of truth for ROI development.
