# Repository Guidelines

## Project Structure & Module Organization
The canonical ROI perception package lives in `src/roi_lidar_corner/` with Python nodes in `roi_lidar_corner/`, message definitions in `msg/`, launch files in `launch/`, configs in `config/`, models in `models/`, and tests in `tests/`. Legacy FAST-LIO/Livox integration references live under `archive/fastlio_integration/` and are not maintained runtime entrypoints. FAST-LIO and Livox are external upstream data producers if needed; they are no longer source packages in this repository.

## Build, Test, and Development Commands
Build and verify the ROI package directly:

- `colcon list --packages-select roi_lidar_corner` to confirm package discovery
- `colcon build --symlink-install --packages-select roi_lidar_corner` to build ROI only
- `ros2 launch roi_lidar_corner roi_lidar_corner.launch.py` to launch the standalone ROI flow
- `python3 -m pytest src/roi_lidar_corner/tests` for ROI solver and launch-default tests
- `python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py` for quick Python syntax verification

## Coding Style & Naming Conventions
Follow the surrounding file style instead of reformatting broadly. Python uses 4-space indentation, type hints, dataclasses, and snake_case module/function names. Launch files follow ROS naming such as `roi_lidar_corner.launch.py`; parameters and env vars stay explicit and snake_case, for example `config_file` and `ROI_LIDAR_CORNER_MAX_RANGE`.

## Testing Guidelines
Put Python tests in `src/roi_lidar_corner/tests/` and name them `test_*.py`. Prefer deterministic unit tests for solver math, message handling, and launch defaults. Hardware-dependent validation belongs in docs or handoff notes with the exact command, machine, and topics checked.

## Commit & Pull Request Guidelines
Recent history uses short scoped subjects such as `roi_lidar_corner: ...`, `fast_lio: ...`, `doc: ...`, and `gitignore: ...`. Keep commits imperative, package-scoped, and small. Pull requests should state which package was touched, note any launch/config default changes, list verification commands run, and include RViz or debug-image evidence when behavior changes are visual.
