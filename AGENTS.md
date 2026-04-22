# Repository Guidelines

## Project Structure & Module Organization
This workspace is organized around maintained shell entrypoints in `scripts/`: `scripts/env_fastlio.sh`, `scripts/env_fastlio_nx.sh`, `scripts/run_fastlio_with_roi.sh`, and `scripts/run_fastlio_with_roi_nx.sh`. Core SLAM code lives in `src/fast_lio_lx/`: C++ sources in `src/`, headers in `include/`, launch files in `launch/`, configs in `config/`, RViz assets in `rviz/`, and operational notes in `doc/`. The ROI perception layer lives under `src/fast_lio_lx/roi_lidar_corner/` with Python nodes, message definitions, launch files, models, and tests. `src/livox_ros_driver2/` is the Livox driver package; keep edits there minimal and upstream-compatible.

## Build, Test, and Development Commands
Load the intended environment before building or launching:

- `source ./scripts/env_fastlio.sh` for the dev machine
- `source ./scripts/env_fastlio_nx.sh` for NX defaults
- `colcon build --symlink-install --packages-select fast_lio livox_ros_driver2` to build the root-discovered packages
- `./scripts/run_fastlio_with_roi.sh` or `./scripts/run_fastlio_with_roi_nx.sh` to launch the maintained integrated flow
- `colcon test --packages-select fast_lio livox_ros_driver2` for package-level checks
- `pytest src/fast_lio_lx/roi_lidar_corner/tests` for ROI solver and launch-default tests
- `python3 -m py_compile src/fast_lio_lx/roi_lidar_corner/launch/*.py src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/*.py` for quick Python syntax verification

## Coding Style & Naming Conventions
Follow the surrounding file style instead of reformatting broadly. C++ targets C++17 via `ament_cmake`; keep the existing include order, brace style, and legacy identifiers in place. Python uses 4-space indentation, type hints, dataclasses, and snake_case module/function names. Launch files follow ROS naming such as `fastlio_with_roi.launch.py`; parameters and env vars stay explicit and snake_case, for example `fastlio_config_file` and `ROI_LIDAR_CORNER_MAX_RANGE`.

## Testing Guidelines
Put Python tests in `src/fast_lio_lx/roi_lidar_corner/tests/` and name them `test_*.py`. Prefer deterministic unit tests for solver math, message handling, and launch defaults. Hardware-dependent validation belongs in `src/fast_lio_lx/doc/handoffs/` with the exact command, machine, and topics checked.

## Commit & Pull Request Guidelines
Recent history uses short scoped subjects such as `roi_lidar_corner: ...`, `fast_lio: ...`, `doc: ...`, and `gitignore: ...`. Keep commits imperative, package-scoped, and small. Pull requests should state which package was touched, note any launch/config default changes, list verification commands run, and include RViz or debug-image evidence when behavior changes are visual.
