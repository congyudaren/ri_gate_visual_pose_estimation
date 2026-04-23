# ROI Debug Decoupling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Separate ROI solver production outputs from debug UV, overlay projection, debug images, and marker visualization, while leaving a clean path for an independent long-exposure LiDAR projection capture tool.

**Architecture:** Keep `corner_lidar_solver_node.py` focused on solving and publishing `/roi_lidar_corner/corners3d` plus lightweight diagnostics. Move optional debug payload generation behind explicit flags, make debug consumers opt in, and implement long-exposure projection as a separate node that reads source topics directly instead of depending on `/roi_lidar_corner/solver_debug_uv`.

**Tech Stack:** ROS 2 Foxy, rclpy, sensor_msgs, nav_msgs, cv_bridge, OpenCV, NumPy, pytest.

---

## Current Coupling Summary

The current code couples normal solving and debug output in these places:

- `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/corner_lidar_solver_node.py` always declares, creates, computes, and publishes `/roi_lidar_corner/solver_debug_uv`.
- `corner_lidar_solver_node.py::_solve()` computes normal `Corner3DArray` output and then builds `debug_objects`, `cloud_uv`, and `cloud_uv_depth` in the same method.
- `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py` always subscribes to `/roi_lidar_corner/corners3d` and `/roi_lidar_corner/solver_debug_uv`, even when debug image output is disabled.
- `roi_generator_node.py` uses solver debug freshness to gate both projected-cloud overlay and corner 3D text.
- `src/fast_lio_lx/roi_lidar_corner/launch/fastlio_with_roi.launch.py` always starts `roi_lidar_debug_markers.py` and wires debug UV between solver and generator by default.
- `scripts/capture_roi_hit_heatmap.sh` depends on `/roi_lidar_corner/solver_debug_uv`; it cannot implement long-exposure pointcloud projection because it consumes only a debug payload, not source pointcloud and pose streams.

## File Map

Modify: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/corner_lidar_solver_node.py`

Purpose: add debug publish gates, split normal solving from optional debug payload construction, and keep diagnostics independent.

Modify: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py`

Purpose: subscribe to solver debug and corner debug streams only when debug image output needs them; separate corner 3D freshness from solver debug freshness.

Modify: `src/fast_lio_lx/roi_lidar_corner/launch/fastlio_with_roi.launch.py`

Purpose: expose debug switches and avoid unconditional debug node wiring where possible.

Modify: `src/fast_lio_lx/roi_lidar_corner/launch/roi_lidar_corner.launch.py`

Purpose: keep standalone launch behavior aligned with integrated launch.

Modify: `scripts/run_fastlio_with_roi.sh`

Modify: `scripts/run_fastlio_with_roi_nx.sh`

Purpose: pass environment-controlled debug switches through maintained wrapper entrypoints.

Modify: `src/fast_lio_lx/roi_lidar_corner/setup.py`

Purpose: register any new capture console script and remove old heatmap console entry if the legacy script is deleted.

Modify: `src/fast_lio_lx/roi_lidar_corner/CMakeLists.txt`

Purpose: install any new node script and remove legacy install references if needed.

Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_corner_lidar_solver_node.py`

Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_roi_generator_node.py`

Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_fastlio_with_roi_launch_defaults.py`

Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_default_alignment.py`

Create: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/lidar_projection_exposure_capture.py`

Purpose: independent long-exposure pointcloud-to-camera projection capture node.

Create: `scripts/capture_lidar_projection_exposure.sh`

Purpose: maintained shell wrapper for long-exposure projection capture.

Create: `src/fast_lio_lx/roi_lidar_corner/tests/test_lidar_projection_exposure_capture.py`

Purpose: deterministic tests for projection buffer, trimming, metadata, and image rendering.

---

## Task 1: Gate Solver Debug Publishing

**Files:**

- Modify: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/corner_lidar_solver_node.py`
- Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_corner_lidar_solver_node.py`

- [ ] **Step 1: Add failing test for disabled solver debug publishing**

Add a test that configures `FakeNode.parameter_overrides = {"publish_debug_uv": False}` before constructing `CornerLidarSolverNode`. Reuse existing pointcloud, odometry, camera info, and ROI helpers in `test_corner_lidar_solver_node.py`. Drive `cloud_callback()`, `odom_callback()`, `camera_info_callback()`, `image_callback()`, then `roi_callback()`. Assert:

```python
assert node.point_pub.messages
assert node.diag_pub.messages
assert node.debug_uv_pub.messages == []
```

The expected initial failure is that `debug_uv_pub.messages` is not empty because debug publishing is currently unconditional.

- [ ] **Step 2: Add failing test for normal debug publishing compatibility**

Add a test with default parameters and set `node.debug_projected_cloud_stride = 1`. Drive the same callbacks. Assert that the last debug payload still contains:

```python
debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
assert "stamp" in debug_payload
assert "objects" in debug_payload
assert "cloud_uv" in debug_payload
assert "cloud_uv_depth" in debug_payload
assert "stats" in debug_payload
```

The expected result after implementation is PASS with current payload shape preserved.

- [ ] **Step 3: Declare and read solver debug parameters**

In `CornerLidarSolverNode.__init__`, add:

```python
self.declare_parameter("publish_debug_uv", True)
self.declare_parameter("debug_overlay_frame_count", 1)
```

After existing parameter reads, add:

```python
self.publish_debug_uv = self.get_parameter("publish_debug_uv").get_parameter_value().bool_value
self.debug_overlay_frame_count = max(
    0,
    int(self.get_parameter("debug_overlay_frame_count").get_parameter_value().integer_value),
)
```

Keep `debug_projected_cloud_stride` as the point sampling stride.

- [ ] **Step 4: Split `_solve()` return contract**

Change `_solve()` from:

```python
def _solve(...) -> Tuple[Corner3DArray, Dict]:
```

to:

```python
def _solve(
    self,
    roi_msg: ObjectROIArray,
    frames: Sequence[DecodedCloudFrame],
    pose: _BufferedPose,
    *,
    build_debug_payload: bool,
) -> Tuple[Corner3DArray, Dict, Optional[Dict]]:
```

Return normal `Corner3DArray`, `stats`, and either debug payload dict or `None`.

- [ ] **Step 5: Keep diagnostics independent of debug payload**

In `roi_callback()`, call:

```python
corners, stats, debug_uv = self._solve(
    msg,
    window_frames,
    pose,
    build_debug_payload=self.publish_debug_uv,
)
self.point_pub.publish(corners)
if self.publish_debug_uv and debug_uv is not None:
    self.debug_uv_pub.publish(String(data=json.dumps(debug_uv, ensure_ascii=False)))
```

Use `stats` directly for `diag`, not `debug_uv.get("stats", {})`.

- [ ] **Step 6: Build debug objects only when needed**

In `_solve()`, keep `out.corners` construction unconditional. Build `debug_objects` and `_make_debug_corner_entry(...)` only when `build_debug_payload` is true.

Use this pattern:

```python
debug_objects = [] if build_debug_payload else None
...
if build_debug_payload:
    debug_object = {"object_id": int(obj.object_id), "corners": []}
...
if build_debug_payload and debug_objects is not None:
    debug_objects.append(debug_object)
```

Do not call `_make_debug_corner_entry()` when `build_debug_payload` is false.

- [ ] **Step 7: Gate overlay cloud projection**

Replace:

```python
overlay_frames = list(normalized_frames[:1])
```

with:

```python
overlay_frames = list(normalized_frames[: self.debug_overlay_frame_count]) if build_debug_payload else []
```

Build `cloud_uv` and `cloud_uv_depth` only when `build_debug_payload` is true.

- [ ] **Step 8: Run solver tests**

Run:

```bash
pytest src/fast_lio_lx/roi_lidar_corner/tests/test_corner_lidar_solver_node.py -q
```

Expected: all tests pass, including the new disabled-debug test and compatibility payload test.

- [ ] **Step 9: Commit**

```bash
git add src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/corner_lidar_solver_node.py src/fast_lio_lx/roi_lidar_corner/tests/test_corner_lidar_solver_node.py
git commit -m "roi_lidar_corner: gate solver debug uv publishing"
```

---

## Task 2: Decouple ROI Generator Debug Consumers

**Files:**

- Modify: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py`
- Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_roi_generator_node.py`

- [ ] **Step 1: Add failing test for no debug subscriptions when debug image is disabled**

Extend the fake node in `test_roi_generator_node.py` so it records created subscriptions. Configure:

```python
FakeNode.parameter_overrides = {"publish_debug_image": False}
node = module.RoiGeneratorNode()
```

Assert that the node still subscribes to the image topic for ROI generation but does not subscribe to `corner3d_topic` or `solver_debug_uv_topic`.

Expected initial failure: the current node always creates both debug subscriptions.

- [ ] **Step 2: Add failing test for independent freshness gates**

Instantiate `RoiGeneratorNode`, set `latest_corner3d` with one valid corner, set `latest_corner3d_stamp` equal to the image stamp, set `latest_solver_debug_uv_stamp = None`, and call the drawing path with a fake image. Assert that detail text uses real corner coordinates instead of `xyz=(?, ?, ?)`.

Expected initial failure: current code gates corner text with solver debug freshness.

- [ ] **Step 3: Add debug subscription parameters**

In `RoiGeneratorNode.__init__`, declare:

```python
self.declare_parameter("subscribe_corner3d_debug", True)
self.declare_parameter("subscribe_solver_debug_uv", True)
```

Read:

```python
self.subscribe_corner3d_debug = self.get_parameter("subscribe_corner3d_debug").get_parameter_value().bool_value
self.subscribe_solver_debug_uv = self.get_parameter("subscribe_solver_debug_uv").get_parameter_value().bool_value
```

- [ ] **Step 4: Gate debug subscriptions**

Replace unconditional debug subscriptions with:

```python
self.corner3d_sub = None
self.solver_debug_uv_sub = None
if self.publish_debug_image and self.subscribe_corner3d_debug:
    self.corner3d_sub = self.create_subscription(Corner3DArray, self.corner3d_topic, self.corner3d_callback, 10)
if self.publish_debug_image and self.subscribe_solver_debug_uv:
    self.solver_debug_uv_sub = self.create_subscription(String, self.solver_debug_uv_topic, self.solver_debug_uv_callback, 10)
```

Keep ROI image subscription and ROI publisher unconditional.

- [ ] **Step 5: Split freshness checks**

Replace `_is_debug_data_fresh()` with:

```python
def _image_stamp(self, image_msg: Image) -> float:
    return float(image_msg.header.stamp.sec) + float(image_msg.header.stamp.nanosec) * 1e-9

def _is_solver_debug_fresh(self, image_msg: Image) -> bool:
    if self.latest_solver_debug_uv_stamp is None:
        return False
    return abs(self.latest_solver_debug_uv_stamp - self._image_stamp(image_msg)) <= 0.2

def _is_corner3d_fresh(self, image_msg: Image) -> bool:
    if self.latest_corner3d_stamp is None:
        return False
    return abs(self.latest_corner3d_stamp - self._image_stamp(image_msg)) <= 0.2
```

- [ ] **Step 6: Draw corner text and solver UV independently**

In `image_callback()`, compute:

```python
draw_solver_debug = self._is_solver_debug_fresh(msg)
draw_corner3d_debug = self._is_corner3d_fresh(msg)
```

Call:

```python
self._draw_corner_rois(debug, objects, draw_solver_debug, draw_corner3d_debug)
```

Change `_draw_corner_rois()` signature to accept both booleans. Use `draw_corner3d_debug` for `latest_corner3d` text and `draw_solver_debug` for `latest_solver_debug_uv` support point overlay.

- [ ] **Step 7: Run generator tests**

Run:

```bash
pytest src/fast_lio_lx/roi_lidar_corner/tests/test_roi_generator_node.py -q
```

Expected: all generator tests pass.

- [ ] **Step 8: Commit**

```bash
git add src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py src/fast_lio_lx/roi_lidar_corner/tests/test_roi_generator_node.py
git commit -m "roi_lidar_corner: make roi debug consumers optional"
```

---

## Task 3: Launch and Wrapper Debug Switches

**Files:**

- Modify: `src/fast_lio_lx/roi_lidar_corner/launch/fastlio_with_roi.launch.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/launch/roi_lidar_corner.launch.py`
- Modify: `scripts/run_fastlio_with_roi.sh`
- Modify: `scripts/run_fastlio_with_roi_nx.sh`
- Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_fastlio_with_roi_launch_defaults.py`
- Test: `src/fast_lio_lx/roi_lidar_corner/tests/test_default_alignment.py`

- [ ] **Step 1: Add failing launch test for new debug args**

In `test_fastlio_with_roi_launch_defaults.py`, update default assertions:

```python
assert defaults["publish_debug_uv"] == "true"
assert defaults["debug_overlay_frame_count"] == "1"
assert defaults["subscribe_corner3d_debug"] == "true"
assert defaults["subscribe_solver_debug_uv"] == "true"
assert defaults["enable_debug_markers"] == "true"
```

Expected initial failure: these args do not exist.

- [ ] **Step 2: Add failing launch wiring test**

Extend `test_launch_wires_current_generator_and_solver_parameters()`:

```python
assert solver_params["publish_debug_uv"].name == "publish_debug_uv"
assert solver_params["debug_overlay_frame_count"].name == "debug_overlay_frame_count"
assert generator_params["subscribe_corner3d_debug"].name == "subscribe_corner3d_debug"
assert generator_params["subscribe_solver_debug_uv"].name == "subscribe_solver_debug_uv"
```

Expected initial failure: params are not wired.

- [ ] **Step 3: Add integrated launch args and wiring**

In `fastlio_with_roi.launch.py`, add `LaunchConfiguration` variables and `DeclareLaunchArgument` entries:

```python
publish_debug_uv = LaunchConfiguration("publish_debug_uv")
debug_overlay_frame_count = LaunchConfiguration("debug_overlay_frame_count")
subscribe_corner3d_debug = LaunchConfiguration("subscribe_corner3d_debug")
subscribe_solver_debug_uv = LaunchConfiguration("subscribe_solver_debug_uv")
enable_debug_markers = LaunchConfiguration("enable_debug_markers")
```

Defaults:

```python
DeclareLaunchArgument("publish_debug_uv", default_value="true")
DeclareLaunchArgument("debug_overlay_frame_count", default_value="1")
DeclareLaunchArgument("subscribe_corner3d_debug", default_value="true")
DeclareLaunchArgument("subscribe_solver_debug_uv", default_value="true")
DeclareLaunchArgument("enable_debug_markers", default_value="true")
```

Pass new params into generator and solver nodes.

- [ ] **Step 4: Gate debug marker node**

Add:

```python
condition=IfCondition(enable_debug_markers)
```

to the `debug_markers = Node(...)` action.

- [ ] **Step 5: Mirror launch args in standalone launch**

Apply equivalent arguments and node parameters to `roi_lidar_corner.launch.py` so standalone and integrated launches stay aligned.

- [ ] **Step 6: Add wrapper env vars**

In both wrapper scripts, pass:

```bash
"publish_debug_uv:=${ROI_LIDAR_CORNER_PUBLISH_DEBUG_UV:-true}"
"debug_overlay_frame_count:=${ROI_LIDAR_CORNER_DEBUG_OVERLAY_FRAME_COUNT:-1}"
"subscribe_corner3d_debug:=${ROI_LIDAR_CORNER_SUBSCRIBE_CORNER3D_DEBUG:-true}"
"subscribe_solver_debug_uv:=${ROI_LIDAR_CORNER_SUBSCRIBE_SOLVER_DEBUG_UV:-true}"
"enable_debug_markers:=${ROI_LIDAR_CORNER_ENABLE_DEBUG_MARKERS:-true}"
```

- [ ] **Step 7: Run launch and wrapper tests**

Run:

```bash
pytest src/fast_lio_lx/roi_lidar_corner/tests/test_fastlio_with_roi_launch_defaults.py src/fast_lio_lx/roi_lidar_corner/tests/test_default_alignment.py src/fast_lio_lx/roi_lidar_corner/tests/test_wrapper_script_defaults.py -q
```

Expected: all tests pass with current default behavior preserved.

- [ ] **Step 8: Commit**

```bash
git add src/fast_lio_lx/roi_lidar_corner/launch/fastlio_with_roi.launch.py src/fast_lio_lx/roi_lidar_corner/launch/roi_lidar_corner.launch.py scripts/run_fastlio_with_roi.sh scripts/run_fastlio_with_roi_nx.sh src/fast_lio_lx/roi_lidar_corner/tests/test_fastlio_with_roi_launch_defaults.py src/fast_lio_lx/roi_lidar_corner/tests/test_default_alignment.py src/fast_lio_lx/roi_lidar_corner/tests/test_wrapper_script_defaults.py
git commit -m "roi_lidar_corner: expose debug pipeline switches"
```

---

## Task 4: Remove or Deprecate Legacy Heatmap Capture

**Files:**

- Modify or delete: `scripts/capture_roi_hit_heatmap.sh`
- Modify or delete: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_hit_heatmap_capture.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/setup.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/tests/test_roi_hit_heatmap_capture.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/README.md`
- Modify: `src/fast_lio_lx/roi_lidar_corner/README.zh-CN.md`

- [ ] **Step 1: Choose deletion or deprecation**

Use deletion if no one needs the old single-best-frame heatmap report. Use deprecation if existing field workflow still references it.

Recommended choice for this repository state: delete the wrapper and Python module after the long-exposure capture script exists.

- [ ] **Step 2: If deleting, remove the wrapper and module**

Delete:

```text
scripts/capture_roi_hit_heatmap.sh
src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_hit_heatmap_capture.py
src/fast_lio_lx/roi_lidar_corner/tests/test_roi_hit_heatmap_capture.py
```

Remove this console entry from `setup.py`:

```python
"roi_hit_heatmap_capture = roi_lidar_corner.roi_hit_heatmap_capture:main",
```

- [ ] **Step 3: If deprecating, make the deprecation explicit**

Add this note to both READMEs:

```markdown
`scripts/capture_roi_hit_heatmap.sh` is a single-best-frame debug report tool. It does not accumulate pointcloud projections over time. Use `scripts/capture_lidar_projection_exposure.sh` for long-exposure pointcloud projection capture.
```

- [ ] **Step 4: Verify references**

Run:

```bash
rg -n "capture_roi_hit_heatmap|roi_hit_heatmap_capture" scripts src/fast_lio_lx/roi_lidar_corner
```

Expected if deleting: no matches.

Expected if deprecating: only README and intentional compatibility tests match.

- [ ] **Step 5: Commit**

Deletion commit:

```bash
git add -A scripts/capture_roi_hit_heatmap.sh src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/roi_hit_heatmap_capture.py src/fast_lio_lx/roi_lidar_corner/tests/test_roi_hit_heatmap_capture.py src/fast_lio_lx/roi_lidar_corner/setup.py src/fast_lio_lx/roi_lidar_corner/README.md src/fast_lio_lx/roi_lidar_corner/README.zh-CN.md
git commit -m "roi_lidar_corner: remove legacy heatmap capture"
```

Deprecation commit:

```bash
git add src/fast_lio_lx/roi_lidar_corner/README.md src/fast_lio_lx/roi_lidar_corner/README.zh-CN.md
git commit -m "doc: clarify roi heatmap capture limitations"
```

---

## Task 5: Add Independent Long-Exposure Projection Capture

**Files:**

- Create: `src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/lidar_projection_exposure_capture.py`
- Create: `scripts/capture_lidar_projection_exposure.sh`
- Create: `src/fast_lio_lx/roi_lidar_corner/tests/test_lidar_projection_exposure_capture.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/CMakeLists.txt`
- Modify: `src/fast_lio_lx/roi_lidar_corner/setup.py`
- Modify: `src/fast_lio_lx/roi_lidar_corner/README.md`
- Modify: `src/fast_lio_lx/roi_lidar_corner/README.zh-CN.md`

- [ ] **Step 1: Add projection buffer unit tests**

Create tests for a pure helper class `ProjectionExposureBuffer`:

```python
buffer = ProjectionExposureBuffer(window_sec=2.0)
buffer.add_points(stamp=10.0, uv=np.asarray([[1, 2], [3, 4]], dtype=np.float32), depth=np.asarray([5.0, 6.0], dtype=np.float32))
buffer.add_points(stamp=12.5, uv=np.asarray([[7, 8]], dtype=np.float32), depth=np.asarray([9.0], dtype=np.float32))
buffer.trim(newest_stamp=12.5)
assert buffer.points_uv.tolist() == [[7.0, 8.0]]
assert buffer.depth.tolist() == [9.0]
```

Expected initial failure: class does not exist.

- [ ] **Step 2: Implement `ProjectionExposureBuffer`**

In `lidar_projection_exposure_capture.py`, implement:

```python
class ProjectionExposureBuffer:
    def __init__(self, window_sec: float) -> None:
        self.window_sec = float(window_sec)
        self._items: list[tuple[float, np.ndarray, np.ndarray]] = []

    def add_points(self, *, stamp: float, uv: np.ndarray, depth: np.ndarray) -> None:
        uv = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
        depth = np.asarray(depth, dtype=np.float32).reshape(-1)
        if uv.shape[0] != depth.shape[0]:
            raise ValueError("uv and depth must have matching row counts")
        self._items.append((float(stamp), uv, depth))
        self.trim(newest_stamp=float(stamp))

    def trim(self, *, newest_stamp: float) -> None:
        lower = float(newest_stamp) - self.window_sec
        self._items = [item for item in self._items if item[0] >= lower]

    @property
    def points_uv(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0, 2), dtype=np.float32)
        return np.vstack([item[1] for item in self._items]).astype(np.float32, copy=False)

    @property
    def depth(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0,), dtype=np.float32)
        return np.concatenate([item[2] for item in self._items]).astype(np.float32, copy=False)
```

- [ ] **Step 3: Add projection helper tests**

Test a pure helper `project_points_to_image(...)` using a small point array, identity pose, simple intrinsics, and image bounds. Assert that points behind camera and outside image are dropped.

Expected initial failure: helper does not exist.

- [ ] **Step 4: Implement projection helper by reusing solver math**

Import and reuse:

```python
from roi_lidar_corner.corner_lidar_solver_node import _load_fastlio_camera_offsets, _to_np_points, _stamp_to_sec, _quat_to_matrix
from roi_lidar_corner.lookback_solver import DecodedCloudFrame, _project_cached_frame, prepare_cached_points
```

Implement `project_points_to_image(...)` as a thin wrapper around `_project_cached_frame()` so projection behavior matches solver behavior.

- [ ] **Step 5: Implement capture node**

Create `LidarProjectionExposureCaptureNode` that declares:

```python
duration_sec
window_sec
output_image
output_json
image_topic
pointcloud_topic
odom_topic
camera_info_topic
fastlio_config_path
fastlio_config_file
min_range
max_range
cache_voxel_size
cloud_frame_mode
point_stride
```

Subscriptions:

```python
Image -> latest background image
CameraInfo -> intrinsics
Odometry -> odom buffer
PointCloud2 -> project and append to ProjectionExposureBuffer
```

Do not subscribe to `/roi_lidar_corner/solver_debug_uv`.

- [ ] **Step 6: Implement output rendering**

Render the latest camera image as background and draw accumulated UV points using depth colormap. Write:

```text
output_image: PNG overlay
output_json: metadata with duration_sec, window_sec, point_count, image_topic, pointcloud_topic, odom_topic, camera_info_topic
```

If no image is available, create a black canvas using camera info width and height. If no points were accumulated, still write metadata and an empty overlay image.

- [ ] **Step 7: Add wrapper script**

Create `scripts/capture_lidar_projection_exposure.sh` mirroring existing wrapper style:

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROI_PYTHON_ROOT="${WORKSPACE_ROOT}/src/fast_lio_lx/roi_lidar_corner"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"

if [[ -f "${ROS_SETUP}" ]]; then
    set +u
    source "${ROS_SETUP}"
    set -u
fi

if [[ -f "${WORKSPACE_SETUP}" ]]; then
    set +u
    source "${WORKSPACE_SETUP}"
    set -u
fi

if python3 -c "import roi_lidar_corner.lidar_projection_exposure_capture" >/dev/null 2>&1; then
    python3 -m roi_lidar_corner.lidar_projection_exposure_capture "$@"
else
    export PYTHONPATH="${ROI_PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
    python3 -m roi_lidar_corner.lidar_projection_exposure_capture "$@"
fi
```

- [ ] **Step 8: Register install hooks**

In `CMakeLists.txt`, add:

```cmake
roi_lidar_corner/lidar_projection_exposure_capture.py
```

to the `install(PROGRAMS ...)` block.

In `setup.py`, add console script:

```python
"lidar_projection_exposure_capture = roi_lidar_corner.lidar_projection_exposure_capture:main",
```

- [ ] **Step 9: Document usage**

Add to both READMEs:

```bash
./scripts/capture_lidar_projection_exposure.sh \
  --duration-sec 20 \
  --window-sec 20 \
  --output-image /tmp/lidar_projection_exposure.png \
  --output-json /tmp/lidar_projection_exposure.json \
  --image-topic /camera/color/image_raw \
  --pointcloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --camera-info-topic /camera/color/camera_info
```

Explain that this is a long-exposure projection capture and does not depend on ROI solver debug topics.

- [ ] **Step 10: Run capture tests**

Run:

```bash
pytest src/fast_lio_lx/roi_lidar_corner/tests/test_lidar_projection_exposure_capture.py -q
```

Expected: all exposure capture tests pass.

- [ ] **Step 11: Commit**

```bash
git add src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/lidar_projection_exposure_capture.py scripts/capture_lidar_projection_exposure.sh src/fast_lio_lx/roi_lidar_corner/tests/test_lidar_projection_exposure_capture.py src/fast_lio_lx/roi_lidar_corner/CMakeLists.txt src/fast_lio_lx/roi_lidar_corner/setup.py src/fast_lio_lx/roi_lidar_corner/README.md src/fast_lio_lx/roi_lidar_corner/README.zh-CN.md
git commit -m "roi_lidar_corner: add lidar projection exposure capture"
```

---

## Final Verification

- [ ] **Step 1: Python syntax check**

Run:

```bash
python3 -m py_compile src/fast_lio_lx/roi_lidar_corner/roi_lidar_corner/*.py
```

Expected: command exits 0.

- [ ] **Step 2: Shell syntax check**

Run:

```bash
bash -n scripts/run_fastlio_with_roi.sh scripts/run_fastlio_with_roi_nx.sh scripts/capture_lidar_projection_exposure.sh
```

Expected: command exits 0.

- [ ] **Step 3: Full ROI Python test suite**

Run:

```bash
pytest src/fast_lio_lx/roi_lidar_corner/tests -q
```

Expected: all tests pass.

- [ ] **Step 4: Manual smoke on live system**

Start integrated flow:

```bash
source ./scripts/env_fastlio.sh
./scripts/run_fastlio_with_roi.sh
```

In another shell, capture long-exposure projection:

```bash
./scripts/capture_lidar_projection_exposure.sh \
  --duration-sec 10 \
  --window-sec 10 \
  --output-image /tmp/lidar_projection_exposure.png \
  --output-json /tmp/lidar_projection_exposure.json
```

Expected:

- `/tmp/lidar_projection_exposure.png` exists and contains accumulated projected pointcloud overlay.
- `/tmp/lidar_projection_exposure.json` reports nonzero `point_count` when `/cloud_registered`, `/Odometry`, camera info, and image topics are active.
- Disabling solver debug with `ROI_LIDAR_CORNER_PUBLISH_DEBUG_UV=false` does not prevent `/roi_lidar_corner/corners3d` from publishing.

---

## Implementation Notes

- Do not move normal 3D corner solving into the new exposure capture script.
- Do not make the exposure capture script depend on `/roi_lidar_corner/solver_debug_uv`.
- Keep default launch behavior compatible unless explicitly changed by env var or launch arg.
- Treat `solver_diag` as lightweight operational telemetry; it can remain in the solver as long as it is not derived from the debug payload.
- The long-exposure capture should project source pointcloud using source pose and camera calibration, not reuse already downsampled debug `cloud_uv`.
