# ROI Offline Analyzer Design

Date: 2026-04-24
Status: Approved for spec handoff
Scope: One-off offline analysis tool for a single object / single capture session

## Goal

Build a disposable offline analyzer for one RGB image and one LiDAR projection `NPZ` capture so we can:

- draw rectangular ROIs directly on the RGB image,
- treat multiple rectangles as one ROI union,
- delete individual rectangles from that union,
- filter points by a selectable LiDAR time window,
- inspect whether ROI sampling is stable enough for downstream frame position estimation.

The tool is explicitly not a reusable product. It is a narrow analysis surface for the current project and current data format.

## User Problem

Current whole-image diagnostics showed that some image regions have unstable per-pixel depth across a long window. That is useful for debugging, but it does not answer the more important question:

`What does the LiDAR sampling inside the actual ROI look like, and how does that change if we inspect a shorter time window?`

The analyzer must let the user define ROI regions after ROI construction logic is already complete, then inspect ROI-local point distribution, ROI-local depth distribution, and ROI-local per-pixel consistency.

## Inputs

The first version is fixed to one dataset:

- RGB base image:
  - `analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png`
- LiDAR point projection data:
  - `analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz`

The `NPZ` is expected to contain:

- `uv`: projected pixel coordinates, shape `(N, 2)`, `float32`
- `depth`: depth in camera coordinates, shape `(N,)`, `float32`
- `stamp`: point/frame timestamps in ROS seconds, shape `(N,)`, `float64`

## Fixed Assumptions

These assumptions are intentional and should be hard-coded in the first version:

- Single object only.
- Single dataset only.
- Disposable, no project persistence workflow.
- Loaded RGB image is already rotated by `180°` relative to raw capture orientation.
- The analyzer therefore must rotate projected `uv` into the same view before any ROI or statistics are computed.

Rotation rule:

- `u' = (image_width - 1) - u`
- `v' = (image_height - 1) - v`

## Out of Scope

Do not implement any of the following in the first version:

- live ROS integration,
- multi-session comparison,
- named ROI groups,
- polygon ROI editing,
- rectangle resize/drag after creation,
- undo/redo,
- long-term save/load project state,
- backend service or database,
- generic package-quality UX polish beyond what is needed for this analysis.

## Chosen Approach

Build a minimal single-page offline web tool with a tiny preprocessing step.

### Why this approach

- The task is highly visual and interaction-heavy.
- The tool should be easy to discard later.
- Rectangular ROI drawing and immediate statistical feedback are easier in a browser canvas than in a CLI or OpenCV desktop loop.
- A static page avoids framework, build, and maintenance overhead.

## High-Level Architecture

Two layers:

1. `Data preparation`
2. `Static analysis page`

### 1. Data Preparation

A lightweight Python script prepares one analysis payload from the fixed RGB + NPZ inputs.

Responsibilities:

- load RGB and NPZ,
- rotate `uv` into the current displayed RGB coordinate system,
- compute reusable point-level arrays,
- compute reusable pixel-level aggregates,
- emit one front-end-friendly data file.

The output payload should include:

- image dimensions,
- image path or embedded image reference,
- rotated point-level rows:
  - `u`, `v`, `depth`, `stamp`
- relative time metadata:
  - `stamp_min`, `stamp_max`
  - per-point `t_rel_sec = stamp - stamp_min`
- pixel-level aggregates over the full dataset:
  - `count`
  - `frame_count`
  - `depth_mean`
  - `depth_std`
  - `depth_span`

The front end may still recompute ROI-filtered pixel aggregation after time-window filtering. The preparation step is there to normalize format and remove file parsing complexity from the browser.

### 2. Static Analysis Page

A plain `HTML + CSS + JS` page renders the workspace.

No React/Vite/build chain is required.

Implementation primitives:

- one main `canvas` for image display and ROI drawing,
- small companion canvases or SVG-free custom drawing for charts,
- plain JS state store for:
  - loaded dataset,
  - current time filter,
  - current ROI rectangle list,
  - current selected rectangle.

## Workspace Layout

Chosen layout: `Focused Workspace`

### Main regions

- `Top-left`: large RGB canvas
- `Right-side`: ROI union list + delete action + compact summary
- `Bottom`: charts and derived statistics

The user already approved the layout direction:

- large drawing surface first,
- stats always visible,
- ROI union treated as one working set.

## Interaction Model

### Canvas

The main canvas shows the rotated RGB base image.

Overlay modes on the same canvas or adjacent preview panels:

- ROI rectangles,
- ROI-filtered projected points with white halo + depth color,
- optional unstable-only view.

### ROI creation

ROI creation is rectangle-only:

1. mouse down sets first corner,
2. drag previews rectangle,
3. mouse up commits rectangle.

Each committed rectangle is appended to the current ROI set.

### ROI set semantics

- The current ROI is the union of all rectangles.
- A point belongs to the active ROI if it falls inside any rectangle.

No named groups.

### ROI deletion

- ROI list shows each rectangle as one row.
- Clicking a row selects it.
- `Delete selected ROI` removes only that rectangle.

If the last rectangle is deleted, the tool returns to `no ROI` state.

### Time window control

Time filtering is applied before ROI statistics.

Controls:

- relative-time range slider over `[0, duration]`,
- quick presets:
  - `All`
  - `Last 1s`
  - `Last 3s`
  - `Last 5s`
  - `Custom`

Displayed time should be relative seconds from `stamp_min`, not raw ROS seconds.

Filtering order:

1. select points inside time window,
2. rotate coordinates already loaded,
3. apply ROI union,
4. recompute ROI-local statistics.

## Statistical Semantics

All statistics operate on the current:

- `time-filtered point set`
- further restricted by `ROI union`

### A. ROI Projection Preview

Display ROI-filtered projected points on RGB:

- darkened RGB base,
- white halo for readability,
- center color mapped by depth.

This is the default visual analysis view.

### B. ROI Depth Distribution

Show:

- histogram of `depth`,
- point count,
- depth range,
- depth quantiles:
  - `p10`
  - `p50`
  - `p90`
  - `p95`

This answers whether ROI depth is narrow, broad, or multi-modal.

### C. ROI Same-Pixel Consistency

Pixel identity is defined as:

- round rotated `u`, `v` to integer pixel indices.

For each pixel inside the ROI:

- collect all time-filtered hits,
- compute:
  - `count`
  - `frame_count`
  - `depth_mean`
  - `depth_std`
  - `depth_span`

Primary outputs:

- stable/medium/unstable pixel counts,
- fraction of multi-frame pixels that are stable,
- visual same-pixel stability overlay.

Thresholds are fixed in version 1:

- `stable < 0.05m`
- `medium >= 0.05m and < 0.20m`
- `unstable >= 0.20m`

### D. ROI Repeated-Hit Statistics

Show:

- repeated-hit count heatmap,
- number of pixels with `frame_count >= 2`,
- count histogram or summary quantiles.

### E. Unstable-Only View

Show only unstable pixels on darkened RGB.

Purpose:

- make edge / mixed-layer / occlusion boundary behavior easy to inspect.

## Visual Outputs

The first version should provide these panels:

- pure RGB base,
- ROI projection dots on RGB,
- ROI stability overlay,
- ROI depth-std heatmap,
- ROI repeated-hit count heatmap,
- ROI unstable-only view,
- depth histogram / compact numeric summary.

All dense point overlays should use the already validated readability pattern:

- white outline / halo,
- colored center,
- darkened RGB background.

Do not place large statistical text blocks on top of image panels. Use a dedicated summary area or bottom bar.

## Error Handling

This is an analysis tool, so errors should be explicit instead of over-engineered.

### Fatal load errors

Show a clear blocking error banner when:

- RGB image is missing,
- NPZ file is missing,
- required arrays are missing,
- image dimensions are invalid.

### Empty-state handling

Allow these states without error:

- no ROI rectangles yet,
- time window contains no points,
- ROI union contains no points,
- ROI contains points but no multi-frame repeated pixels.

Expected UI behavior:

- preserve the image,
- keep controls interactive,
- show `0` / `N/A` in affected statistics.

## Validation Plan

Before implementation is considered ready, verify:

1. page loads fixed RGB + NPZ successfully,
2. rotated point coordinates align with current displayed RGB orientation,
3. two-point rectangle drawing works,
4. multiple rectangles behave as one ROI union,
5. deleting one rectangle updates ROI and stats correctly,
6. changing time window updates:
   - ROI point count,
   - depth histogram,
   - same-pixel consistency values,
   - unstable-only preview,
7. a manually checked ROI matches an offline reference computation for:
   - point count,
   - `p50 depth`,
   - stable/medium/unstable counts.

## Minimal Deliverables

The implementation should produce:

- a preprocessing script,
- one static analyzer directory,
- one generated analysis payload,
- one launch/readme note describing how to open the tool locally.

Placement:

- `analysis_artifacts/roi_offline_analyzer/`

Contents:

- `index.html`
- `app.js`
- `styles.css`
- `data/session.json`
- helper preprocessing script

## Recommended Next Step

Use this spec to write a narrow implementation plan for:

1. data preparation,
2. front-end state model,
3. ROI drawing + deletion,
4. time window controls,
5. statistics and charts,
6. verification.
