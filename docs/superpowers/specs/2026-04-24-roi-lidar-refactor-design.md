# ROI LiDAR Refactor Design

Date: 2026-04-24
Status: Draft for user review
Scope: Refactor ROI and LiDAR corner recovery so ROI provides structure masks and LiDAR reconstructs the front-face corners online

## Goal

Refactor the current ROI-plus-corner pipeline into a structure-first pipeline:

- ROI keeps the existing stable image-side corner extraction path, but only uses it to derive structure masks.
- LiDAR becomes responsible for online recovery of the front-face geometry.
- The main output becomes the four front-face corners in the camera frame.
- Debug output still exposes the two directly solved top corners.

The intended result is a system that matches the real sensing situation:

- the target is a single known frame structure,
- the front face is approximately facing the sensor,
- the frame geometry is known up to small placement error,
- LiDAR support is sparse and uneven,
- top-beam visibility may fail after initialization,
- tracking matters more than one-shot batch solving.

## Context

The current implementation mixes these responsibilities:

- image-side corner finding,
- corner ROI generation,
- lookback point accumulation,
- direct per-corner support aggregation,
- final 3D corner publication.

That design no longer matches the desired method.

The target is a single frame object built from two stacked `1m x 1m x 1m` frame sections. For the front face:

- total physical height is `2m`,
- front-face width is fixed by the known frame geometry,
- the front face is still treated as approximately facing the sensor,
- small pose error is acceptable,
- the camera is physically inverted relative to image intuition,
- the camera/LiDAR extrinsics are already aligned for the current installation.

Image-side structure semantics must therefore follow physical meaning instead of image-side meaning:

- `top_beam` means the physical top beam of the front face,
- `left_post` and `right_post` mean the physical left and right posts,
- these physical semantics are not the same as raw image-side top/left/right because the camera is inverted.

The scene assumption is also intentionally narrow:

- only one frame target is present,
- no general multi-object tracking is required,
- the module may keep tracking over time, but it only needs to maintain a single target state.

## In Scope

- Keep the stable ROI corner extraction logic as the image-side geometric prior.
- Degrade ROI output from `corner_rois` to `structure_rois`.
- Derive `left_post`, `right_post`, and `top_beam` masks from the recovered image corners.
- Add a LiDAR data-processing layer that only performs point filtering and point-set cleanup.
- Add an online structure-tracking layer for the three physical structures.
- Add a restoration layer that outputs the four front-face corners in the camera frame.
- Add debug output for the two directly solved top corners.
- Support offline verification without live camera or LiDAR topics.

## Out of Scope

- No full-object 6DoF pose solver.
- No general multi-target association or generic `track_id` infrastructure.
- No output of all 12 full-frame corners in this layer.
- No publication of a full 3D frame mesh or explicit edge list as the main API.
- No dependence on live synchronized camera/LiDAR capture for method validation.
- No rewrite of the existing stable image-side corner detector.
- No new requirement that ROI produce exact per-pixel semantic segmentation of the three structures.

## Chosen Approach

Use a three-stage structure-first design:

1. ROI derives `left_post`, `right_post`, and `top_beam` masks from the existing image-side corner result.
2. LiDAR data processing projects and filters points into those structure masks.
3. LiDAR tracking and restoration update three structure states online and reconstruct the front-face corners from those states every frame.

### Why this approach

- It preserves the currently stable image-side part instead of replacing it.
- It moves geometric responsibility to the LiDAR side where the user wants it.
- It matches the actual observables: sparse support on two posts plus an optionally visible top beam.
- It treats tracking as a first-class concern instead of a side effect of a batch lookback window.
- It keeps the main API small while still exposing enough debug detail to evaluate the method.

## ROI Layer

### ROI responsibility

ROI is responsible for:

- detecting the single frame target in the image,
- recovering the image-side four corners using the existing stable logic,
- remapping those corners into physical structure semantics,
- deriving three structure masks from those corners.

ROI is not responsible for:

- publishing corner semantics as the main output,
- deciding final 3D corner positions,
- doing temporal tracking,
- using LiDAR information to refine the image result.

### ROI internal method

The chosen method is intentionally a degradation of the current stable corner path, not a replacement:

1. Keep the current `bbox -> 4 image corners` logic.
2. Reinterpret those corners using physical frame semantics.
3. Connect the relevant corner pairs into three physical structure lines.
4. Dilate each line in its normal direction to form a structure ROI mask.

This keeps the stable image-side part while changing only the semantic role of the output.

### Physical semantic remapping

If the existing corner order is image-semantic `TL / TR / BL / BR`, then for the current inverted camera setup the physical structure mapping is:

- `top_beam = BL -> BR`
- `left_post = TR -> BR`
- `right_post = TL -> BL`

This mapping is required so that all later stages use physical semantics consistently.

### ROI output

The ROI-to-LiDAR contract should be replaced cleanly with structure semantics. The new structure object should include:

- target `bbox`,
- `left_post_mask`,
- `right_post_mask`,
- `top_beam_mask`,
- per-structure `valid`,
- optional structure line endpoints for debug and visualization,
- source/debug metadata such as whether the line came from refined corners or from fallback corners.

The old corner-ROI interface should not be kept in parallel.

### ROI validity rule

ROI should be permissive.

If the target `bbox` exists and the corner pipeline produces a usable geometry, the three structure masks should be emitted even if they are coarse. The downstream LiDAR stages are responsible for deciding whether enough support exists to recover corners.

## LiDAR Data-Processing Layer

### Responsibility

The data-processing layer only filters points. It does not recover geometry and it does not output corners.

Its job is to convert projected point evidence into cleaner per-structure point sets:

- `left_post_points_filtered`
- `right_post_points_filtered`
- `top_beam_points_filtered`

### Input

Per frame, the layer consumes:

- current structure masks from ROI,
- projected point evidence in image coordinates,
- depth values,
- timestamps.

The data-processing layer may use a short lookback window, but that window serves point evidence collection only.

### Method

For each structure independently:

1. collect raw projected hits inside the structure mask,
2. estimate the near front-surface depth peak,
3. remove background points,
4. remove points likely belonging to the rear bar,
5. keep the cleaned point set plus diagnostic counters.

The target geometry provides the key depth prior:

- the frame is built from `1m` members,
- the rear structure should appear approximately `1m` behind the front structure,
- rear-bar rejection should therefore use a `1.0m +/- tolerance` prior rather than a generic clustering method.

### Point-count assumption

The expected filtered support is sparse even after cleanup. A representative `1s` window may contain only approximately:

- `26+` points on the left post,
- `26+` points on the right post,
- `14+` points on the top beam.

Because of that, this layer must stay conservative:

- no aggressive geometric sub-clustering,
- no repeated hard pruning after the depth split,
- no line fitting here,
- no final semantic decision here.

The restoration layer needs those remaining points to preserve robustness.

## LiDAR Restoration Architecture

### Top-level structure

The LiDAR recovery path is split into two internal layers:

1. structure tracking,
2. front-face restoration.

These are internal responsibilities inside the same logical solver. They should not be separate ROS nodes in the first implementation.

### Why online tracking instead of batch solving

The solver should not wait until enough data has been collected and then run the whole stack as a one-shot three-layer solve.

Instead:

- every incoming frame updates the state of `left_post`, `right_post`, and `top_beam`,
- those three evolving structure states serve the final restoration layer,
- the lookback window exists as short-term memory for observations, not as a batch replay buffer.

This matches the actual requirement better because tracking is a core problem, not an afterthought.

## Single-Target Tracking Model

Because only one target frame is present in the intended scene, the solver maintains exactly one `FaceTrack`.

No generic multi-target association is required.

`FaceTrack` contains:

- `left_post_state`
- `right_post_state`
- `top_beam_state`
- the current tracked front-face output

The current frame's observations always update this single object.

## Per-Frame Observations

The word `observation` in this design means a single-frame structure observation generated by the restoration layer from the filtered point sets.

It does not mean the raw point set itself and it does not mean the long-term state.

### Post observation

For each post:

- `support_count`
- `x_obs`
- `z_obs`
- `y_visible_min`
- `y_visible_max`
- `x_dispersion`
- `z_dispersion`
- `front_peak_confidence`
- `top_side_sample_present`

The top-side sample is a weak clue collected from the physical top side of the post. It does not itself define the final top edge in that frame.

### Top-beam observation

For the top beam:

- `support_count`
- `y_top_obs`
- `z_obs`
- `x_span`
- `z_dispersion`
- `front_peak_confidence`

### Observation states

Each single-frame observation is classified as:

- `observed`
- `weak`
- `missing`

For posts:

- `observed` means the point count, depth peak, and `x/z` concentration are good enough to trust `x_obs` and `z_obs`,
- `weak` means some support exists but should only weakly influence tracking,
- `missing` means the frame should not update that structure's geometry.

For the top beam:

- `observed` means point count, span, and front-depth consistency are sufficient to use it as a top-edge anchor,
- `weak` means it may help maintain tracking but should not initialize the model,
- `missing` means no usable top-beam structure exists in this frame.

## Structure State Update

### General rule

Each structure state keeps a deque of recent observations covering the last `1s`.

That deque stores single-frame structure observations, not a replay buffer of raw points.

### Post state contents

Each post state stores:

- recent observation deque,
- `x_state`,
- `z_state`,
- `y_top_candidate_state`,
- `confidence`,
- `freshness`,
- `lost_age`,
- `initialized`,
- `top_initialized`

`y_top_candidate_state` is the tracked post-side top clue. It is not a direct top-edge measurement.

### Top-beam state contents

`top_beam_state` stores:

- recent observation deque,
- `y_top_state`,
- `z_state`,
- `x_span_state`,
- `confidence`,
- `freshness`,
- `lost_age`,
- `initialized`

### Update behavior by observation state

If a structure observation is `observed`:

- append it to the deque,
- update geometry with high weight,
- reset `lost_age`,
- refresh `freshness`,
- raise `confidence`.

If it is `weak`:

- append it to the deque,
- allow only a small state correction,
- do not use it for first initialization,
- only partially recover freshness/confidence.

If it is `missing`:

- do not update geometry,
- decay freshness and confidence,
- increase `lost_age`.

### State estimation from the deque

For each post:

- `x_state` and `z_state` come from a weighted robust center over recent `x_obs` and `z_obs`,
- `observed` entries get full weight,
- `weak` entries get reduced weight,
- `missing` entries do not contribute.

The first implementation should prefer a weighted robust center such as a weighted median instead of a parametric filter.

For the top beam:

- `y_top_state` comes from the weighted robust center of `y_top_obs`,
- `z_state` and `x_span_state` are estimated the same way from recent valid beam observations.

### Top-side sample handling

A single frame may contain too little post-top evidence to define a stable top point. Therefore:

- a single-frame post top-side sample is only treated as a weak clue,
- `y_top_candidate_state` is estimated over time from recent top-side samples,
- this state is used to maintain tracking when the top beam is temporarily unavailable,
- it should not by itself serve as the first valid top anchor for model initialization.

## Model Initialization Rule

The full front-face model may only be initialized after at least one frame has produced a valid `top_beam_state` initialization.

After that first successful top-beam initialization:

- the top beam may fail in later frames,
- the model may continue in `tracking` using left and right post states plus historical top information.

This rule prevents the solver from fabricating an initial top edge out of sparse single-post evidence.

## Front-Face Restoration

### Main restoration inputs

The restoration layer reads:

- `left_post_state`
- `right_post_state`
- `top_beam_state`
- tracked historical top information
- fixed geometry priors

The fixed geometry priors are:

- front-face width from the known frame,
- front-face height `H = 2m`.

### Solved state

The restoration layer works with a low-degree state:

- `x_left`
- `z_left`
- `x_right`
- `z_right`
- `y_top`

This preserves the simplified front-facing assumption while still allowing the left and right posts to live at different depths.

### Top source priority

The final `y_top` should be chosen by priority:

1. `top_beam_state` if valid,
2. fused post top candidates if both posts have usable top-side history,
3. the previously tracked top value.

This makes the top beam a strong top anchor, but not a permanently required one.

### Corner reconstruction

The front-face top corners are reconstructed first:

- `top_left = (x_left, y_top, z_left)`
- `top_right = (x_right, y_top, z_right)`

Then the physical height prior closes the front face:

- `bottom_left = top_left + height_prior`
- `bottom_right = top_right + height_prior`

The sign convention of the height offset should follow the existing camera-frame axis convention in the implementation, but the semantic meaning remains:

- `top_left`
- `top_right`
- `bottom_left`
- `bottom_right`

These four corners are the only main geometric output of this layer.

## Main Output and Debug Output

### Main output

The main result of this layer is:

- the four front-face corners in the camera frame,
- ordered by physical semantics:
  - `top_left`
  - `top_right`
  - `bottom_left`
  - `bottom_right`

The main API should not publish the full face model, explicit edges, or the 2D image bbox.

### Debug output

For debugging, the layer should also publish:

- the two directly solved top corners,
- their source and state,
- tracking diagnostics.

This preserves visibility into which corners are directly supported versus inferred from geometry priors.

### Not yet in scope

Although downstream logic may eventually need 12 corners for the full structure, this layer stops at:

- 4 front-face corners as the main output,
- 2 directly solved top corners as debug output.

The expansion to 12 corners should be treated as a later stage.

## Corner Status Semantics

For diagnostics, each corner should carry an internal semantic state:

- `observed`
- `inferred`
- `invalid`

Expected behavior:

- top corners are often `observed` when the top edge is directly supported,
- top corners become `inferred` when the top edge comes from post-side history or tracked state,
- bottom corners are usually `inferred` because they rely on the `2m` height prior,
- corners are `invalid` when the model cannot be trusted.

## Solution State

The outward-facing solution state is simplified to:

- `tracking`
- `invalid`
- `lost`

### tracking

`tracking` means:

- the current model is usable,
- the current output may combine fresh observations with tracked state,
- basic geometry checks pass.

### invalid

`invalid` means:

- the model has not yet been initialized,
- or current evidence is insufficient to produce a trustworthy output.

### lost

`lost` means:

- a valid tracked model existed in the recent past,
- but the solver can no longer maintain a trustworthy result.

No separate `ready` state is required in the first design.

## Geometry Validation

A model may be published as `tracking` only if basic sanity checks pass:

- width consistency against the known front-face width,
- reasonable left/right depth difference,
- top consistency when a valid top beam exists,
- non-expired structure freshness,
- sufficient tracked confidence to avoid publishing a stale hallucinated model.

The first implementation should favor simple hard checks that catch obviously bad geometry.

## Offline Validation

The refactor should be verifiable without live camera or LiDAR topics.

### Validation mode

Use a static-scene hybrid offline validation mode:

- ROI runs on a reference RGB image of the stationary frame,
- LiDAR tracking/restoration runs on recorded projected-point data,
- both assets are accepted as compatible because the frame position did not change between captures.

This is not a synchronization test. It is a method test under a static-scene assumption.

### Chosen assets

Reference RGB image for ROI:

- [analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png](/home/sy/code/ws_fastlio_nx/analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png)

Projected LiDAR evidence for tracking/restoration:

- [analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz](/home/sy/code/ws_fastlio_nx/analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz)

The recorded `.npz` contains:

- `uv`
- `depth`
- `stamp`

Using camera intrinsics, these should be sufficient to reconstruct camera-frame point samples for the offline pipeline.

### What offline validation should verify

- ROI produces the three structure masks in physically correct semantics.
- Filtered point counts remain plausible for the three structures.
- The three structure states update over time without live sensors.
- The model initializes only after at least one valid top-beam initialization.
- The solver maintains `tracking` when the top beam later weakens or disappears.
- The four front-face corners remain stable in the camera frame.
- The debug top-corner output matches directly supported structure evidence.

### What offline validation does not verify

- live topic synchronization,
- real-time ROS timing behavior,
- robustness to dynamic scene changes,
- generic multi-object tracking.

## Implementation Impact

Expected change areas:

- ROI message definitions for structure semantics,
- `roi_generator_node.py`,
- image-side structure derivation from the existing corner result,
- LiDAR-side filtering logic,
- online structure state management,
- final corner restoration output,
- offline verification helpers and tests.

Likely obsolete or downgraded pieces:

- direct corner-ROI publication as the main contract,
- direct per-corner LiDAR solve logic.

## Risks and Tradeoffs

Main tradeoffs:

- The design intentionally keeps a strong geometry prior instead of solving a generic unconstrained 3D model.
- The top beam is required for first initialization, which delays first success if top-beam evidence is poor.
- Tracking may keep output alive through weak observation periods, which makes diagnostics critical.

Mitigations:

- keep debug outputs for the directly solved top corners,
- keep `tracking / invalid / lost` plus confidence diagnostics,
- keep offline validation focused on state transitions and structural correctness,
- preserve the existing stable image-side corner logic instead of replacing it.

## Recommended Next Step

Write an implementation plan that covers:

1. replacing the corner-based ROI contract with structure masks,
2. adapting ROI generation from corner connections plus line dilation,
3. implementing the LiDAR point-filtering layer,
4. implementing single-target structure states and update rules,
5. restoring the four front-face corners plus two debug top corners,
6. building the offline validation path from the chosen RGB and `.npz` assets.
