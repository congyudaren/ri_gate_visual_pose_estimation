# ROI Generator Priors Design

Date: 2026-06-13

## Goal

Improve first-stage ROI generation stability without switching to ORB or another feature method. The node should keep YOLO as the object detector and keep the current Hough/refined-line extraction as the first candidate generator, then validate and stabilize the candidate before publishing `FrontFaceROIArray`.

The main failure observed on `analysis_artifacts/video/20260422_142131_rot180.mp4` is not YOLO bbox instability. The detector produced one object on most frames, with only a short missed burst at frames 240-243. The unstable part is structure refinement: vertical post lines can collapse to a few pixels or flip between different image edges.

## Scope

This change is limited to the first-stage image ROI path:

- `roi_generator_node`
- small helper modules for geometry validation and temporal state, if useful
- deterministic unit tests
- evaluation using `evaluate_roi_video_stability`

The downstream LiDAR solver and existing message contract should remain unchanged for the first implementation. If later needed, a separate change can add explicit refiner quality fields to the message schema.

## Geometry Prior

The real gate geometry is:

- top beam length: 1 m
- vertical post length: 2 m

The first-stage node has no depth, so it cannot use absolute meters directly. It should use the soft image-space ratio:

```text
top_beam_length_px / median_vertical_post_length_px ~= 0.5
```

This prior must not be hard-coded as a brittle equality. It should be scored with tolerance and relaxed when the object is clipped by the image border.

## Candidate Validation

For each detected bbox, the node should run the existing `refine_corners_inside_bbox` path and build candidate structure lines. Before publishing those lines, validate them with image-space checks:

- Left and right posts should be mostly vertical.
- The top beam should be mostly horizontal.
- Post lengths should not collapse relative to bbox height.
- Left and right post lengths should be reasonably similar.
- `top_len / median_post_len` should usually stay near `0.5`.
- Left and right posts should remain near the left and right sides of the bbox.
- The top beam should remain near the top side of the gate candidate.
- If the bbox touches the image border, relax ratio and length checks because partial field-of-view clipping can distort visible lengths.

Invalid refined candidates should not be published as if they were high-quality lines.

## Temporal Prior

The node should maintain a lightweight last-valid image-space ROI state per object stream. This is intentionally simpler than a full tracker.

Use the last accepted structure lines to:

- reject sudden line midpoint or length jumps;
- hold the last valid ROI through short detector miss bursts;
- prefer last-valid or bbox-derived fallback when Hough refinement produces an implausible result.

The initial implementation can assume one target gate in the first-stage evaluation path. If multi-target support becomes necessary, object association can be added later using bbox IoU or center distance.

## Runtime Behavior

For each input frame:

1. Run YOLO detection as today.
2. For each detected bbox, run the current refined-corner candidate generator.
3. Convert candidate corners into structure lines.
4. Validate geometry and temporal consistency.
5. If valid, publish the refined structure ROI and update last-valid state.
6. If invalid but the bbox is valid, publish bbox-derived structure lines or last-valid held lines.
7. If the detector misses for a short burst, publish the last valid ROI for a configurable number of frames.
8. If misses exceed the hold limit, publish no object.

The fallback source should be distinguishable internally. The existing `StructureROI.source` can use values such as:

- `corner_refined`
- `bbox_fallback`
- `temporal_hold`

## Parameters

Add conservative parameters to `roi_generator_node`:

```yaml
roi_enable_geometry_prior: true
roi_enable_temporal_prior: true
roi_temporal_hold_frames: 5
roi_max_line_jump_px: 80.0
roi_min_post_bbox_height_ratio: 0.45
roi_expected_top_post_ratio: 0.5
roi_top_post_ratio_tolerance: 0.25
roi_border_relax_px: 8
```

These defaults should prefer stable output over accepting visibly bad refined lines. They can be tuned with the offline video evaluator.

## Tests

Add deterministic unit tests for:

- expected 1:2 gate geometry is accepted;
- collapsed post lines are rejected;
- top/bottom line flip is rejected;
- border-clipped bbox relaxes ratio checks;
- invalid refinement falls back to bbox-derived structures or last valid structures;
- short detector miss burst holds previous ROI;
- long detector miss burst clears output;
- existing debug-image and ROI publisher behavior remains compatible.

Tests should avoid requiring the real YOLO model or video file. The offline video evaluator remains the integration/performance check.

## Evaluation

Use the reusable evaluator:

```bash
export PYTHONPATH=src/roi_lidar_corner:$PYTHONPATH
export YOLO_CONFIG_DIR=/tmp/ultralytics_cfg

python3 -m roi_lidar_corner.evaluate_roi_video_stability \
  analysis_artifacts/video/20260422_142131_rot180.mp4 \
  --output-csv analysis_artifacts/video/roi_stability_20260422_142131_rot180.csv
```

Baseline on the rotated video before adding priors:

```text
frames=366
detected=362
missed=4
multi=0
missed_frame_indices=240,241,242,243
left_len_delta_px max ~= 402.95
right_len_delta_px max ~= 403.55
```

Initial success criteria:

- no duplicate detections are introduced;
- short missed burst is covered by temporal hold or reduced by threshold tuning;
- post line collapse disappears from the published ROI stream;
- `left_len_delta_px` and `right_len_delta_px` no longer show jumps near full post height;
- bbox metrics do not regress materially.

## Non-Goals

- Do not replace YOLO.
- Do not switch to ORB in this change.
- Do not alter downstream LiDAR solver behavior.
- Do not change message schemas unless a separate schema-change decision is made.
