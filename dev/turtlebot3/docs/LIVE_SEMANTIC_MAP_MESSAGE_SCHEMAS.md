# Live Semantic Map Message Schemas

This file documents the message contracts that the live semantic map, query, nav, and refiner paths depend on.

## `tb3_semantic_map_msgs/SemanticMapState`

Top-level semantic state in `map`.

- `header.frame_id`
  Expected to be `map`
- `source`
  Origin string such as `tb3_semantic_map/live` or `tb3_semantic_map/live_with_refinement`
- `refinement_active`
  `true` when reference-only refined semantics have been merged into the published state
- `entities`
  `SemanticEntity[]`
- `places`
  `SemanticPlace[]`
- `relations`
  `SemanticRelation[]`
- `anchors`
  `SemanticAnchor[]`

## `tb3_semantic_map_msgs/SemanticEntity`

Navigation-facing persistent entity.

- `entity_id`
  Stable live-memory id
- `semantic_name`
  Canonical semantic target name
- `detector_label`
  Detector-facing class label
- `place_id`
  Current place membership when assigned
- `state`
  Lifecycle state such as `active` or `stale`
- `aliases`
  Alternate labels
- `provenance`
  Source markers such as `live_ros2_grounded` and `refined_match:<id>`
- `pose`
  Entity pose in `map`
- `pose_covariance`
  Flattened `6x6` covariance, with `x/y` populated from live grounding uncertainty
- `extent`
  Approximate size
- `confidence`
  Aggregated confidence
- `observation_count`
  Number of contributing observations
- `last_seen`
  Timestamp of the latest contributing observation

## `tb3_semantic_map_msgs/SemanticPlace`

Conservative place grouping for nearby entities.

- `place_id`
- `place_type`
- `member_entity_ids`
- `anchor_pose`
- `extent`
- `confidence`

## `tb3_semantic_map_msgs/SemanticRelation`

Conservative semantic or spatial relation.

- `subject_id`
- `predicate`
- `object_id`
- `confidence`

## `tb3_semantic_map_msgs/SemanticAnchor`

Navigation-facing anchor pose in `map`.

- `anchor_id`
- `anchor_type`
- `target_id`
- `pose`
- `confidence`

## `tb3_query/SemanticQueryResult`

Query result emitted by `tb3_query`.

- `success`
- `query_text`
- `semantic_name`
- `detector_label`
- `object_id`
- `position`
  Target entity position in the source frame
- `frame_id`
  True source frame of the target data
- `confidence`
  Entity match confidence
- `has_anchor`
  `true` when a native semantic anchor is attached
- `anchor_id`
- `anchor_type`
- `anchor_pose`
  Preferred goal pose when `has_anchor=true`
- `anchor_confidence`
- `status_message`

## Authority Rules

- `SemanticMapState` from `tb3_semantic_map` is the live navigation-facing map.
- `SemanticMapState` from `tb3_semantic_refiner` is reference-only aligned output.
- Refined semantics may enrich live state, but they do not replace live entity poses.
- No semantic message writes back into SLAM `/map`.
