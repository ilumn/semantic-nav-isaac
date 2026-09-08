# Monocular Semantic Mapping Plan

## Purpose

This document defines a standalone plan for building a monocular semantic mapping system from RGB video feeds before integrating it into the existing robot simulation stack.

The intention is to separate semantic-mapping research from the live robot runtime so the representation can mature independently. The output of this work should be a persistent, hierarchical, 3D-tagged world model that can later be adapted back into the robot’s semantic navigation layer.

This document is written to stand on its own. It includes the relevant robotics-project context directly here and does not assume the reader has any other project document open.

## Executive Summary

The robotics project already has a semantic navigation concept, but its current semantic perception is intentionally simple. The robot stack currently assumes that:

- a single semantic runtime owns semantic state
- semantic memory is robot-centric rather than simulator-truth-driven
- semantic state should support text commands such as "go to the red ball"
- a topometric layer should summarize the mapped world into regions
- navigation should reason about target objects rather than only raw coordinates

Those are the right architectural instincts.

What is missing is a richer perception and world-modeling backend. At the moment, the semantic layer is closer to a demo object tracker than a general semantic scene model. It can represent a few detected objects, attach them to coarse topometric segments, and drive simple search and approach behavior, but it does not yet build a broad, realistic, hierarchical understanding of the world.

The purpose of this new phase is to build that richer understanding from RGB video alone.

The result should not be "a detector that emits labels at approximate coordinates."

The result should be:

- a visual reconstruction of the scene or trajectory
- a persistent semantic entity memory
- a place graph
- object-object and object-place relations
- future navigation anchors derived from semantic structure

This is the layer that later gets connected back into robot simulation and semantic navigation.

## Relevant Robotics Context

The broader robotics project is a TurtleBot3 simulation workspace with a semantic navigation layer built on top of mapping and navigation infrastructure.

At a conceptual level, the current robot stack works like this:

- The robot observes the world through onboard sensors.
- A semantic runtime owns semantic detection, semantic memory, topometric indexing, command handling, and target-oriented behavior.
- Semantic state is meant to come from robot-observable sensing, not from simulator ground truth.
- Confirmed semantic objects become memory entries.
- Confirmed semantic objects are attached to coarse free-space regions in a topometric graph.
- A user can issue commands such as "list", "segments", "status", or "go to the red ball".
- When a target is known, the system navigates toward a safe approach pose around that target.
- When a target is unknown, the system searches until the target is found.

That stack is useful, but the perception and representation remain intentionally narrow:

- the detector is hand-built and class-specific
- the object vocabulary is tiny
- object identity is simple
- geometry is limited to what is needed for the demo
- semantic representation is still mostly "object track at estimated position"

This new work is meant to replace that narrow front-end with a stronger semantic world-modeling backbone.

## Main Goal

Build a standalone monocular semantic mapping pipeline that takes RGB video and produces a persistent, hierarchical, 3D-tagged world representation suitable for later adaptation into robot semantic navigation.

That representation should support:

- 3D object instances rather than frame-local detections
- uncertainty-aware object grounding
- persistent entity identity across frames and revisits
- place-level abstractions such as workspace, hallway-like region, doorway zone, tabletop cluster, room-like cluster
- semantic relations such as `on`, `near`, `inside`, `left_of`
- later task anchors for inspection, approach, and interaction

## What We Want the System to Say

Not:

- "chair at x=1.2, y=3.8"

But:

- "chair_03 is a persistent 3D entity observed from keyframes 12, 14, and 21"
- "chair_03 is near desk_01"
- "monitor_01 and keyboard_01 belong to workspace_place_02"
- "cup_02 is on table_01"
- "door_01 is a transition landmark between place_a and place_b"
- "inspection_anchor_05 is a good viewpoint for monitor_01"

That is the real target.

## Terminology

To avoid confusion:

- `Visual SLAM` means incremental camera-pose estimation and map-building from video.
- `Structure-from-Motion` means offline or batch reconstruction of camera poses and sparse geometry.
- `Multi-View Stereo` means denser geometry reconstruction from multiple posed images.
- `Semantic Mapping` means fusing semantics into the geometric world model.
- `Topometric Mapping` means reasoning over places, regions, and connectivity rather than only raw metric coordinates.
- `Monocular` means RGB-only, no LiDAR, no stereo depth, no robot odometry required.

For this project phase, "SLAM" is acceptable language, but "monocular semantic mapping" is more precise.

## Core Design Principle

Geometry and semantics must remain separate layers that are fused together.

Do not build:

- detector output directly as world truth

Do build:

1. frame observations
2. short-term tracked observations
3. persistent 3D entities
4. places
5. relations
6. task anchors

This separation is important because later, the geometry source can change without forcing a rewrite of the semantic layer.

For example:

- in the standalone phase, geometry may come from monocular reconstruction
- later in simulation, geometry may come from SLAM, TF, and occupancy maps
- the higher semantic layers should remain conceptually stable

## Non-Goals for This Phase

This standalone effort should not try to:

- drive live navigation
- command a robot base
- replace the current simulation runtime immediately
- claim high-accuracy metric ground truth from monocular video
- solve robust autonomous exploration
- solve final ROS integration in the same phase
- treat monocular video as if it were equivalent to LiDAR + odometry

Those are later integration problems.

## Why Video-First Is a Good Phase

Video-first experimentation has three benefits:

1. It isolates the semantic representation problem from robot-control noise.
2. It lets geometry and semantic fusion be inspected carefully offline.
3. It creates a better target representation before runtime constraints force simplifications.

This is especially useful because a lot of semantic-navigation failure is really semantic-memory failure, not motion-planning failure.

If the system cannot form a stable, useful world model from a video sequence, it will not suddenly become semantically robust once it is embedded in the simulation stack.

## Recommended First Principle for Representation

Even without LiDAR and robot odometry, the system should still produce a 3D tagged representation.

However, it must represent uncertainty honestly.

The reconstruction should be interpreted as:

- a world model in a reconstruction coordinate frame
- potentially up-to-scale or weakly scaled
- uncertain in depth and global metric alignment

The semantic layer should not hide that uncertainty.

Recommended conventions:

- use `world_pose_estimate` or `reconstruction_pose_mean`
- store `pose_covariance`
- store `scale_status`
- store confidence and observation history

Avoid implying exact metric precision unless the geometry backend supports it reliably.

## Recommended High-Level Architecture

The standalone system should have the following layers.

### 1. Video Ingest Layer

Responsibilities:

- read video files or stream frames
- manage timestamps
- handle frame sampling and keyframe selection
- maintain camera calibration metadata

Outputs:

- frames
- timestamps
- keyframe candidates
- camera intrinsics

### 2. Geometry Backbone Layer

Responsibilities:

- estimate camera poses
- reconstruct sparse geometry
- optionally reconstruct dense geometry or a mesh
- maintain coordinate-frame definition
- support later surface inference

Possible implementations:

- offline: COLMAP
- near-online/live research: DROID-SLAM
- later robotics/live integration: ORB-SLAM3 or similar

Recommended first implementation:

- use COLMAP for offline or batch-style reconstruction from video

Reason:

- easiest to debug and inspect
- geometry artifacts are exportable
- semantic errors are easier to separate from pose-tracking errors

### 3. Detector Layer

Responsibilities:

- detect objects of interest in frames or keyframes
- emit class/posterior information
- optionally emit masks
- expose detector confidence and prompt source

Recommended first implementation:

- YOLO-World through Ultralytics

Recommended model starting points:

- `yolov8s-worldv2` for faster iteration
- `yolov8m-worldv2` if the small model misses too much

Recommended first prompt vocabulary:

- person
- chair
- desk
- table
- door
- monitor
- keyboard
- cup
- bottle
- backpack
- cabinet
- plant
- trash can
- sofa
- bed

Start small. The point is to validate representation quality, not to maximize vocabulary size immediately.

### 4. 2D Tracking Layer

Responsibilities:

- maintain short-term image-space continuity
- reduce detector flicker
- connect detections across nearby frames

Important rule:

- a 2D tracker ID is not a semantic world ID

Tracker IDs are temporary evidence, not final entity identity.

### 5. 3D Grounding Layer

Responsibilities:

- lift 2D detections into the reconstruction frame
- triangulate or otherwise estimate object 3D position
- estimate extent and uncertainty
- connect observations from multiple viewpoints

Grounding strategies may include:

- triangulation from multiple keyframes
- intersection of detection rays with reconstructed geometry
- mask-projected support on dense point cloud or mesh
- plane-based placement priors such as floor/table support

### 6. Persistent Entity Layer

Responsibilities:

- merge observations into world-centric instances
- maintain long-term identity
- model uncertainty and staleness
- represent object properties beyond a single label

Entity identity should combine:

- geometric consistency
- appearance similarity
- class compatibility
- co-visibility history
- support-surface compatibility

### 7. Ontology Layer

Responsibilities:

- define semantic categories and aliases
- encode hierarchy
- encode affordances
- encode mobility priors
- encode support-surface priors

The ontology should be separate from detector prompts.

### 8. Place Layer

Responsibilities:

- group entities and views into meaningful places
- produce room-like or zone-like abstractions
- bridge geometry to navigation-relevant semantics

Examples:

- workspace place
- desk area
- hallway segment
- doorway zone
- lounge area

### 9. Relation Layer

Responsibilities:

- infer semantic relations between entities and places
- support later query and behavior generation

Examples:

- on(cup_02, table_01)
- near(chair_03, desk_01)
- inside(monitor_01, workspace_place_02)
- left_of(door_01, cabinet_02)

### 10. Task Anchor Layer

Responsibilities:

- compute future navigation-relevant anchor points
- identify useful viewpoints for inspection or approach

Examples:

- inspection anchor for a monitor
- approach anchor for a doorway
- interaction anchor for a tabletop object

This layer is not needed to test monocular reconstruction itself, but the schema should reserve space for it because later robot reintegration will need it.

## Recommended World Model Schema

The final output should not be a single flat JSON file with labels and coordinates.

Use several related outputs or a structured combined output.

Recommended top-level artifacts:

- `reconstruction.json`
- `keyframes.json`
- `observations.json`
- `entities.json`
- `places.json`
- `relations.json`
- `scene_graph.json`
- optional labeled point cloud or mesh output

### Reconstruction Schema

Recommended fields:

- coordinate frame definition
- scale status
- camera intrinsics
- camera poses
- sparse geometry metadata
- dense geometry metadata if available

Example:

```json
{
  "frame_id": "reconstruction_world",
  "scale_status": "unknown_or_relative",
  "camera_model": "pinhole",
  "intrinsics": {
    "fx": 800.0,
    "fy": 800.0,
    "cx": 640.0,
    "cy": 360.0
  },
  "keyframes": ["kf_001", "kf_002", "kf_003"]
}
```

### Observation Schema

An observation is a grounded detector output from one keyframe.

Recommended fields:

- `observation_id`
- `keyframe_id`
- `detector_label`
- `class_distribution`
- `bbox`
- `mask_ref`
- `confidence`
- `camera_pose_ref`
- `ray_bundle_ref`
- `world_pose_estimate`
- `pose_covariance`
- `appearance_embedding_ref`

### Entity Schema

An entity is a persistent world-level object hypothesis.

Recommended fields:

```json
{
  "entity_id": "chair_03",
  "canonical_label": "chair",
  "class_distribution": {
    "chair": 0.91,
    "stool": 0.06,
    "bench": 0.03
  },
  "aliases": ["office chair", "seat"],
  "ontology_parents": ["furniture", "seat", "static_landmark"],
  "world_pose_estimate": [1.24, -0.81, 0.02],
  "pose_covariance": [[0.04, 0.0, 0.0], [0.0, 0.04, 0.0], [0.0, 0.0, 0.09]],
  "extent_3d": [0.58, 0.61, 0.94],
  "support_surface_id": "floor_01",
  "place_id": "workspace_place_02",
  "observed_in_keyframes": ["kf_012", "kf_014", "kf_021"],
  "observation_ids": ["obs_140", "obs_163", "obs_190"],
  "state": "confirmed",
  "mobility": "static",
  "confidence": 0.87
}
```

### Place Schema

A place is a semantic region inferred from geometry and object structure.

Recommended fields:

```json
{
  "place_id": "workspace_place_02",
  "place_type_distribution": {
    "workspace": 0.74,
    "office_corner": 0.63,
    "desk_area": 0.81
  },
  "anchor_pose": [1.8, -1.1, 0.0],
  "member_entities": ["desk_01", "chair_03", "monitor_01", "keyboard_01"],
  "support_surfaces": ["desk_surface_01", "floor_01"],
  "observed_in_keyframes": ["kf_014", "kf_021", "kf_022"],
  "confidence": 0.76
}
```

### Relation Schema

Recommended fields:

```json
{
  "subject": "cup_02",
  "predicate": "on",
  "object": "table_01",
  "confidence": 0.81,
  "evidence_keyframes": ["kf_044", "kf_046"]
}
```

### Scene Graph Schema

The fused output should join entities, places, and relations into a navigable graph structure.

That graph should be easy to query later for:

- object lookup
- place lookup
- relation lookup
- later behavior planning

## Ontology Design

The ontology should be a dedicated asset, not a side effect of the detector prompt list.

Recommended ontology fields:

- canonical class
- aliases
- parent categories
- affordances
- mobility
- support priors
- place priors

Example:

```yaml
chair:
  aliases: [office chair, seat]
  parents: [furniture, seat, static_landmark]
  affordances: [landmark, obstacle, human_workspace_context]
  mobility: static
  support_priors: [floor]

cup:
  aliases: [mug, tumbler]
  parents: [container, tabletop_object, movable_object]
  affordances: [pickup_candidate, tabletop_item]
  mobility: movable
  support_priors: [table, desk, counter]

door:
  aliases: [doorway]
  parents: [transition_landmark, structural_element]
  affordances: [room_transition, navigation_landmark]
  mobility: static
  support_priors: [wall_boundary]
```

Why this matters:

- detector labels are noisy
- object naming may vary by prompt set
- later robot commands should resolve over ontology, not raw detector strings

## Why YOLO-World Is Recommended First

YOLO-World is a good first detector because:

- it is open-vocabulary enough to test semantic coverage
- it supports prompt restriction, which helps keep outputs stable
- it allows the project to evolve ontology and vocabulary without immediately retraining
- it is much closer to real robotics perception than hand-coded color heuristics

Recommended practice:

- start with a constrained prompt vocabulary
- inspect failure modes
- stabilize the vocabulary
- later save or freeze the specialized vocabulary
- later distill or fine-tune a closed detector only if runtime deployment demands it

The point of this phase is semantic representation research, not detector perfection.

## Why COLMAP Is Recommended First

COLMAP is recommended as the first geometry backend because it is well-suited for:

- recorded video
- pose estimation from multiple views
- sparse reconstruction
- optional dense reconstruction
- artifact inspection and debugging

This is a good fit for the current phase because the main question is:

- can the project build a useful semantic world model from video?

That question is easier to answer with a robust offline reconstruction than with a fragile real-time stack.

Later, if live performance becomes important:

- DROID-SLAM can be explored for stream-like experiments
- ORB-SLAM3 can be explored for later robotics integration

## 3D Grounding Strategy

This is the critical technical area.

The system must turn 2D detections into 3D semantic entities in a principled way.

Recommended grounding approach:

1. estimate camera poses and keyframes
2. run detector on keyframes
3. collect observations for the same semantic candidate across multiple views
4. estimate a 3D centroid and extent using:
   - triangulation where possible
   - projection into dense geometry where available
   - support-plane priors where direct triangulation is weak
5. maintain uncertainty rather than forcing exact positions

Recommended use of priors:

- chairs usually rest on floor
- tables usually rest on floor
- cups usually rest on tables/desks/counters
- monitors usually rest on desks or mount to walls
- doors align with structural boundaries

These priors should regularize ambiguous monocular grounding.

Important rule:

- do not overclaim 3D precision

If the geometry is weak, the entity should remain uncertain.

## Tracking and Identity Strategy

There should be two levels of temporal continuity:

### Short-Term Tracklets

Purpose:

- reduce detector flicker
- maintain per-frame continuity
- gather multi-frame evidence

### Long-Term Entities

Purpose:

- represent persistent object instances in the world model
- merge evidence across revisits
- avoid tying semantic identity to a tracker implementation

Association for long-term entities should combine:

- class compatibility
- appearance similarity
- geometric proximity in reconstruction space
- support-surface compatibility
- co-visibility and revisit evidence

Avoid:

- direct one-to-one use of tracker IDs as final world entities

## Place Inference Strategy

A good semantic world model needs places, not just objects.

Place inference should use:

- keyframe overlap
- camera pose neighborhoods
- structural geometry
- co-occurring objects
- support surfaces

Examples:

- `desk + monitor + keyboard + chair` suggests a workspace-like place
- `sofa + table + TV` suggests a lounge-like place
- `door + corridor geometry` suggests a transition zone
- `bed + nightstand + lamp` suggests a sleeping area

Places do not need to be perfect room labels at first.

What matters is that they become stable semantic grouping units.

## Relation Inference Strategy

Relations are the bridge from object detection to useful semantic behavior.

Recommended first relation set:

- `near`
- `left_of`
- `right_of`
- `in_front_of`
- `behind`
- `on`
- `under`
- `inside`
- `attached_to`
- `in_place`
- `seen_from_place`

Recommended inference sources:

- 3D geometry
- support surfaces
- object class priors
- co-visibility in keyframes

Examples:

- if cup centroid is above and close to table surface, infer `on`
- if monitor and keyboard share desk support and are close, infer same workspace place
- if a door lies on a structural boundary and connects two pose clusters, infer transition role

## Task Anchors for Later Navigation

Even though navigation is not part of this phase, the semantic model should reserve a layer for future robot behavior.

Task anchors should include:

- inspection anchor
- approach anchor
- interaction anchor
- view anchor

Examples:

- monitor inspection anchor at a normal-facing viewpoint
- doorway approach anchor just before the threshold
- tabletop interaction anchor facing the nearest free side

This matters because later robot navigation should not target raw object centroids.

It should target behavior-specific anchors.

## Recommended Implementation Modules

Suggested standalone code structure:

- `video_io.py`
  - frame loading
  - keyframe extraction
  - timestamp handling

- `geometry_backend.py`
  - interface for visual reconstruction

- `geometry_colmap.py`
  - COLMAP-backed reconstruction

- `detector.py`
  - detector abstraction

- `detector_yoloworld.py`
  - YOLO-World implementation

- `tracker_2d.py`
  - image-space tracklets

- `grounder_3d.py`
  - 2D-to-3D grounding

- `entity_memory.py`
  - persistent instance fusion

- `ontology.py`
  - ontology loading and queries

- `place_builder.py`
  - place inference

- `relation_inference.py`
  - relation generation

- `scene_graph.py`
  - graph assembly

- `exporters.py`
  - JSON, point cloud, mesh, visualization export

- `visualizer.py`
  - inspection and debugging output

## Recommended Processing Pipeline

The end-to-end pipeline should look like this:

1. ingest video
2. calibrate or load intrinsics
3. extract keyframes
4. reconstruct camera poses and sparse geometry
5. optionally densify geometry
6. run YOLO-World on keyframes
7. build short-term 2D tracklets
8. ground detections into 3D observation hypotheses
9. merge observations into persistent entities
10. infer support planes and support surfaces
11. infer places
12. infer relations
13. generate task anchors
14. export scene graph and visual artifacts

## Recommended Development Phases

### Phase 0: Requirements and Schema

Deliverables:

- problem statement
- ontology draft
- prompt list
- entity/place/relation schemas
- video dataset selection

Success criteria:

- vocabulary is bounded
- data model is explicit
- outputs are defined before code expands

### Phase 1: Geometry Backbone

Deliverables:

- keyframe extraction
- camera pose estimation
- sparse reconstruction
- reconstruction export

Success criteria:

- visually coherent camera trajectory
- useful keyframe set
- enough geometric consistency to support semantic fusion

### Phase 2: Detector Integration

Deliverables:

- YOLO-World inference pipeline
- constrained prompt config
- raw detection logging

Success criteria:

- detections are inspectable and reasonably stable
- prompt set is not too noisy

### Phase 3: Short-Term Tracking

Deliverables:

- frame-to-frame 2D continuity
- tracklet logs

Success criteria:

- reduced flicker
- reasonable short-term identity continuity

### Phase 4: 3D Grounding

Deliverables:

- observation lifting into reconstruction space
- uncertainty-aware 3D hypotheses
- support-surface reasoning

Success criteria:

- entities are plausibly placed in 3D
- obvious duplicates are reduced
- support assignments make sense

### Phase 5: Persistent Entity Memory

Deliverables:

- long-term entity memory
- merge/split logic
- state transitions

Entity states may include:

- candidate
- confirmed
- occluded
- stale
- lost

Success criteria:

- stable entities across revisits
- no excessive fragmentation

### Phase 6: Place and Relation Graph

Deliverables:

- place inference
- relation inference
- fused scene graph

Success criteria:

- place clusters are meaningful
- relations are mostly sensible
- graph is queryable and useful

### Phase 7: Visualization and Evaluation

Deliverables:

- annotated video
- entity viewer
- scene graph export
- labeled point cloud or mesh

Success criteria:

- a human can inspect and trust the output
- errors are diagnosable

### Phase 8: Adapter for Later Robot Reintegration

Deliverables:

- export or adapter layer that can later project the richer semantic model into the robot runtime

Success criteria:

- geometry-source-agnostic semantic interfaces
- reintegration path is clear

## Evaluation Criteria

This phase should be evaluated on semantic mapping quality, not robot navigation.

### Detection Quality

- are important object classes found reliably?
- how many false positives come from the prompt set?

### Tracking Quality

- how often does one real object split into many tracklets?
- how often do different real objects get merged?

### 3D Grounding Quality

- are entities plausibly located in the reconstructed scene?
- do extents look reasonable?
- are support-surface assignments sensible?

### Entity Memory Quality

- do entities persist across revisits?
- are duplicates manageable?

### Place Quality

- do object clusters form meaningful places?
- do place labels make sense for the scene?

### Relation Quality

- are `on`, `near`, `inside`, `left_of` mostly credible?

### Representation Quality

- can a human read the scene graph and understand the scene?
- does the graph support later robot queries?

## Known Risks

### Risk: Monocular Scale Ambiguity

Monocular reconstruction often lacks stable metric scale.

Mitigation:

- store scale status explicitly
- avoid overclaiming metric truth
- use priors later if needed

### Risk: Depth Ambiguity for Small Objects

Small objects such as cups may be hard to place accurately.

Mitigation:

- use support-surface priors
- require multi-view evidence
- keep uncertainty high where necessary

### Risk: Dynamic Objects Disturb Geometry

People or moving objects may degrade reconstruction.

Mitigation:

- focus initial testing on mostly static scenes
- downweight dynamic detections during geometry fusion

### Risk: Open-Vocabulary Prompt Noise

Broad prompt sets can create unstable detections.

Mitigation:

- keep prompt list constrained
- evolve ontology separately from prompts

### Risk: Tracker IDs Become Semantic IDs

This produces brittle memory.

Mitigation:

- use tracker IDs only as evidence
- maintain separate entity identity

## Recommended Technical Decisions

Recommended first stack:

- Python
- Ultralytics YOLO-World
- COLMAP
- JSON scene-graph exports
- optional Open3D visualization

Recommended first operating mode:

- offline video processing

Recommended first scenes:

- indoor static or mostly static spaces
- visible furniture and support surfaces
- moderate camera motion
- enough overlap for reconstruction

Avoid initially:

- fast egomotion
- strong motion blur
- crowded dynamic scenes
- low-light video

## Outputs Expected From the Separate Project

Minimum expected outputs:

- semantic mapping design doc
- prompt config
- ontology file
- reconstruction artifact
- semantic entity memory export
- place graph export
- relation graph export
- annotated video

Ideal outputs:

- labeled point cloud
- labeled mesh
- simple interactive viewer
- comparison results across different geometry backends or prompt sets

## Reintegration Intent

When this work returns to the robot project later, the goal should not be to copy the standalone code directly into the runtime unchanged.

The goal should be to transfer:

- the semantic data model
- the ontology
- the entity/place/relation logic
- the detector abstraction
- the task-anchor concepts

The future robot runtime should then choose a geometry source appropriate to the robot:

- monocular visual reconstruction for experiments
- simulation-time SLAM and TF in the robot stack
- later RGB-D, stereo, or LiDAR if available

This is why geometry and semantics must stay modular.

## Final Intent Statement

The purpose of this work is to develop a standalone monocular semantic mapping system that transforms RGB video into a persistent, hierarchical, uncertainty-aware world model composed of 3D entities, places, and relations, so that the robotics project can later replace its current narrow semantic front-end with a richer semantic layer suitable for realistic semantic navigation.

## Summary of Recommended Path

If choosing one concrete path to begin with:

1. use prerecorded video
2. reconstruct geometry with COLMAP
3. detect objects with YOLO-World using a constrained prompt list
4. maintain short-term 2D tracklets
5. fuse multi-view detections into persistent 3D entities
6. infer support surfaces, places, and relations
7. export a scene graph and labeled reconstruction
8. only after that, design the adapter back into the robot simulation runtime

That is the recommended order because it builds the semantic representation first, which is the real asset.
