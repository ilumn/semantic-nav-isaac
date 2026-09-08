# Live Semantic Map Implementation Checklist

This checklist is intended to be implementation-complete. If every item here is done and validated, the plan is implemented.

## 1. Workspace and package scaffolding

- [x] Create `tb3_semantic_map_msgs` package in `dev/turtlebot3/ros2_ws/src`
- [x] Add `package.xml` for `tb3_semantic_map_msgs`
- [x] Add `CMakeLists.txt` for message generation
- [x] Add message definitions for `SemanticEntity`
- [x] Add message definitions for `SemanticPlace`
- [x] Add message definitions for `SemanticRelation`
- [x] Add message definitions for `SemanticAnchor`
- [x] Add message definitions for `SemanticMapState`
- [x] Add dependencies needed by the messages
- [ ] Build the workspace and verify message generation succeeds

- [x] Create `tb3_semantic_map` package in `dev/turtlebot3/ros2_ws/src`
- [x] Add `package.xml` for `tb3_semantic_map`
- [x] Add `setup.py`
- [x] Add `setup.cfg`
- [x] Add package resource marker
- [x] Add Python module directory
- [x] Add `config/semantic_map.yaml`
- [x] Add `launch/semantic_map.launch.py`
- [x] Add `README.md`

- [x] Create `tb3_semantic_refiner` package in `dev/turtlebot3/ros2_ws/src`
- [x] Add `package.xml` for `tb3_semantic_refiner`
- [x] Add `setup.py`
- [x] Add `setup.cfg`
- [x] Add package resource marker
- [x] Add Python module directory
- [x] Add `config/semantic_refiner.yaml`
- [x] Add `launch/semantic_refiner.launch.py`
- [x] Add `README.md`

## 2. ROS2 message and schema design

- [x] Define required fields for `SemanticEntity`
- [x] Define required fields for `SemanticPlace`
- [x] Define required fields for `SemanticRelation`
- [x] Define required fields for `SemanticAnchor`
- [x] Define top-level structure for `SemanticMapState`
- [x] Include timestamps and frame ids where needed
- [x] Include confidence fields where needed
- [x] Include ids and parent references where needed
- [x] Keep all navigation-facing poses in `map`
- [x] Confirm message shapes are sufficient for query, nav, RViz, and refinement

## 3. Live semantic-map package core

- [x] Add a semantic-map node entrypoint
- [x] Add node parameters for image topic names
- [x] Add node parameters for camera info topic names
- [x] Add node parameters for scan topic names
- [x] Add node parameters for map topic names
- [x] Add node parameters for TF / frame names
- [x] Add node parameters for detector settings
- [x] Add node parameters for frame sampling rate
- [x] Add node parameters for match thresholds
- [x] Add node parameters for publish rates
- [x] Add node parameters for class filters / prompt vocabulary

- [x] Subscribe to `/camera/image_raw`
- [x] Subscribe to `/camera/camera_info`
- [x] Subscribe to `/scan`
- [x] Subscribe to `/map`
- [x] Initialize TF buffer / listener
- [x] Cache latest camera intrinsics
- [x] Cache latest scan
- [x] Cache latest occupancy map
- [x] Add frame sampler so inference does not run on every image callback

## 4. Detection integration

- [x] Choose the first detector backend for the live semantic map
- [x] Reuse current YOLOv8 stack where practical for the first milestone
- [x] Ensure detector output preserves detector label and confidence
- [x] Add optional class filtering from config
- [x] Add optional debug image publication
- [x] Add detector wrapper module with no ROS dependency where possible
- [x] Add detector unit tests or smoke tests

## 5. Map-frame grounding

- [x] Convert detection pixel center to image bearing
- [x] Use camera intrinsics or configured HFOV consistently
- [x] Fuse scan data to estimate target range
- [x] Handle missing or invalid scan returns
- [x] Use robot pose from TF to move detections into `map`
- [x] Apply camera-to-base extrinsic assumptions consistently
- [x] Reject observations when TF lookup fails
- [x] Reject observations outside map bounds when needed
- [x] Attach confidence / covariance estimate to grounded observations
- [x] Keep grounding logic in a testable pure-Python module where possible

## 6. Live entity memory

- [x] Add in-memory entity registry for map-frame entities
- [x] Match by semantic label plus map-frame distance
- [x] Add entity creation rules
- [x] Add entity update rules
- [x] Add pose smoothing in `map`
- [x] Track `times_seen`
- [x] Track `last_seen`
- [x] Track lifecycle state
- [x] Add stale timeout policy
- [x] Add removal timeout policy
- [x] Add confidence smoothing / aggregation
- [x] Add provenance fields for live observations
- [x] Add unit tests for entity matching and aging

## 7. Place, relation, and anchor builders

- [x] Add place builder module
- [x] Group nearby entities into places
- [x] Assign place ids
- [x] Compute place anchor poses
- [x] Estimate place extents
- [x] Add place confidence scoring

- [x] Add relation builder module
- [x] Implement conservative `near` relation
- [x] Implement conservative support / containment relation if practical
- [x] Prevent low-confidence relation spam
- [x] Add relation confidence scoring

- [x] Add anchor builder module
- [x] Create approach / inspection anchors from entities
- [x] Keep anchors in `map`
- [x] Associate anchors with target entity ids or place ids
- [x] Add anchor confidence scoring

## 8. Semantic map publication

- [x] Publish `/semantic_map/state` as `SemanticMapState`
- [x] Publish at a controlled rate
- [x] Ensure all published poses use `map`
- [x] Include entities in the state
- [x] Include places in the state
- [x] Include relations in the state
- [x] Include anchors in the state
- [x] Include basic diagnostics or status metadata

## 9. Legacy compatibility export

- [x] Add converter from `SemanticEntity` to `vision_msgs/Detection3D`
- [x] Add converter from semantic map state to `Detection3DArray`
- [x] Publish `/semantic_map/compat_objects`
- [x] Preserve object id in the compatibility export
- [x] Preserve detector label in the compatibility export
- [x] Preserve confidence in the compatibility export
- [x] Preserve `map` frame id in the compatibility export
- [x] Add tests for conversion correctness

## 10. RViz and ROS2 debugging surfaces

- [x] Add marker publisher for live entities
- [x] Add marker publisher for places
- [x] Add marker publisher for anchors
- [x] Use stable marker ids
- [x] Delete stale markers correctly
- [x] Add color scheme by semantic class
- [x] Add readable labels
- [x] Add `/semantic_map/status`
- [x] Add `/semantic_map/debug_image` if useful
- [x] Update RViz config to display the new semantic-map layers

## 11. Query integration

- [x] Add config switch in `tb3_query` for memory source selection
- [x] Support legacy source `/semantic_memory_node/objects`
- [x] Support new source `/semantic_map/compat_objects`
- [x] Preserve current command parsing behavior
- [x] Preserve `semantic_targets.yaml` mapping rules
- [ ] Validate query success against the new compatibility source
- [ ] Validate query failure modes against the new compatibility source

## 12. Nav adapter integration

- [ ] Validate `tb3_nav_adapter` against query results driven by the new semantic-map source
- [ ] Confirm goal poses remain in `map` when available
- [ ] Confirm approach offsets remain safe
- [ ] Confirm current fallback behavior still works
- [x] Add support for semantic anchors when query is upgraded to native semantic-map data

## 13. Coordinator and launch integration

- [x] Add `tb3_semantic_map` launch to the relevant bringup path
- [x] Add `tb3_semantic_refiner` launch to the relevant bringup path
- [x] Add source-selection launch arg for query integration
- [x] Keep the legacy detector/localizer/memory stack launchable
- [x] Add new semantic-map nodes to the full semantic navigation launch
- [x] Keep startup ordering safe with timers or readiness checks

## 14. Asynchronous refiner package

- [x] Add frame buffer component
- [x] Store sampled RGB frames for refinement jobs
- [x] Store synchronized timestamps for buffered frames
- [x] Store robot poses in `map` for buffered frames
- [x] Store camera metadata and intrinsics
- [x] Gate sampled frames on robot motion baseline so static views do not flood the bundle
- [x] Define bundle / job directory structure
- [x] Add worker invocation wrapper
- [x] Add status reporting for jobs
- [x] Gate worker launch on bundle readiness so weak bundles do not trigger COLMAP
- [x] Prevent repeated worker retries on unchanged bundles
- [x] Add timeout / failure handling
- [x] Add cleanup policy for old jobs

## 15. `semantic-nav-memory` worker integration

- [x] Identify a stable library or CLI entrypoint in `semantic-nav-memory`
- [x] Add non-browser worker entrypoint if missing
- [x] Ensure the worker can run from exported frame bundles
- [x] Ensure prompts / detector configuration are controllable by config
- [x] Ensure outputs can be consumed without the web server
- [x] Document Python runtime and environment expectations
- [x] Keep this worker off the live ROS2 callback path

## 16. Reconstruction-to-map alignment

- [x] Export pose correspondences between keyframes and robot `map` poses
- [x] Match COLMAP camera centers to buffered frame ids
- [x] Estimate similarity transform from reconstruction coordinates to `map`
- [x] Recover scale from pose correspondences
- [x] Recover rotation / translation into `map`
- [x] Validate planar consistency with the SLAM map
- [x] Compute alignment confidence
- [x] Reject weak alignment solutions
- [x] Store alignment metadata with each refinement job
- [x] Add tests for alignment with synthetic or recorded correspondences

## 17. COLMAP semantic overlay publication

- [x] Read aligned scene graph outputs from the refiner
- [x] Convert aligned entities into RViz markers
- [x] Convert aligned places into RViz markers
- [x] Convert aligned anchors into RViz markers
- [x] Publish `/semantic_map/colmap_markers`
- [x] Publish `/semantic_map/refiner_status`
- [x] Optionally publish machine-readable aligned semantic state
- [x] Make clear in diagnostics that this layer is reference-only

## 18. Semantic enrichment from refinement output

- [x] Define matching rules between live entities and aligned COLMAP entities
- [x] Define provenance representation for refined semantics
- [x] Add optional place enrichment from aligned scene graph
- [x] Add optional relation enrichment from aligned scene graph
- [x] Add optional anchor refinement from aligned scene graph
- [x] Prevent weak aligned data from replacing strong live map data
- [x] Keep live semantic map authoritative for navigation

## 19. Testing and validation

- [x] Add unit tests for grounding
- [x] Add unit tests for entity memory
- [x] Add unit tests for place builder
- [x] Add unit tests for relation builder
- [x] Add unit tests for anchor builder
- [x] Add unit tests for compatibility export
- [x] Add unit tests for alignment math

- [ ] Run end-to-end simulation with legacy semantic stack only
- [ ] Run end-to-end simulation with new semantic-map stack plus compatibility export
- [ ] Visualize live semantic map in RViz
- [ ] Validate `go to the person` against the new source
- [ ] Validate `go to the table` against the new source
- [ ] Validate `go to the stop sign` against the new source
- [ ] Compare new semantic-map stability against legacy memory
- [ ] Validate degraded behavior when the refiner is disabled
- [ ] Validate degraded behavior when a refiner job fails

## 20. Documentation and operator guidance

- [x] Document package purpose and launch steps for `tb3_semantic_map`
- [x] Document package purpose and launch steps for `tb3_semantic_refiner`
- [x] Document topic contracts
- [x] Document message schemas
- [x] Document runtime dependencies
- [x] Document debugging workflow in RViz
- [x] Document how to switch query source between legacy and new semantic map
- [x] Document refiner limitations and authority boundaries
- [x] Document that semantic mapping never writes back into SLAM `/map`

## 21. Cutover completion criteria

- [ ] Live semantic map is visible and stable in RViz
- [ ] Query succeeds from the new semantic-map source
- [ ] Nav goal generation succeeds from the new semantic-map source
- [ ] Legacy semantic stack remains available during transition
- [ ] COLMAP-aligned overlay can be generated and visualized
- [ ] Failures in the refiner do not break live semantic navigation
- [ ] Team can explain which semantic source is authoritative at runtime
