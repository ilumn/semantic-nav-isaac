# Hybrid Semantic Enrichment Checklist

This checklist tracks the hybrid runtime direction where live ROS semantics stay authoritative and COLMAP is enrichment-only.

## 1. Plan and docs

- [x] Save the hybrid semantic enrichment plan to a repo file
- [x] Save a corresponding implementation checklist to a repo file
- [x] Update operator-facing docs for relational query support

## 2. Query parsing and selection

- [x] Add relational command parsing to `tb3_query`
- [x] Normalize `next to`, `left of`, and `right of` to canonical `near`
- [x] Preserve target-only command behavior
- [x] Add target/reference pair scoring for relational queries
- [x] Rank relational matches by relation confidence first
- [x] Keep `SemanticQueryResult` schema unchanged

## 3. Semantic-map integration

- [x] Read relations from `SemanticMapState` inside `tb3_query`
- [x] Reject relational queries when running in `detection3d` mode
- [x] Use robot-relative distance when ranking `map`-frame semantic targets
- [x] Preserve semantic-anchor preference for selected targets

## 4. Hybrid authority boundaries

- [x] Confirm refined relations only enrich matched live entities
- [x] Confirm refinement cannot create nav-authoritative entities on its own
- [x] Remove any synthetic bootstrap overlay behavior from the tracked production path

## 5. Validation

- [x] Add unit tests for relational parsing
- [x] Add unit tests for relational pair scoring
- [x] Add unit tests for alias normalization
- [x] Run `tb3_query` tests
- [x] Build `tb3_query`
- [x] Run native semantic-map smoke test for target-only and relational queries
- [x] Validate target-only queries still work
- [x] Validate relational queries against `semantic_map_state`

## 6. Nav adapter

- [x] Fix world-frame standoff computation for native semantic-map query results
- [x] Add unit tests for world-frame standoff math
- [x] Build `tb3_nav_adapter`
- [x] Validate map-frame non-anchor goal generation against robot pose
- [x] Validate anchor passthrough for native semantic-map query results
- [x] Run native nav-adapter smoke test

## 7. Monocular COLMAP hardening

- [x] Remove lossy bundle-frame video re-encoding from the refiner worker path
- [x] Feed `semantic-nav-memory` directly from sampled robot image sequences
- [x] Carry ROS camera intrinsics into the reconstruction worker when available
- [x] Add stronger COLMAP fallback attempts for low-texture monocular scenes
- [x] Increase exported sparse-point density for RViz overlays
- [x] Retune refiner defaults toward higher-quality bundles over faster refresh
- [x] Run focused validation for the new direct-image worker path
