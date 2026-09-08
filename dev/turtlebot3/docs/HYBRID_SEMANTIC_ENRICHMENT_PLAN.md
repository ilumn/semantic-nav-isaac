# Hybrid Semantic Enrichment Plan

## Summary

The runtime semantic system should remain grounded in the robot's ROS-native geometry:

- SLAM `map`
- 2D occupancy and free-space geometry
- 2D scan-based grounding
- camera intrinsics and camera-to-base assumptions
- navigation-safe semantic anchors in `map`

`semantic-nav-memory` and COLMAP stay in the stack, but only as an enrichment path. A successful refinement job may improve anchor choice, extent estimates, place membership, and weak relations for entities already grounded by the live semantic map. It must not become the authority for query or navigation.

## Implementation Direction

### 1. Runtime authority

- Keep `tb3_semantic_map` authoritative for live entities, places, relations, and anchors.
- Keep `tb3_query` and `tb3_nav_adapter` pointed at the live semantic map state for navigation-facing behavior.
- Treat `tb3_semantic_refiner` output as reference-only enrichment and visualization.
- Preserve live operation when the refiner is disabled or when COLMAP reconstruction fails.

### 2. Relation vocabulary

- Keep canonical stored relation predicate as `near`.
- Do not publish `left_of` or `right_of` in runtime semantic state.
- Interpret user phrases `next to`, `left of`, and `right of` as direction-agnostic proximity.
- Keep conservative runtime relations limited to claims the current geometry can defend.

### 3. Query behavior

- Add relational query parsing to `tb3_query` for commands like `go to the person near the table`.
- Support relation phrases:
  - `near`
  - `next to`
  - `left of`
  - `right of`
- Normalize those phrases onto canonical predicate `near`.
- Keep `SemanticQueryResult` stable; place relation-match trace details in status text only.
- Require `memory_mode=semantic_map_state` for relational query execution.

### 4. Pair selection policy

- Evaluate all valid target/reference pairs.
- Rank pairs by:
  1. strongest relation confidence
  2. nearest target to the robot
  3. highest target confidence
- Keep anchor preference unchanged once a target has been selected.

### 5. COLMAP enrichment boundaries

- Enrichment may attach to already grounded live entities only.
- Enrichment may add weak relations, anchors, aliases, place assignment, and extent hints.
- Enrichment must not create navigation-authoritative entities on its own.
- Overlay visualization should show accepted real refinement output only.

## Acceptance

- Target-only queries still work.
- Relational queries work against `SemanticMapState`.
- `left of`, `right of`, and `next to` behave as aliases for proximity.
- Query/nav remain operational when refinement fails.
- A failed COLMAP job never clears the live semantic map.

## Monocular COLMAP Quality Hardening

The enrichment path should push monocular reconstruction quality as far as possible without introducing sensors or APIs that the robot does not already have.

- Feed `semantic-nav-memory` the sampled robot camera frames directly instead of rebuilding a compressed intermediate video.
- Carry robot camera intrinsics from ROS camera info into the reconstruction worker when available.
- Prefer higher-coverage frame bundles and denser sparse-point export over low-latency refresh.
- Keep the runtime interface ROS-native:
  - one live RGB camera topic
  - one `map` frame
  - one semantic refiner node
- Treat COLMAP density improvements as internal worker changes so sim and real robot integrations keep the same ROS surface area.
