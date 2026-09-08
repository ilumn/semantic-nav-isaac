from __future__ import annotations

import copy
import math


def _entity_label(entity) -> str:
    return getattr(entity, "semantic_name", "") or getattr(entity, "detector_label", "")


def _distance_xy(left_x: float, left_y: float, right_x: float, right_y: float) -> float:
    return math.hypot(left_x - right_x, left_y - right_y)


def match_refined_entities_to_live(
    live_entities: list,
    refined_entities: list,
    match_distance_m: float,
    min_confidence: float,
) -> dict[str, str]:
    matches: dict[str, str] = {}
    claimed_live_ids: set[str] = set()

    refined_sorted = sorted(
        [entity for entity in refined_entities if float(getattr(entity, "confidence", 0.0)) >= min_confidence],
        key=lambda entity: float(getattr(entity, "confidence", 0.0)),
        reverse=True,
    )
    for refined in refined_sorted:
        refined_label = _entity_label(refined)
        best_live = None
        best_distance = None
        for live in live_entities:
            live_id = getattr(live, "entity_id", "")
            if live_id in claimed_live_ids:
                continue
            if _entity_label(live) != refined_label:
                continue
            distance = _distance_xy(
                getattr(live.pose.position, "x", 0.0),
                getattr(live.pose.position, "y", 0.0),
                getattr(refined.pose.position, "x", 0.0),
                getattr(refined.pose.position, "y", 0.0),
            )
            if distance > match_distance_m:
                continue
            if best_live is None or distance < best_distance:
                best_live = live
                best_distance = distance
        if best_live is not None:
            refined_id = getattr(refined, "entity_id", "")
            live_id = getattr(best_live, "entity_id", "")
            matches[refined_id] = live_id
            claimed_live_ids.add(live_id)
    return matches


def enrich_entities_with_refinement(live_entities: list, refined_entities: list, matches: dict[str, str]) -> int:
    refined_by_id = {getattr(entity, "entity_id", ""): entity for entity in refined_entities}
    applied = 0
    for live in live_entities:
        live_id = getattr(live, "entity_id", "")
        refined_id = next((rid for rid, lid in matches.items() if lid == live_id), "")
        if not refined_id:
            continue
        refined = refined_by_id.get(refined_id)
        if refined is None:
            continue
        live.aliases = sorted(set(list(getattr(live, "aliases", [])) + list(getattr(refined, "aliases", []))))
        live.provenance = sorted(
            set(
                list(getattr(live, "provenance", []))
                + list(getattr(refined, "provenance", []))
                + [f"refined_match:{refined_id}"]
            )
        )
        applied += 1
    return applied


def enrich_places_with_refinement(
    live_places: list,
    refined_places: list,
    matches: dict[str, str],
    match_distance_m: float,
    min_confidence: float,
) -> tuple[list, dict[str, str]]:
    enriched_places = list(live_places)
    refined_place_map: dict[str, str] = {}

    for refined in refined_places:
        if float(getattr(refined, "confidence", 0.0)) < min_confidence:
            continue
        mapped_members = [
            matches[member_id]
            for member_id in list(getattr(refined, "member_entity_ids", []))
            if member_id in matches
        ]
        if not mapped_members:
            continue
        duplicate = False
        for live in live_places:
            if set(getattr(live, "member_entity_ids", [])) == set(mapped_members):
                duplicate = True
                break
            distance = _distance_xy(
                getattr(live.anchor_pose.position, "x", 0.0),
                getattr(live.anchor_pose.position, "y", 0.0),
                getattr(refined.anchor_pose.position, "x", 0.0),
                getattr(refined.anchor_pose.position, "y", 0.0),
            )
            if distance <= match_distance_m * 0.5:
                duplicate = True
                break
        if duplicate:
            continue

        place = copy.deepcopy(refined)
        original_id = getattr(place, "place_id", "")
        place.place_id = f"refined_{original_id}"
        place.member_entity_ids = mapped_members
        place.confidence = min(0.9, float(getattr(place, "confidence", 0.0)))
        enriched_places.append(place)
        refined_place_map[original_id] = place.place_id
    return enriched_places, refined_place_map


def enrich_relations_with_refinement(
    live_relations: list,
    refined_relations: list,
    entity_matches: dict[str, str],
    refined_place_map: dict[str, str],
    min_confidence: float,
) -> list:
    enriched_relations = list(live_relations)
    seen = {
        (
            getattr(relation, "subject_id", ""),
            getattr(relation, "predicate", ""),
            getattr(relation, "object_id", ""),
        )
        for relation in live_relations
    }

    for refined in refined_relations:
        if float(getattr(refined, "confidence", 0.0)) < min_confidence:
            continue
        subject = entity_matches.get(getattr(refined, "subject_id", ""), "")
        object_id = getattr(refined, "object_id", "")
        mapped_object = entity_matches.get(object_id, refined_place_map.get(object_id, ""))
        if not subject or not mapped_object:
            continue
        triplet = (subject, getattr(refined, "predicate", ""), mapped_object)
        if triplet in seen:
            continue
        relation = copy.deepcopy(refined)
        relation.subject_id = subject
        relation.object_id = mapped_object
        relation.confidence = min(0.85, float(getattr(relation, "confidence", 0.0)))
        enriched_relations.append(relation)
        seen.add(triplet)
    return enriched_relations


def enrich_anchors_with_refinement(
    live_anchors: list,
    refined_anchors: list,
    entity_matches: dict[str, str],
    min_confidence: float,
    duplicate_distance_m: float,
) -> list:
    enriched_anchors = list(live_anchors)
    existing_by_target: dict[str, list] = {}
    for anchor in live_anchors:
        existing_by_target.setdefault(getattr(anchor, "target_id", ""), []).append(anchor)

    for refined in refined_anchors:
        if float(getattr(refined, "confidence", 0.0)) < min_confidence:
            continue
        mapped_target = entity_matches.get(getattr(refined, "target_id", ""), "")
        if not mapped_target:
            continue
        duplicate = False
        for existing in existing_by_target.get(mapped_target, []):
            distance = _distance_xy(
                getattr(existing.pose.position, "x", 0.0),
                getattr(existing.pose.position, "y", 0.0),
                getattr(refined.pose.position, "x", 0.0),
                getattr(refined.pose.position, "y", 0.0),
            )
            if distance <= duplicate_distance_m:
                duplicate = True
                break
        if duplicate:
            continue
        anchor = copy.deepcopy(refined)
        anchor.anchor_id = f"refined_{getattr(refined, 'anchor_id', '')}"
        anchor.target_id = mapped_target
        anchor.confidence = min(0.9, float(getattr(anchor, "confidence", 0.0)))
        enriched_anchors.append(anchor)
        existing_by_target.setdefault(mapped_target, []).append(anchor)
    return enriched_anchors


def enrich_state_with_refinement(
    state,
    refined_state,
    match_distance_m: float,
    min_confidence: float,
):
    if refined_state is None:
        return state
    live_place_count = len(list(getattr(state, "places", [])))
    live_relation_count = len(list(getattr(state, "relations", [])))
    live_anchor_count = len(list(getattr(state, "anchors", [])))
    entity_matches = match_refined_entities_to_live(
        list(getattr(state, "entities", [])),
        list(getattr(refined_state, "entities", [])),
        match_distance_m=match_distance_m,
        min_confidence=min_confidence,
    )
    if not entity_matches:
        return state

    applied = enrich_entities_with_refinement(
        list(getattr(state, "entities", [])),
        list(getattr(refined_state, "entities", [])),
        entity_matches,
    )
    state.places, refined_place_map = enrich_places_with_refinement(
        list(getattr(state, "places", [])),
        list(getattr(refined_state, "places", [])),
        entity_matches,
        match_distance_m=match_distance_m,
        min_confidence=min_confidence,
    )
    state.relations = enrich_relations_with_refinement(
        list(getattr(state, "relations", [])),
        list(getattr(refined_state, "relations", [])),
        entity_matches,
        refined_place_map,
        min_confidence=min_confidence,
    )
    state.anchors = enrich_anchors_with_refinement(
        list(getattr(state, "anchors", [])),
        list(getattr(refined_state, "anchors", [])),
        entity_matches,
        min_confidence=min_confidence,
        duplicate_distance_m=match_distance_m * 0.35,
    )
    if (
        applied
        or len(state.places) > live_place_count
        or len(state.relations) > live_relation_count
        or len(state.anchors) > live_anchor_count
    ):
        state.source = "tb3_semantic_map/live_with_refinement"
        state.refinement_active = True
    return state
