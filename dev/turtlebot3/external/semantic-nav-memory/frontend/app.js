import { PointCloudViewer } from "./viewer.js";

const elements = {
  form: document.getElementById("upload-form"),
  file: document.getElementById("video-file"),
  fileError: document.getElementById("video-file-error"),
  preset: document.getElementById("prompt-preset"),
  presetNote: document.getElementById("preset-note"),
  modelSize: document.getElementById("model-size"),
  modelNote: document.getElementById("model-note"),
  prompts: document.getElementById("prompts"),
  submit: document.getElementById("submit-button"),
  progressFill: document.getElementById("progress-fill"),
  progressLabel: document.getElementById("progress-label"),
  jobId: document.getElementById("job-id"),
  jobStage: document.getElementById("job-stage"),
  jobState: document.getElementById("job-state"),
  jobError: document.getElementById("job-error"),
  summaryGrid: document.getElementById("summary-grid"),
  inspector: document.getElementById("inspector"),
  caption: document.getElementById("viewer-caption"),
  videoFrame: document.querySelector(".video-frame"),
  inspectionVideo: document.getElementById("inspection-video"),
  videoRelations: document.getElementById("video-relations"),
  videoOverlays: document.getElementById("video-overlays"),
  videoCard: document.getElementById("video-card"),
  videoCardTitle: document.getElementById("video-card-title"),
  videoCardSubtitle: document.getElementById("video-card-subtitle"),
  videoCardMetrics: document.getElementById("video-card-metrics"),
  videoCardRelations: document.getElementById("video-card-relations"),
  videoCardClose: document.getElementById("video-card-close"),
  videoEmpty: document.getElementById("video-empty"),
  videoPlayToggle: document.getElementById("video-play-toggle"),
  videoPrevKeyframe: document.getElementById("video-prev-keyframe"),
  videoNextKeyframe: document.getElementById("video-next-keyframe"),
  videoScrubber: document.getElementById("video-scrubber"),
  videoTime: document.getElementById("video-time"),
  videoKeyframe: document.getElementById("video-keyframe"),
  videoDetectionCount: document.getElementById("video-detection-count"),
  graphCanvas: document.getElementById("graph-canvas"),
  graphStats: document.getElementById("graph-stats"),
  graphRelations: document.getElementById("graph-relations"),
  graphMeta: document.getElementById("graph-meta"),
  tab3d: document.getElementById("tab-3d"),
  tabVideo: document.getElementById("tab-video"),
  tabGraph: document.getElementById("tab-graph"),
  pane3d: document.getElementById("pane-3d"),
  paneVideo: document.getElementById("pane-video"),
  paneGraph: document.getElementById("pane-graph"),
  rotationValue: document.getElementById("rotation-value"),
  viewerCanvas: document.getElementById("viewer-canvas"),
};
const paneOrder = ["3d", "video", "graph"];

let appConfig = {
  default_prompt_preset: "general",
  prompt_presets: {},
  default_yolo_model_size: "small",
  yolo_world_models: {},
};
let currentScene = null;
let sceneIndex = null;
let currentEntityId = null;
let currentDetectionId = null;
let currentJobId = null;
let currentVideoUrl = null;
let hasUnsavedChanges = false;
let activePane = "3d";

const viewer = new PointCloudViewer(document.getElementById("viewer-canvas"), {
  onSelect: (entityId) => {
    currentEntityId = entityId;
    viewer.setSelectedEntity(entityId);
    renderInspector();
    renderGraphPanel();
  },
  onStateChange: (state) => {
    updateRotationValue(state.rollDegrees);
    syncViewerHelp(state);
  },
});

function formatTitleCase(value) {
  return String(value || "")
    .split(/[_\s]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

function getPresetDetails(presetName = elements.preset.value) {
  return appConfig.prompt_presets?.[presetName] || null;
}

function getModelDetails(modelSize = elements.modelSize.value) {
  return appConfig.yolo_world_models?.[modelSize] || null;
}

function updatePresetNote() {
  const details = getPresetDetails();
  if (!details) {
    elements.presetNote.textContent = "Preset vocabulary unavailable.";
    return;
  }
  const promptCount = details.prompts?.length || 0;
  elements.presetNote.textContent = `${details.description} ${promptCount} base prompts, ${formatTitleCase(details.scene_profile)} scene profile.`;
}

function updateModelNote() {
  const details = getModelDetails();
  if (!details) {
    elements.modelNote.textContent = "Detector size unavailable.";
    return;
  }
  const cacheStatus = details.cached ? "Cached locally." : "Downloads on first use.";
  elements.modelNote.textContent = `${details.description} ${cacheStatus}`;
}

function markDirty(value = true) {
  hasUnsavedChanges = value;
}

function clearFileError() {
  elements.fileError.textContent = "";
  elements.file.removeAttribute("aria-invalid");
}

function setFileError(message) {
  elements.fileError.textContent = message;
  elements.file.setAttribute("aria-invalid", "true");
}

function setSubmitLoading(isLoading) {
  elements.submit.disabled = isLoading;
  elements.submit.classList.toggle("is-loading", isLoading);
  elements.submit.setAttribute("aria-busy", isLoading ? "true" : "false");
  const label = elements.submit.querySelector(".button-label");
  if (label) {
    label.textContent = isLoading ? "Building Semantic Map…" : "Build Semantic Map";
  }
}

function updateRotationValue(degrees = viewer.getRollDegrees()) {
  const normalized = ((degrees % 360) + 360) % 360;
  elements.rotationValue.textContent = `${normalized}°`;
}

function syncViewerHelp(state = viewer.getInteractionState()) {
  const hint = state?.hint || "Left drag orbit. Shift+Left drag pan. Ctrl/Cmd+Left drag zoom. Wheel zooms. 1/3/7 snap views.";
  elements.viewerCanvas.setAttribute("title", hint);
  elements.viewerCanvas.setAttribute("aria-description", hint);
}

function getPaneFromUrl() {
  const url = new URL(window.location.href);
  const view = url.searchParams.get("view");
  return paneOrder.includes(view) ? view : "3d";
}

function syncPaneUrl(nextPane, { replaceHistory = false } = {}) {
  const url = new URL(window.location.href);
  if (nextPane === "3d") {
    url.searchParams.delete("view");
  } else {
    url.searchParams.set("view", nextPane);
  }
  window.history[replaceHistory ? "replaceState" : "pushState"]({ view: nextPane }, "", url);
}

function setActivePane(nextPane, { updateHistory = false, replaceHistory = false } = {}) {
  const pane = paneOrder.includes(nextPane) ? nextPane : "3d";
  if (updateHistory && pane !== activePane) {
    syncPaneUrl(pane, { replaceHistory });
  }
  activePane = pane;
  const paneConfig = [
    { name: "3d", button: elements.tab3d, panel: elements.pane3d, title: "Scene" },
    { name: "video", button: elements.tabVideo, panel: elements.paneVideo, title: "Video" },
    { name: "graph", button: elements.tabGraph, panel: elements.paneGraph, title: "Graph" },
  ];
  paneConfig.forEach(({ name, button, panel }) => {
    const isActive = pane === name;
    button.classList.toggle("control-button-active", isActive);
    button.setAttribute("aria-selected", isActive ? "true" : "false");
    button.tabIndex = isActive ? 0 : -1;
    panel.classList.toggle("viewer-pane-hidden", !isActive);
    panel.classList.toggle("viewer-pane-active", isActive);
    panel.hidden = !isActive;
  });
  document.title = `Semantic Nav Memory · ${paneConfig.find((item) => item.name === pane)?.title || "Scene"}`;
  if (pane === "video") {
    queueVideoInspectorRender();
  }
}

function setJobState(record) {
  currentJobId = record?.job_id || currentJobId;
  currentVideoUrl = record ? getInputVideoUrl(record) : null;
  if (record?.prompt_preset && appConfig.prompt_presets?.[record.prompt_preset]) {
    elements.preset.value = record.prompt_preset;
    updatePresetNote();
  }
  if (record?.model_size && appConfig.yolo_world_models?.[record.model_size]) {
    elements.modelSize.value = record.model_size;
    updateModelNote();
  }
  elements.jobId.textContent = record?.job_id || "None";
  elements.jobStage.textContent = record?.current_step || "Waiting";
  elements.jobState.textContent = record?.status || "Ready";
  elements.progressFill.style.width = `${Math.round((record?.progress || 0) * 100)}%`;
  elements.progressLabel.textContent = record ? `${Math.round((record.progress || 0) * 100)}%` : "Idle";
  elements.jobError.textContent = record?.error || "";
  syncVideoSource();
}

function updateSummary(scene) {
  const values = [
    formatTitleCase(scene.scene_profile || "general"),
    formatTitleCase(scene.prompt_preset || "general"),
    scene.reconstruction.keyframes.length,
    scene.reconstruction.cameras.length,
    scene.reconstruction.sparse_points.length,
    scene.observations.length,
    scene.entities.length,
    scene.relations.length,
  ];
  [...elements.summaryGrid.querySelectorAll("dd")].forEach((node, index) => {
    node.textContent = values[index] ?? "0";
  });
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function compareByLabelAndId(left, right) {
  return (
    String(left.canonical_label || left.place_id || "")
      .localeCompare(String(right.canonical_label || right.place_id || ""))
    || String(left.entity_id || left.place_id || "").localeCompare(String(right.entity_id || right.place_id || ""))
  );
}

function formatPercent(value) {
  return Number.isFinite(value) ? `${(value * 100).toFixed(value >= 0.1 ? 0 : 1)}%` : "n/a";
}

function formatTime(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) {
    return "00:00.0";
  }
  const minutes = Math.floor(seconds / 60);
  const remainder = seconds - (minutes * 60);
  const wholeSeconds = Math.floor(remainder);
  const tenths = Math.floor((remainder - wholeSeconds) * 10);
  return `${String(minutes).padStart(2, "0")}:${String(wholeSeconds).padStart(2, "0")}.${tenths}`;
}

function basename(path) {
  return String(path || "").split(/[\\/]/).pop() || "";
}

function getInputVideoUrl(record) {
  if (!record?.job_id) {
    return null;
  }
  if (record.input_video_url) {
    return record.input_video_url;
  }
  if (record.input_video) {
    return `/jobs/${record.job_id}/input/${basename(record.input_video)}`;
  }
  return null;
}

function buildSceneIndex(scene) {
  const sortedKeyframes = [...scene.reconstruction.keyframes]
    .sort((left, right) => left.timestamp_s - right.timestamp_s);
  const keyframeById = new Map(sortedKeyframes.map((keyframe) => [keyframe.keyframe_id, keyframe]));
  const detectionsByKeyframe = new Map();
  const detectionById = new Map();
  scene.detections.forEach((detection) => {
    detectionById.set(detection.detection_id, detection);
    const items = detectionsByKeyframe.get(detection.keyframe_id) || [];
    items.push(detection);
    detectionsByKeyframe.set(detection.keyframe_id, items);
  });
  detectionsByKeyframe.forEach((items) => items.sort((left, right) => right.confidence - left.confidence));

  const observationById = new Map();
  const observationByDetectionId = new Map();
  scene.observations.forEach((observation) => {
    observationById.set(observation.observation_id, observation);
    const existing = observationByDetectionId.get(observation.detection_id);
    if (!existing || observation.confidence > existing.confidence) {
      observationByDetectionId.set(observation.detection_id, observation);
    }
  });

  const entityById = new Map(scene.entities.map((entity) => [entity.entity_id, entity]));
  const entityByObservationId = new Map();
  const entityByDetectionId = new Map();
  const observationsByEntityKeyframe = new Map();
  scene.entities.forEach((entity) => {
    entity.observation_ids.forEach((observationId) => {
      const observation = observationById.get(observationId);
      if (!observation) {
        return;
      }
      entityByObservationId.set(observationId, entity);
      const detectionContext = entityByDetectionId.get(observation.detection_id);
      if (!detectionContext || observation.confidence > detectionContext.observation.confidence) {
        entityByDetectionId.set(observation.detection_id, { entity, observation });
      }
      const byKeyframe = observationsByEntityKeyframe.get(entity.entity_id) || new Map();
      const observations = byKeyframe.get(observation.keyframe_id) || [];
      observations.push(observation);
      observations.sort((left, right) => right.confidence - left.confidence);
      byKeyframe.set(observation.keyframe_id, observations);
      observationsByEntityKeyframe.set(entity.entity_id, byKeyframe);
    });
  });

  const relationsByEntity = new Map();
  scene.relations.forEach((relation) => {
    if (entityById.has(relation.subject)) {
      const items = relationsByEntity.get(relation.subject) || [];
      items.push(relation);
      relationsByEntity.set(relation.subject, items);
    }
    if (entityById.has(relation.object)) {
      const items = relationsByEntity.get(relation.object) || [];
      items.push(relation);
      relationsByEntity.set(relation.object, items);
    }
  });

  return {
    sortedKeyframes,
    keyframeById,
    detectionsByKeyframe,
    detectionById,
    observationById,
    observationByDetectionId,
    entityById,
    entityByObservationId,
    entityByDetectionId,
    observationsByEntityKeyframe,
    relationsByEntity,
  };
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function getVideoDuration() {
  const measuredDuration = elements.inspectionVideo?.duration;
  if (Number.isFinite(measuredDuration) && measuredDuration > 0) {
    return measuredDuration;
  }
  return sceneIndex?.sortedKeyframes.at(-1)?.timestamp_s || 0;
}

function getNearestKeyframeForTime(time) {
  if (!sceneIndex?.sortedKeyframes.length) {
    return null;
  }
  let nearest = sceneIndex.sortedKeyframes[0];
  let bestDelta = Math.abs(nearest.timestamp_s - time);
  for (const keyframe of sceneIndex.sortedKeyframes) {
    const delta = Math.abs(keyframe.timestamp_s - time);
    if (delta < bestDelta) {
      nearest = keyframe;
      bestDelta = delta;
    }
  }
  return nearest;
}

function getPrimaryObservationForEntityKeyframe(entityId, keyframeId) {
  return sceneIndex?.observationsByEntityKeyframe.get(entityId)?.get(keyframeId)?.[0] || null;
}

function getVideoContentRect() {
  const video = elements.inspectionVideo;
  if (!video || !video.clientWidth || !video.clientHeight) {
    return null;
  }
  const sourceWidth = video.videoWidth || sceneIndex?.sortedKeyframes[0]?.width || 1;
  const sourceHeight = video.videoHeight || sceneIndex?.sortedKeyframes[0]?.height || 1;
  const scale = Math.min(video.clientWidth / sourceWidth, video.clientHeight / sourceHeight);
  const width = sourceWidth * scale;
  const height = sourceHeight * scale;
  return {
    left: (video.clientWidth - width) * 0.5,
    top: (video.clientHeight - height) * 0.5,
    width,
    height,
    sourceWidth,
    sourceHeight,
  };
}

function getDetectionLayout(detection, keyframe, frameRect) {
  const actualWidth = Math.max(0, detection.bbox.x2 - detection.bbox.x1) * (frameRect.width / keyframe.width);
  const actualHeight = Math.max(0, detection.bbox.y2 - detection.bbox.y1) * (frameRect.height / keyframe.height);
  const width = Math.max(24, actualWidth);
  const height = Math.max(24, actualHeight);
  let left = frameRect.left + (detection.bbox.x1 * (frameRect.width / keyframe.width)) - ((width - actualWidth) * 0.5);
  let top = frameRect.top + (detection.bbox.y1 * (frameRect.height / keyframe.height)) - ((height - actualHeight) * 0.5);
  left = clamp(left, frameRect.left, frameRect.left + frameRect.width - width);
  top = clamp(top, frameRect.top, frameRect.top + frameRect.height - height);
  return {
    left,
    top,
    width,
    height,
    centerX: left + (width * 0.5),
    centerY: top + (height * 0.5),
    actualWidth,
    actualHeight,
  };
}

function queueVideoInspectorRender() {
  window.requestAnimationFrame(() => {
    renderVideoInspector();
  });
}

function syncVideoSource() {
  const video = elements.inspectionVideo;
  if (!video) {
    return;
  }
  const nextSource = currentVideoUrl || "";
  if (video.dataset.sourceUrl === nextSource) {
    return;
  }
  video.pause();
  video.dataset.sourceUrl = nextSource;
  currentDetectionId = null;
  if (nextSource) {
    video.src = nextSource;
  } else {
    video.removeAttribute("src");
  }
  video.load();
  queueVideoInspectorRender();
}

function renderGraphPanel() {
  if (!elements.graphCanvas || !elements.graphStats || !elements.graphMeta || !elements.graphRelations) {
    return;
  }
  if (!currentScene) {
    elements.graphMeta.textContent = "No scene";
    elements.graphCanvas.innerHTML = "";
    elements.graphRelations.textContent = "relations: none";
    elements.graphStats.textContent = "detections: none";
    return;
  }

  const selected = currentScene.entities.find((item) => item.entity_id === currentEntityId) || null;
  const entities = [...currentScene.entities].sort(compareByLabelAndId);
  const places = [...currentScene.places].sort((left, right) => String(left.place_id).localeCompare(String(right.place_id)));

  const width = 1100;
  const height = 720;
  const centerX = width * 0.5;
  const centerY = height * 0.5;
  const positions = new Map();
  const edgeParts = [];
  const nodeParts = [];

  if (places.length === 1) {
    positions.set(places[0].place_id, { x: centerX, y: centerY });
  } else {
    places.forEach((place, index) => {
      const angle = (-Math.PI / 2) + (index / Math.max(places.length, 1)) * Math.PI * 2;
      positions.set(place.place_id, {
        x: centerX + Math.cos(angle) * 120,
        y: centerY + Math.sin(angle) * 84,
      });
    });
  }

  const entityRadiusX = Math.max(240, width * 0.36);
  const entityRadiusY = Math.max(188, height * 0.28);
  entities.forEach((entity, index) => {
    const angle = (-Math.PI / 2) + (index / Math.max(entities.length, 1)) * Math.PI * 2;
    positions.set(entity.entity_id, {
      x: centerX + Math.cos(angle) * entityRadiusX,
      y: centerY + Math.sin(angle) * entityRadiusY,
    });
  });

  const relationGroups = new Map();
  currentScene.relations.forEach((relation) => {
    const groupKey = `${relation.subject}->${relation.object}`;
    const group = relationGroups.get(groupKey) || {
      subject: relation.subject,
      object: relation.object,
      predicates: [],
      confidenceTotal: 0,
    };
    group.predicates.push(relation.predicate);
    group.confidenceTotal += relation.confidence || 0;
    relationGroups.set(groupKey, group);
  });

  const showEdgeLabels = relationGroups.size <= 28;
  relationGroups.forEach((group) => {
    const from = positions.get(group.subject);
    const to = positions.get(group.object);
    if (!from || !to) {
      return;
    }
    const midX = (from.x + to.x) * 0.5;
    const midY = (from.y + to.y) * 0.5;
    const isSelectedRelation = selected && (group.subject === selected.entity_id || group.object === selected.entity_id);
    const predicates = [...new Set(group.predicates)];
    const edgeStroke = isSelectedRelation ? "rgba(216,109,53,0.42)" : "rgba(57,73,89,0.16)";
    const edgeWidth = isSelectedRelation ? 2.4 : Math.min(2.2, 0.9 + predicates.length * 0.14);
    edgeParts.push(
      `<line x1="${from.x}" y1="${from.y}" x2="${to.x}" y2="${to.y}" stroke="${edgeStroke}" stroke-width="${edgeWidth}" />`,
    );
    if (showEdgeLabels) {
      const predicateSummary = predicates.slice(0, 2).join(", ");
      const suffix = predicates.length > 2 ? ` +${predicates.length - 2}` : "";
      edgeParts.push(
        `<text x="${midX}" y="${midY - 8}" fill="#62707d" font-size="10" text-anchor="middle">${escapeHtml(`${predicateSummary}${suffix}`)}</text>`,
      );
    }
  });

  places.forEach((place) => {
    const pos = positions.get(place.place_id);
    if (!pos) {
      return;
    }
    nodeParts.push(`
      <g data-place-id="${escapeHtml(place.place_id)}" aria-hidden="true">
        <rect x="${pos.x - 62}" y="${pos.y - 24}" width="124" height="48" rx="18" fill="rgba(132,163,255,0.18)" stroke="rgba(88,121,255,0.28)" stroke-width="1.5" />
        <text x="${pos.x}" y="${pos.y - 3}" fill="#3350a8" font-size="14" font-weight="700" text-anchor="middle">${escapeHtml(place.place_id)}</text>
        <text x="${pos.x}" y="${pos.y + 14}" fill="#5270bc" font-size="11" text-anchor="middle">${escapeHtml(`${place.member_entities.length} members`)}</text>
      </g>
    `);
  });

  entities.forEach((entity) => {
    const pos = positions.get(entity.entity_id);
    if (!pos) {
      return;
    }
    const isSelected = entity.entity_id === selected?.entity_id;
    const fill = isSelected ? "#d86d35" : "rgba(255,255,255,0.96)";
    const stroke = isSelected ? "#b75425" : "rgba(57,73,89,0.18)";
    const textColor = isSelected ? "#fffaf6" : "#20303d";
    nodeParts.push(`
      <g data-entity-id="${escapeHtml(entity.entity_id)}" style="cursor:pointer">
        <circle cx="${pos.x}" cy="${pos.y}" r="${isSelected ? 42 : 35}" fill="${fill}" stroke="${stroke}" stroke-width="2" />
        <text x="${pos.x}" y="${pos.y - 5}" fill="${textColor}" font-size="15" font-weight="700" text-anchor="middle">${escapeHtml(entity.canonical_label)}</text>
        <text x="${pos.x}" y="${pos.y + 14}" fill="${textColor}" font-size="11" text-anchor="middle">${escapeHtml(entity.entity_id)}</text>
      </g>
    `);
  });

  elements.graphCanvas.innerHTML = [...edgeParts, ...nodeParts].join("");
  elements.graphMeta.textContent = [
    `${entities.length} entities`,
    `${places.length} places`,
    `${currentScene.relations.length} relations`,
    `${currentScene.detections.length} detections`,
    selected ? `selected ${selected.entity_id}` : "",
  ].filter(Boolean).join(" · ");
  elements.graphCanvas.querySelectorAll("[data-entity-id]").forEach((node) => {
    node.addEventListener("click", () => {
      currentEntityId = node.getAttribute("data-entity-id");
      viewer.setSelectedEntity(currentEntityId);
      renderInspector();
      renderGraphPanel();
    });
  });

  const detectionCounts = currentScene.detections.reduce((accumulator, detection) => {
    accumulator[detection.detector_label] = (accumulator[detection.detector_label] || 0) + 1;
    return accumulator;
  }, {});
  const detectionLines = Object.entries(detectionCounts)
    .sort((left, right) => right[1] - left[1])
    .map(([label, count]) => `${label.padEnd(12, " ")} ${String(count).padStart(3, " ")}`);
  const entityLines = entities.map((entity) => {
    const placeLabel = entity.place_id || "-";
    return `${entity.entity_id.padEnd(12, " ")} ${entity.canonical_label.padEnd(14, " ")} obs ${String(entity.observation_count).padStart(3, " ")} place ${placeLabel}`;
  });
  const placeLines = places.map((place) => `${place.place_id.padEnd(12, " ")} members ${String(place.member_entities.length).padStart(3, " ")} confidence ${place.confidence.toFixed(2)}`);
  const relationLines = [...currentScene.relations]
    .sort((left, right) => (
      String(left.subject).localeCompare(String(right.subject))
      || String(left.predicate).localeCompare(String(right.predicate))
      || String(left.object).localeCompare(String(right.object))
    ))
    .map((relation) => `${relation.subject.padEnd(12, " ")} ${relation.predicate.padEnd(12, " ")} ${relation.object.padEnd(12, " ")} ${relation.confidence.toFixed(2)}`);

  elements.graphRelations.textContent = relationLines.length ? relationLines.join("\n") : "relations: none";
  elements.graphStats.textContent = [
    "entities",
    entityLines.length ? entityLines.join("\n") : "none",
    "",
    "places",
    placeLines.length ? placeLines.join("\n") : "none",
    "",
    "detections",
    detectionLines.length ? detectionLines.join("\n") : "none",
  ].join("\n");
}

function setVideoTransportState() {
  const duration = getVideoDuration();
  const currentTime = Number.isFinite(elements.inspectionVideo.currentTime) ? elements.inspectionVideo.currentTime : 0;
  const hasVideo = Boolean(currentScene && currentVideoUrl && duration > 0);
  elements.videoPlayToggle.disabled = !hasVideo;
  elements.videoPrevKeyframe.disabled = !sceneIndex?.sortedKeyframes.length;
  elements.videoNextKeyframe.disabled = !sceneIndex?.sortedKeyframes.length;
  elements.videoScrubber.disabled = !hasVideo;
  elements.videoScrubber.max = String(duration || 0);
  elements.videoScrubber.value = String(clamp(currentTime, 0, duration || 0));
  elements.videoPlayToggle.textContent = hasVideo && !elements.inspectionVideo.paused ? "Pause" : "Play";
  elements.videoTime.textContent = `${formatTime(currentTime)} / ${formatTime(duration)}`;
}

function buildVisibleRelationData(selectedDetectionId, activeKeyframeId, overlayMap) {
  const selectedContext = sceneIndex?.entityByDetectionId.get(selectedDetectionId) || null;
  if (!selectedContext?.entity) {
    return {
      selectedEntity: null,
      visibleLinks: [],
      relationItems: [],
      relatedDetectionIds: new Set(),
    };
  }

  const relationItems = [];
  const visibleLinksByDetection = new Map();
  const entityRelations = sceneIndex.relationsByEntity.get(selectedContext.entity.entity_id) || [];
  entityRelations.forEach((relation) => {
    const statement = `${relation.subject} ${relation.predicate} ${relation.object}`;
    const item = {
      statement,
      confidence: relation.confidence,
      visible: false,
    };
    relationItems.push(item);

    let targetEntityId = null;
    if (relation.subject === selectedContext.entity.entity_id && sceneIndex.entityById.has(relation.object)) {
      targetEntityId = relation.object;
    } else if (relation.object === selectedContext.entity.entity_id && sceneIndex.entityById.has(relation.subject)) {
      targetEntityId = relation.subject;
    }
    if (!targetEntityId) {
      return;
    }

    const targetObservation = getPrimaryObservationForEntityKeyframe(targetEntityId, activeKeyframeId);
    const targetOverlay = targetObservation ? overlayMap.get(targetObservation.detection_id) : null;
    if (!targetOverlay) {
      return;
    }

    item.visible = true;
    const group = visibleLinksByDetection.get(targetObservation.detection_id) || {
      targetObservation,
      targetEntity: sceneIndex.entityById.get(targetEntityId),
      predicates: [],
      confidence: 0,
    };
    group.predicates.push(relation.predicate);
    group.confidence = Math.max(group.confidence, relation.confidence || 0);
    visibleLinksByDetection.set(targetObservation.detection_id, group);
  });

  return {
    selectedEntity: selectedContext.entity,
    visibleLinks: [...visibleLinksByDetection.values()],
    relationItems: relationItems.sort((left, right) => (
      Number(right.visible) - Number(left.visible)
      || right.confidence - left.confidence
      || left.statement.localeCompare(right.statement)
    )),
    relatedDetectionIds: new Set(visibleLinksByDetection.keys()),
  };
}

function positionVideoCard(anchorLayout) {
  if (!elements.videoCard || !elements.videoFrame || !anchorLayout) {
    return;
  }
  const frameWidth = elements.videoFrame.clientWidth;
  const frameHeight = elements.videoFrame.clientHeight;
  const cardWidth = elements.videoCard.offsetWidth;
  const cardHeight = elements.videoCard.offsetHeight;
  let left = anchorLayout.left + anchorLayout.width + 14;
  if (left + cardWidth > frameWidth - 12) {
    left = anchorLayout.left - cardWidth - 14;
  }
  left = clamp(left, 12, Math.max(12, frameWidth - cardWidth - 12));

  let top = anchorLayout.top;
  if (top + cardHeight > frameHeight - 12) {
    top = frameHeight - cardHeight - 12;
  }
  top = clamp(top, 12, Math.max(12, frameHeight - cardHeight - 12));
  elements.videoCard.style.left = `${left}px`;
  elements.videoCard.style.top = `${top}px`;
  elements.videoCard.style.visibility = "visible";
}

function renderVideoCard(selectedOverlay, activeKeyframe, relationData) {
  if (!selectedOverlay) {
    elements.videoCard.hidden = true;
    elements.videoCard.style.visibility = "hidden";
    return;
  }

  const observation = sceneIndex?.observationByDetectionId.get(selectedOverlay.detection.detection_id) || null;
  const entityContext = sceneIndex?.entityByDetectionId.get(selectedOverlay.detection.detection_id) || null;
  const entity = entityContext?.entity || null;
  const visibleLinks = relationData.visibleLinks.length;
  const relationItems = relationData.relationItems.slice(0, 8);
  const hiddenRelationCount = Math.max(0, relationData.relationItems.length - relationItems.length);

  elements.videoCardTitle.textContent = entity
    ? `${entity.canonical_label} · ${entity.entity_id}`
    : selectedOverlay.detection.detector_label;
  elements.videoCardSubtitle.textContent = [
    `${selectedOverlay.detection.detector_label} detection`,
    `${activeKeyframe.keyframe_id} @ ${formatTime(activeKeyframe.timestamp_s)}`,
    entity ? entity.state : "not grounded to entity",
  ].join(" · ");

  elements.videoCardMetrics.innerHTML = [
    ["Detection", selectedOverlay.detection.detection_id],
    ["Confidence", formatPercent(selectedOverlay.detection.confidence)],
    ["Observation", observation?.observation_id || "Unlinked"],
    ["Entity", entity?.entity_id || "Unresolved"],
    ["Visible Links", String(visibleLinks)],
    ["Place", entity?.place_id || "None"],
  ]
    .map(([label, value]) => (
      `<div><dt>${escapeHtml(label)}</dt><dd>${escapeHtml(value)}</dd></div>`
    ))
    .join("");

  elements.videoCardRelations.innerHTML = relationItems.length
    ? [
      ...relationItems.map((item) => (
        `<li>${escapeHtml(item.statement)} · ${escapeHtml(formatPercent(item.confidence))}${item.visible ? " · visible in frame" : ""}</li>`
      )),
      hiddenRelationCount ? `<li>${hiddenRelationCount} more relations not shown</li>` : "",
    ].filter(Boolean).join("")
    : "<li>No semantic links for this selection yet.</li>";

  elements.videoCard.hidden = false;
  elements.videoCard.style.visibility = "hidden";
  window.requestAnimationFrame(() => {
    positionVideoCard(selectedOverlay.layout);
  });
}

function renderVideoInspector() {
  setVideoTransportState();

  if (!currentScene || !sceneIndex) {
    elements.videoOverlays.innerHTML = "";
    elements.videoRelations.innerHTML = "";
    elements.videoKeyframe.textContent = "No keyframe";
    elements.videoDetectionCount.textContent = "0 detections";
    elements.videoEmpty.textContent = "Load a processed scene to inspect detections over the source video.";
    elements.videoEmpty.hidden = false;
    renderVideoCard(null, null, { visibleLinks: [], relationItems: [] });
    return;
  }

  const currentTime = Number.isFinite(elements.inspectionVideo.currentTime) ? elements.inspectionVideo.currentTime : 0;
  const activeKeyframe = getNearestKeyframeForTime(currentTime) || sceneIndex.sortedKeyframes[0];
  const detections = sceneIndex.detectionsByKeyframe.get(activeKeyframe.keyframe_id) || [];
  const frameRect = getVideoContentRect();

  elements.videoKeyframe.textContent = `${activeKeyframe.keyframe_id} @ ${formatTime(activeKeyframe.timestamp_s)}`;
  elements.videoDetectionCount.textContent = `${detections.length} detections`;

  if (currentDetectionId && !detections.some((item) => item.detection_id === currentDetectionId)) {
    currentDetectionId = null;
  }

  if (!currentVideoUrl) {
    elements.videoOverlays.innerHTML = "";
    elements.videoRelations.innerHTML = "";
    elements.videoEmpty.textContent = "Source video unavailable for this job.";
    elements.videoEmpty.hidden = false;
    renderVideoCard(null, null, { visibleLinks: [], relationItems: [] });
    return;
  }

  if (!frameRect) {
    elements.videoOverlays.innerHTML = "";
    elements.videoRelations.innerHTML = "";
    elements.videoEmpty.textContent = "Loading video…";
    elements.videoEmpty.hidden = false;
    renderVideoCard(null, null, { visibleLinks: [], relationItems: [] });
    return;
  }

  const overlayItems = detections.map((detection) => {
    const entityContext = sceneIndex.entityByDetectionId.get(detection.detection_id) || null;
    return {
      detection,
      entity: entityContext?.entity || null,
      observation: sceneIndex.observationByDetectionId.get(detection.detection_id) || null,
      layout: getDetectionLayout(detection, activeKeyframe, frameRect),
    };
  });
  const overlayMap = new Map(overlayItems.map((item) => [item.detection.detection_id, item]));
  const relationData = currentDetectionId
    ? buildVisibleRelationData(currentDetectionId, activeKeyframe.keyframe_id, overlayMap)
    : {
      selectedEntity: null,
      visibleLinks: [],
      relationItems: [],
      relatedDetectionIds: new Set(),
    };

  elements.videoOverlays.innerHTML = overlayItems.map((item) => {
    const isSelected = item.detection.detection_id === currentDetectionId;
    const isRelated = relationData.relatedDetectionIds.has(item.detection.detection_id);
    const label = item.entity ? `${item.entity.canonical_label} · ${item.entity.entity_id}` : item.detection.detector_label;
    const showLabel = isSelected || isRelated || item.layout.width >= 52 || item.layout.height >= 34;
    return `
      <button
        type="button"
        class="detection-box${isSelected ? " is-selected" : ""}${isRelated ? " is-related" : ""}"
        data-detection-id="${escapeHtml(item.detection.detection_id)}"
        style="left:${item.layout.left}px;top:${item.layout.top}px;width:${item.layout.width}px;height:${item.layout.height}px"
        aria-pressed="${isSelected ? "true" : "false"}"
        aria-label="${escapeHtml(`${label}, ${formatPercent(item.detection.confidence)}, ${activeKeyframe.keyframe_id}`)}"
      >
        ${showLabel ? `<span class="detection-box-label">${escapeHtml(label)}</span>` : ""}
      </button>
    `;
  }).join("");

  elements.videoOverlays.querySelectorAll("[data-detection-id]").forEach((button) => {
    button.addEventListener("click", () => {
      const detectionId = button.getAttribute("data-detection-id");
      currentDetectionId = detectionId;
      const selection = sceneIndex.entityByDetectionId.get(detectionId);
      currentEntityId = selection?.entity?.entity_id || null;
      viewer.setSelectedEntity(currentEntityId);
      renderInspector();
      renderGraphPanel();
      renderVideoInspector();
    });
  });

  elements.videoRelations.setAttribute("viewBox", `0 0 ${elements.inspectionVideo.clientWidth} ${elements.inspectionVideo.clientHeight}`);
  elements.videoRelations.innerHTML = relationData.visibleLinks.map((link) => {
    const from = overlayMap.get(currentDetectionId);
    const to = overlayMap.get(link.targetObservation.detection_id);
    if (!from || !to) {
      return "";
    }
    const midX = (from.layout.centerX + to.layout.centerX) * 0.5;
    const midY = (from.layout.centerY + to.layout.centerY) * 0.5;
    const label = [...new Set(link.predicates)].join(" · ");
    const labelWidth = Math.max(56, (label.length * 6.7) + 16);
    return `
      <line
        x1="${from.layout.centerX}"
        y1="${from.layout.centerY}"
        x2="${to.layout.centerX}"
        y2="${to.layout.centerY}"
        stroke="rgba(216, 109, 53, 0.78)"
        stroke-width="2.2"
        stroke-linecap="round"
      />
      <rect
        x="${midX - (labelWidth * 0.5)}"
        y="${midY - 18}"
        width="${labelWidth}"
        height="20"
        rx="10"
        fill="rgba(255,255,255,0.94)"
        stroke="rgba(216, 109, 53, 0.18)"
      />
      <text
        x="${midX}"
        y="${midY - 4}"
        fill="#b75425"
        font-size="11"
        font-weight="700"
        text-anchor="middle"
      >${escapeHtml(label)}</text>
    `;
  }).join("");

  elements.videoEmpty.hidden = detections.length > 0;
  if (!detections.length) {
    elements.videoEmpty.textContent = "No detections on the nearest keyframe.";
  }
  renderVideoCard(currentDetectionId ? overlayMap.get(currentDetectionId) : null, activeKeyframe, relationData);
}

function renderInspector() {
  if (!currentScene || !currentEntityId) {
    elements.inspector.innerHTML = "<p>Select an entity in the scene, video, or graph.</p>";
    return;
  }
  const entity = currentScene.entities.find((item) => item.entity_id === currentEntityId);
  if (!entity) {
    elements.inspector.innerHTML = "<p>Selected entity is not available in the current scene.</p>";
    return;
  }
  const relations = currentScene.relations.filter(
    (relation) => relation.subject === entity.entity_id || relation.object === entity.entity_id,
  );
  const observations = currentScene.observations.filter((item) => entity.observation_ids.includes(item.observation_id));
  const place = currentScene.places.find((item) => item.place_id === entity.place_id);
  const relationSummary = relations.length
    ? relations.map((relation) => `${relation.subject} ${relation.predicate} ${relation.object}`).join("\n")
    : "No relation evidence for this entity yet.";
  elements.inspector.innerHTML = `
    <div>
      <strong>${entity.entity_id}</strong>
      <div class="entity-meta">${entity.canonical_label} · ${entity.state} · ${entity.mobility}</div>
    </div>
    <pre>${JSON.stringify(
      {
        pose: entity.world_pose_estimate,
        extent_3d: entity.extent_3d,
        place_id: entity.place_id,
        observations: entity.observation_count,
        support_points: entity.support_point_count,
        observed_in_keyframes: entity.observed_in_keyframes,
        place_extent_3d: place?.extent_3d || null,
        observation_support: observations.map((item) => ({
          observation_id: item.observation_id,
          keyframe_id: item.keyframe_id,
          support_point_count: item.support_point_count,
        })),
      },
      null,
      2,
    )}</pre>
    <div>
      <strong>Relations</strong>
      <pre>${relationSummary}</pre>
    </div>
  `;
}

async function loadSceneGraph(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error("Failed to load scene graph.");
  }
  currentScene = await response.json();
  sceneIndex = buildSceneIndex(currentScene);
  currentEntityId = currentScene.entities[0]?.entity_id || null;
  currentDetectionId = null;
  const frameUrls = {};
  if (currentJobId) {
    currentScene.reconstruction.cameras.forEach((camera) => {
      frameUrls[camera.image_name] = `/jobs/${currentJobId}/work/images/${camera.image_name}`;
    });
  }
  updateSummary(currentScene);
  renderInspector();
  renderGraphPanel();
  syncVideoSource();
  renderVideoInspector();
  viewer.setScene(currentScene);
  viewer.setFrameImageUrls(frameUrls);
  viewer.setSelectedEntity(currentEntityId);
  const diagnostics = currentScene.diagnostics || {};
  elements.caption.textContent = `${formatTitleCase(currentScene.scene_profile)} scene · ${formatTitleCase(currentScene.yolo_model_size || diagnostics.yolo_model_size || "small")} model · ${currentScene.entities.length} entities · ${diagnostics.observations || currentScene.observations.length} grounded observations`;
}

async function pollJob(jobId) {
  while (true) {
    const response = await fetch(`/api/jobs/${jobId}`);
    if (!response.ok) {
      throw new Error("Failed to query job state.");
    }
    const record = await response.json();
    setJobState(record);
    if (record.status === "completed" && record.scene_graph_url) {
      await loadSceneGraph(record.scene_graph_url);
      return;
    }
    if (record.status === "failed") {
      return;
    }
    await new Promise((resolve) => window.setTimeout(resolve, 1500));
  }
}

async function loadAppConfig() {
  const response = await fetch("/api/config");
  if (!response.ok) {
    throw new Error("Failed to load app configuration.");
  }
  appConfig = await response.json();
  const presets = Object.entries(appConfig.prompt_presets || {});
  elements.preset.innerHTML = presets
    .map(
      ([key, details]) =>
        `<option value="${key}">${details.label || formatTitleCase(key)}</option>`,
    )
    .join("");
  elements.preset.value = appConfig.default_prompt_preset || presets[0]?.[0] || "general";

  const modelEntries = Object.entries(appConfig.yolo_world_models || {});
  elements.modelSize.innerHTML = modelEntries
    .map(([key, details]) => `<option value="${key}">${details.label || formatTitleCase(key)}</option>`)
    .join("");
  elements.modelSize.value = appConfig.default_yolo_model_size || modelEntries[0]?.[0] || "small";
  updatePresetNote();
  updateModelNote();
}

elements.preset.addEventListener("change", updatePresetNote);
elements.preset.addEventListener("change", () => markDirty(true));
elements.modelSize.addEventListener("change", updateModelNote);
elements.modelSize.addEventListener("change", () => markDirty(true));
elements.file.addEventListener("change", () => {
  clearFileError();
  markDirty(true);
});
elements.prompts.addEventListener("input", () => markDirty(true));
elements.prompts.addEventListener("keydown", (event) => {
  if ((event.metaKey || event.ctrlKey) && event.key === "Enter") {
    event.preventDefault();
    elements.form.requestSubmit();
  }
});
elements.videoCardClose.addEventListener("click", () => {
  currentDetectionId = null;
  renderVideoInspector();
});
elements.videoPlayToggle.addEventListener("click", async () => {
  if (elements.inspectionVideo.paused) {
    await elements.inspectionVideo.play();
  } else {
    elements.inspectionVideo.pause();
  }
  setVideoTransportState();
});
elements.videoPrevKeyframe.addEventListener("click", () => {
  if (!sceneIndex?.sortedKeyframes.length) {
    return;
  }
  const current = getNearestKeyframeForTime(elements.inspectionVideo.currentTime || 0) || sceneIndex.sortedKeyframes[0];
  const index = sceneIndex.sortedKeyframes.findIndex((item) => item.keyframe_id === current.keyframe_id);
  const nextKeyframe = sceneIndex.sortedKeyframes[Math.max(0, index - 1)];
  elements.inspectionVideo.currentTime = nextKeyframe.timestamp_s;
  renderVideoInspector();
});
elements.videoNextKeyframe.addEventListener("click", () => {
  if (!sceneIndex?.sortedKeyframes.length) {
    return;
  }
  const current = getNearestKeyframeForTime(elements.inspectionVideo.currentTime || 0) || sceneIndex.sortedKeyframes[0];
  const index = sceneIndex.sortedKeyframes.findIndex((item) => item.keyframe_id === current.keyframe_id);
  const nextKeyframe = sceneIndex.sortedKeyframes[Math.min(sceneIndex.sortedKeyframes.length - 1, index + 1)];
  elements.inspectionVideo.currentTime = nextKeyframe.timestamp_s;
  renderVideoInspector();
});
elements.videoScrubber.addEventListener("input", () => {
  elements.inspectionVideo.currentTime = Number(elements.videoScrubber.value || 0);
  renderVideoInspector();
});
["loadedmetadata", "timeupdate", "seeked", "play", "pause", "ended"].forEach((eventName) => {
  elements.inspectionVideo.addEventListener(eventName, () => {
    renderVideoInspector();
  });
});
window.addEventListener("resize", queueVideoInspectorRender);

elements.form.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!elements.file.files?.length) {
    setFileError("Choose a video file before starting a job.");
    elements.file.focus();
    return;
  }
  clearFileError();

  const payload = new FormData();
  payload.append("file", elements.file.files[0]);
  payload.append("preset", elements.preset.value);
  payload.append("model_size", elements.modelSize.value);
  payload.append("prompts", elements.prompts.value.trim());

  setSubmitLoading(true);
  currentJobId = null;
  setJobState(null);
  elements.caption.textContent = "Uploading video and starting semantic mapping job.";
  markDirty(false);

  try {
    const response = await fetch("/api/jobs", {
      method: "POST",
      body: payload,
    });
    if (!response.ok) {
      throw new Error("Failed to create processing job.");
    }
    const record = await response.json();
    setJobState(record);
    await pollJob(record.job_id);
  } catch (error) {
    elements.jobError.textContent = error.message;
  } finally {
    setSubmitLoading(false);
  }
});

window.addEventListener("beforeunload", (event) => {
  if (!hasUnsavedChanges) {
    return;
  }
  event.preventDefault();
  event.returnValue = "";
});

function handleTabKeydown(event) {
  const currentIndex = paneOrder.indexOf(activePane);
  let nextIndex = currentIndex;
  switch (event.key) {
    case "ArrowRight":
    case "ArrowDown":
      nextIndex = (currentIndex + 1) % paneOrder.length;
      break;
    case "ArrowLeft":
    case "ArrowUp":
      nextIndex = (currentIndex - 1 + paneOrder.length) % paneOrder.length;
      break;
    case "Home":
      nextIndex = 0;
      break;
    case "End":
      nextIndex = paneOrder.length - 1;
      break;
    default:
      return;
  }
  event.preventDefault();
  const nextPane = paneOrder[nextIndex];
  setActivePane(nextPane, { updateHistory: true });
  ({ "3d": elements.tab3d, video: elements.tabVideo, graph: elements.tabGraph }[nextPane] || elements.tab3d).focus();
}

elements.tab3d.addEventListener("click", () => {
  setActivePane("3d", { updateHistory: true });
});

elements.tabVideo.addEventListener("click", () => {
  setActivePane("video", { updateHistory: true });
});
elements.tabGraph.addEventListener("click", () => {
  setActivePane("graph", { updateHistory: true });
});
elements.tab3d.addEventListener("keydown", handleTabKeydown);
elements.tabVideo.addEventListener("keydown", handleTabKeydown);
elements.tabGraph.addEventListener("keydown", handleTabKeydown);
window.addEventListener("popstate", () => {
  setActivePane(getPaneFromUrl());
});

updateSummary({
  scene_profile: "general",
  prompt_preset: "general",
  reconstruction: { keyframes: [], cameras: [], sparse_points: [] },
  observations: [],
  entities: [],
  places: [],
  relations: [],
  diagnostics: {},
});

updateRotationValue();
syncViewerHelp();
renderGraphPanel();
renderVideoInspector();
setActivePane(getPaneFromUrl(), { updateHistory: true, replaceHistory: true });

loadAppConfig().catch((error) => {
  elements.preset.innerHTML = '<option value="general">General Mixed</option>';
  elements.preset.value = "general";
  elements.presetNote.textContent = error.message;
  elements.modelSize.innerHTML = '<option value="small">Small</option>';
  elements.modelSize.value = "small";
  elements.modelNote.textContent = "Model sizes unavailable.";
});
