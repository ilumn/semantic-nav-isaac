const COLORS = {
  path: "rgba(114,208,186,0.38)",
  cameras: "#72d0ba",
  frustums: "rgba(114,208,186,0.28)",
  entities: "#f5a46a",
  entityBox: "rgba(245,164,106,0.42)",
  places: "#84a3ff",
  placeExtent: "rgba(132,163,255,0.18)",
  relations: "rgba(226,233,242,0.18)",
  relationSelected: "rgba(255,213,138,0.58)",
  support: "rgba(255,240,160,0.96)",
  supportHalo: "rgba(255,240,160,0.22)",
  labels: "rgba(243,247,252,0.92)",
  axes: "rgba(232,238,244,0.14)",
  selected: "#fff0a0",
};

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function subtract(left, right) {
  return [
    left[0] - right[0],
    left[1] - right[1],
    left[2] - right[2],
  ];
}

function dot(left, right) {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

function normalizeVector(vector, fallback = [0, 0, 1]) {
  const length = Math.hypot(vector[0], vector[1], vector[2]);
  if (length <= 1e-6) {
    return [...fallback];
  }
  return [vector[0] / length, vector[1] / length, vector[2] / length];
}

function directionToYawPitch(direction) {
  const normalized = normalizeVector(direction);
  return {
    yaw: Math.atan2(normalized[0], normalized[2]),
    pitch: Math.asin(clamp(normalized[1], -1, 1)),
  };
}

function normalizeAngle(angle) {
  const fullTurn = Math.PI * 2;
  let normalized = angle % fullTurn;
  if (normalized <= -Math.PI) {
    normalized += fullTurn;
  }
  if (normalized > Math.PI) {
    normalized -= fullTurn;
  }
  return normalized;
}

function isEditableTarget(target) {
  return Boolean(target && (target.closest("input, textarea, select, button") || target.isContentEditable));
}

function addScaled(base, direction, scale) {
  return [
    base[0] + direction[0] * scale,
    base[1] + direction[1] * scale,
    base[2] + direction[2] * scale,
  ];
}

function rgba(color, alpha = 1) {
  if (!Array.isArray(color) || color.length < 3) {
    return `rgba(214,224,236,${alpha})`;
  }
  return `rgba(${color[0]},${color[1]},${color[2]},${alpha})`;
}

function ensure4x4(matrix) {
  if (!Array.isArray(matrix) || !matrix.length) {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ];
  }
  if (matrix.length === 4) {
    return matrix;
  }
  return [
    [matrix[0][0] ?? 1, matrix[0][1] ?? 0, matrix[0][2] ?? 0, matrix[0][3] ?? 0],
    [matrix[1][0] ?? 0, matrix[1][1] ?? 1, matrix[1][2] ?? 0, matrix[1][3] ?? 0],
    [matrix[2][0] ?? 0, matrix[2][1] ?? 0, matrix[2][2] ?? 1, matrix[2][3] ?? 0],
    [0, 0, 0, 1],
  ];
}

function invertRigidMatrix(matrix) {
  const m = ensure4x4(matrix);
  const rotation = [
    [m[0][0], m[0][1], m[0][2]],
    [m[1][0], m[1][1], m[1][2]],
    [m[2][0], m[2][1], m[2][2]],
  ];
  const translation = [m[0][3], m[1][3], m[2][3]];
  const rotationT = [
    [rotation[0][0], rotation[1][0], rotation[2][0]],
    [rotation[0][1], rotation[1][1], rotation[2][1]],
    [rotation[0][2], rotation[1][2], rotation[2][2]],
  ];
  const worldTranslation = [
    -(rotationT[0][0] * translation[0] + rotationT[0][1] * translation[1] + rotationT[0][2] * translation[2]),
    -(rotationT[1][0] * translation[0] + rotationT[1][1] * translation[1] + rotationT[1][2] * translation[2]),
    -(rotationT[2][0] * translation[0] + rotationT[2][1] * translation[1] + rotationT[2][2] * translation[2]),
  ];
  return { rotation: rotationT, translation: worldTranslation };
}

function transformPoint(rotation, translation, point) {
  return [
    rotation[0][0] * point[0] + rotation[0][1] * point[1] + rotation[0][2] * point[2] + translation[0],
    rotation[1][0] * point[0] + rotation[1][1] * point[1] + rotation[1][2] * point[2] + translation[1],
    rotation[2][0] * point[0] + rotation[2][1] * point[1] + rotation[2][2] * point[2] + translation[2],
  ];
}

export class PointCloudViewer {
  constructor(canvas, callbacks = {}) {
    this.canvas = canvas;
    this.canvas.tabIndex = 0;
    this.ctx = canvas.getContext("2d");
    this.onSelect = callbacks.onSelect ?? null;
    this.onStateChange = callbacks.onStateChange ?? null;
    this.scene = null;
    this.selectedEntityId = null;
    this.options = {
      points: true,
      path: true,
      cameras: true,
      frustums: true,
      frames: true,
      entities: true,
      places: true,
      relations: true,
      labels: true,
      support: true,
    };
    this.frameImages = new Map();
    this.frameDrawLimit = 12;
    this.sceneScale = 12;
    this.pointLookup = new Map();
    this.supportPointLookup = new Map();
    this.defaultView = {
      yaw: -0.65,
      pitch: 0.45,
      roll: 0,
      distance: 12,
      target: [0, 0, 0],
    };
    this.state = {
      yaw: this.defaultView.yaw,
      pitch: this.defaultView.pitch,
      roll: this.defaultView.roll,
      distance: this.defaultView.distance,
      target: [...this.defaultView.target],
      dragging: false,
      dragMode: null,
      moved: false,
      lastX: 0,
      lastY: 0,
    };

    this.resize = this.resize.bind(this);
    this.render = this.render.bind(this);
    this.handlePointerDown = this.handlePointerDown.bind(this);
    this.handlePointerMove = this.handlePointerMove.bind(this);
    this.handlePointerUp = this.handlePointerUp.bind(this);
    this.handleWheel = this.handleWheel.bind(this);
    this.handleClick = this.handleClick.bind(this);
    this.handleKeyDown = this.handleKeyDown.bind(this);
    this.handleContextMenu = this.handleContextMenu.bind(this);

    window.addEventListener("resize", this.resize);
    window.addEventListener("pointermove", this.handlePointerMove);
    window.addEventListener("pointerup", this.handlePointerUp);
    window.addEventListener("keydown", this.handleKeyDown);
    canvas.addEventListener("pointerdown", this.handlePointerDown);
    canvas.addEventListener("wheel", this.handleWheel, { passive: false });
    canvas.addEventListener("click", this.handleClick);
    canvas.addEventListener("contextmenu", this.handleContextMenu);
    this.resize();
    this.emitStateChange();
  }

  emitStateChange() {
    if (!this.onStateChange) {
      return;
    }
    this.onStateChange({
      rollDegrees: this.getRollDegrees(),
      hint: this.getShortcutHint(),
    });
  }

  getShortcutHint() {
    return "Left drag orbit. Shift+Left drag pan. Ctrl/Cmd+Left drag zoom. Wheel zooms. 1/3/7 snap views.";
  }

  getInteractionState() {
    return {
      rollDegrees: this.getRollDegrees(),
      hint: this.getShortcutHint(),
    };
  }

  setOptions(nextOptions) {
    this.options = { ...this.options, ...nextOptions };
    this.render();
  }

  setScene(scene) {
    this.scene = scene;
    this.pointLookup.clear();
    this.supportPointLookup.clear();

    const points = [];
    for (const point of scene.reconstruction.sparse_points) {
      this.pointLookup.set(point.point_id, point);
      points.push(point.xyz);
    }
    for (const point of scene.reconstruction.support_points || []) {
      this.supportPointLookup.set(point.point_id, point);
      points.push(point.xyz);
    }
    for (const camera of scene.reconstruction.cameras) {
      points.push(camera.center);
    }
    for (const entity of scene.entities) {
      points.push(entity.world_pose_estimate);
    }

    if (!points.length) {
      this.render();
      return;
    }

    const bounds = points.reduce(
      (acc, point) => {
        point.forEach((value, index) => {
          acc.min[index] = Math.min(acc.min[index], value);
          acc.max[index] = Math.max(acc.max[index], value);
        });
        return acc;
      },
      { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] },
    );
    const center = bounds.min.map((value, index) => (value + bounds.max[index]) * 0.5);
    const diagonal = Math.hypot(
      bounds.max[0] - bounds.min[0],
      bounds.max[1] - bounds.min[1],
      bounds.max[2] - bounds.min[2],
    );
    this.sceneScale = diagonal > 0 ? diagonal : 12;
    const initialCamera = scene.reconstruction.cameras[0] || null;
    if (initialCamera?.center?.length === 3 && initialCamera?.view_direction?.length === 3) {
      const cameraCenter = initialCamera.center.map(Number);
      const forward = normalizeVector(initialCamera.view_direction);
      const centerOffset = Math.max(dot(subtract(center, cameraCenter), forward), this.sceneScale * 0.22, 1.4);
      const { yaw, pitch } = directionToYawPitch(forward);
      this.defaultView = {
        yaw,
        pitch,
        roll: 0,
        distance: centerOffset,
        target: addScaled(cameraCenter, forward, centerOffset),
      };
    } else {
      this.defaultView = {
        yaw: -0.65,
        pitch: 0.4,
        roll: this.state.roll,
        distance: this.sceneScale > 0 ? this.sceneScale * 1.28 : 12,
        target: center,
      };
    }
    this.state.target = [...center];
    this.resetView();
    this.render();
  }

  resetView() {
    this.state.yaw = this.defaultView.yaw;
    this.state.pitch = this.defaultView.pitch;
    this.state.roll = this.defaultView.roll;
    this.state.distance = this.defaultView.distance;
    this.state.target = [...this.defaultView.target];
    this.emitStateChange();
    this.render();
  }

  setFrameImageUrls(frameUrls) {
    this.frameImages.clear();
    for (const [imageName, url] of Object.entries(frameUrls || {})) {
      const image = new Image();
      image.loading = "lazy";
      image.decoding = "async";
      image.onload = () => this.render();
      image.src = url;
      this.frameImages.set(imageName, image);
    }
    this.render();
  }

  setSelectedEntity(entityId) {
    this.selectedEntityId = entityId;
    this.render();
  }

  rotateLeftQuarter() {
    this.state.roll = normalizeAngle(this.state.roll - Math.PI / 2);
    this.emitStateChange();
    this.render();
  }

  rotateRightQuarter() {
    this.state.roll = normalizeAngle(this.state.roll + Math.PI / 2);
    this.emitStateChange();
    this.render();
  }

  resetRoll() {
    this.state.roll = 0;
    this.emitStateChange();
    this.render();
  }

  getRollDegrees() {
    return Math.round((this.state.roll * 180) / Math.PI);
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.render();
  }

  getRotationMatrix() {
    const { yaw, pitch, roll } = this.state;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const cp = Math.cos(pitch);
    const sp = Math.sin(pitch);
    const cr = Math.cos(roll);
    const sr = Math.sin(roll);

    return [
      [cr * cy - sr * sp * sy, -sr * cp, -cr * sy - sr * sp * cy],
      [sr * cy + cr * sp * sy, cr * cp, -sr * sy + cr * sp * cy],
      [cp * sy, sp, cp * cy],
    ];
  }

  viewToWorld(vector) {
    const matrix = this.getRotationMatrix();
    const [x, y, z] = vector;
    return [
      matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z,
      matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z,
      matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z,
    ];
  }

  panByPixels(dx, dy) {
    const scale = Math.max(this.state.distance, this.sceneScale) * 0.0021;
    const right = this.viewToWorld([1, 0, 0]);
    const up = this.viewToWorld([0, 1, 0]);
    this.state.target = addScaled(this.state.target, right, -dx * scale);
    this.state.target = addScaled(this.state.target, up, dy * scale);
  }

  panDepth(direction) {
    const forward = this.viewToWorld([0, 0, 1]);
    const amount = Math.max(this.state.distance, this.sceneScale) * 0.04 * direction;
    this.state.target = addScaled(this.state.target, forward, amount);
  }

  dollyBy(delta) {
    const factor = Math.exp(delta * 0.0025);
    this.state.distance = clamp(this.state.distance * factor, 0.16, 5000);
  }

  resolvePointerMode(event) {
    const leftLike = event.button === 0 || event.button === 1;
    if (!leftLike) {
      return null;
    }
    if (event.shiftKey) {
      return "Pan";
    }
    if (event.ctrlKey || event.metaKey) {
      return "Zoom";
    }
    return "Orbit";
  }

  handlePointerDown(event) {
    this.canvas.focus({ preventScroll: true });
    this.state.dragMode = this.resolvePointerMode(event);
    this.state.dragging = Boolean(this.state.dragMode);
    this.state.moved = false;
    this.state.lastX = event.clientX;
    this.state.lastY = event.clientY;
    if (this.state.dragging) {
      event.preventDefault();
    }
    this.emitStateChange();
  }

  handlePointerMove(event) {
    if (!this.state.dragging || !this.state.dragMode) {
      return;
    }
    const dx = event.clientX - this.state.lastX;
    const dy = event.clientY - this.state.lastY;
    this.state.lastX = event.clientX;
    this.state.lastY = event.clientY;
    if (Math.abs(dx) + Math.abs(dy) > 2) {
      this.state.moved = true;
    }

    if (this.state.dragMode === "Orbit") {
      this.state.yaw += dx * 0.006;
      this.state.pitch = clamp(this.state.pitch + dy * 0.006, -1.35, 1.35);
    } else if (this.state.dragMode === "Pan") {
      this.panByPixels(dx, dy);
    } else if (this.state.dragMode === "Zoom") {
      this.dollyBy(dy * 3.2);
    }

    this.emitStateChange();
    this.render();
  }

  handlePointerUp() {
    this.state.dragging = false;
    this.state.dragMode = null;
    this.emitStateChange();
  }

  handleWheel(event) {
    event.preventDefault();
    this.dollyBy(event.deltaY);
    this.render();
  }

  snapView(view) {
    if (view === "front") {
      this.state.yaw = 0;
      this.state.pitch = 0;
    } else if (view === "right") {
      this.state.yaw = Math.PI / 2;
      this.state.pitch = 0;
    } else if (view === "top") {
      this.state.yaw = 0;
      this.state.pitch = -Math.PI / 2 + 0.01;
    }
    this.state.roll = 0;
    this.emitStateChange();
    this.render();
  }

  handleContextMenu(event) {
    event.preventDefault();
  }

  handleKeyDown(event) {
    if (event.metaKey || event.ctrlKey || isEditableTarget(event.target)) {
      return;
    }

    let handled = true;
    const key = event.key.toLowerCase();
    switch (key) {
      case "1":
        this.snapView("front");
        break;
      case "3":
        this.snapView("right");
        break;
      case "7":
        this.snapView("top");
        break;
      case "r":
        this.resetView();
        break;
      case "[":
        this.rotateLeftQuarter();
        break;
      case "]":
        this.rotateRightQuarter();
        break;
      default:
        handled = false;
        break;
    }

    if (handled) {
      event.preventDefault();
      this.emitStateChange();
      this.render();
    }
  }

  handleClick(event) {
    if (!this.scene || !this.scene.entities.length || this.state.moved) {
      return;
    }
    const rect = this.canvas.getBoundingClientRect();
    const mouseX = event.clientX - rect.left;
    const mouseY = event.clientY - rect.top;
    let best = null;
    let bestDistance = Infinity;
    for (const entity of this.scene.entities) {
      const projected = this.project(entity.world_pose_estimate, rect.width, rect.height);
      if (!projected) {
        continue;
      }
      const distance = Math.hypot(projected.x - mouseX, projected.y - mouseY);
      if (distance < bestDistance && distance <= 16) {
        bestDistance = distance;
        best = entity.entity_id;
      }
    }
    if (best && this.onSelect) {
      this.onSelect(best);
    }
  }

  rotate(point) {
    const [tx, ty, tz] = this.state.target;
    const x = point[0] - tx;
    const y = point[1] - ty;
    const z = point[2] - tz;

    const matrix = this.getRotationMatrix();
    return [
      matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
      matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
      matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + this.state.distance,
    ];
  }

  project(point, width, height) {
    const [x, y, z] = this.rotate(point);
    if (z <= 0.001) {
      return null;
    }
    const focal = Math.min(width, height) * 0.72;
    return {
      x: width * 0.5 + (x * focal) / z,
      y: height * 0.5 - (y * focal) / z,
      z,
    };
  }

  drawMarker(projected, color, radius) {
    this.ctx.beginPath();
    this.ctx.fillStyle = color;
    this.ctx.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
    this.ctx.fill();
  }

  drawPixel(projected, color, size) {
    const half = size * 0.5;
    this.ctx.fillStyle = color;
    this.ctx.fillRect(projected.x - half, projected.y - half, size, size);
  }

  drawLine(a, b, color, width = 1) {
    this.ctx.beginPath();
    this.ctx.strokeStyle = color;
    this.ctx.lineWidth = width;
    this.ctx.moveTo(a.x, a.y);
    this.ctx.lineTo(b.x, b.y);
    this.ctx.stroke();
  }

  drawAxis(width, height) {
    const size = Math.max(1, this.sceneScale * 0.06);
    const axes = [
      { from: [-size, 0, 0], to: [size, 0, 0] },
      { from: [0, -size, 0], to: [0, size, 0] },
      { from: [0, 0, -size], to: [0, 0, size] },
    ];
    axes.forEach((axis) => {
      const start = this.project(axis.from, width, height);
      const end = this.project(axis.to, width, height);
      if (start && end) {
        this.drawLine(start, end, COLORS.axes, 1);
      }
    });
  }

  drawLabel(text, x, y, color = COLORS.labels) {
    this.ctx.fillStyle = color;
    this.ctx.font = '12px "Avenir Next", sans-serif';
    this.ctx.fillText(text, x, y);
  }

  getEntityById(entityId) {
    return this.scene?.entities?.find((entity) => entity.entity_id === entityId) || null;
  }

  drawFrameBillboard(projected, image) {
    const scale = clamp(this.sceneScale / Math.max(projected.z, this.sceneScale * 0.24), 0.48, 1);
    const maxWidth = 84 * scale;
    const maxHeight = 62 * scale;
    const aspect = image.naturalWidth && image.naturalHeight ? image.naturalWidth / image.naturalHeight : 16 / 9;
    let drawWidth = maxWidth;
    let drawHeight = drawWidth / aspect;
    if (drawHeight > maxHeight) {
      drawHeight = maxHeight;
      drawWidth = drawHeight * aspect;
    }

    const left = projected.x - drawWidth * 0.5;
    const top = projected.y - drawHeight - 12;
    this.ctx.save();
    this.ctx.globalAlpha = 0.95;
    this.ctx.fillStyle = "rgba(7, 10, 14, 0.82)";
    this.ctx.fillRect(left - 3, top - 3, drawWidth + 6, drawHeight + 6);
    this.ctx.drawImage(image, left, top, drawWidth, drawHeight);
    this.ctx.strokeStyle = "rgba(255,255,255,0.65)";
    this.ctx.lineWidth = 1;
    this.ctx.strokeRect(left - 1.5, top - 1.5, drawWidth + 3, drawHeight + 3);
    this.ctx.restore();
    this.drawLine(
      { x: projected.x, y: top + drawHeight + 3 },
      { x: projected.x, y: projected.y },
      "rgba(255,255,255,0.16)",
      1,
    );
  }

  projectBox(center, extent, width, height) {
    const half = extent.map((value) => Math.max(value, this.sceneScale * 0.008) * 0.5);
    const corners = [];
    for (const dx of [-half[0], half[0]]) {
      for (const dy of [-half[1], half[1]]) {
        for (const dz of [-half[2], half[2]]) {
          corners.push([center[0] + dx, center[1] + dy, center[2] + dz]);
        }
      }
    }
    const projected = corners.map((corner) => this.project(corner, width, height));
    if (projected.some((item) => !item)) {
      return null;
    }
    return projected;
  }

  drawBox(projectedCorners, color, width = 1) {
    const edges = [
      [0, 1], [0, 2], [0, 4],
      [1, 3], [1, 5],
      [2, 3], [2, 6],
      [3, 7],
      [4, 5], [4, 6],
      [5, 7],
      [6, 7],
    ];
    edges.forEach(([from, to]) => {
      this.drawLine(projectedCorners[from], projectedCorners[to], color, width);
    });
  }

  drawCameraFrustum(camera, width, height) {
    const intrinsics = camera.intrinsics || {};
    const cameraWidth = Number(intrinsics.width || 0);
    const cameraHeight = Number(intrinsics.height || 0);
    const fx = Number(intrinsics.fx || 0);
    const fy = Number(intrinsics.fy || 0);
    const cx = Number(intrinsics.cx || cameraWidth * 0.5);
    const cy = Number(intrinsics.cy || cameraHeight * 0.5);
    if (!cameraWidth || !cameraHeight || !fx || !fy) {
      return;
    }

    const inverse = invertRigidMatrix(camera.cam_from_world);
    const depth = Math.max(this.sceneScale * 0.04, 0.15);
    const localCorners = [
      [((0 - cx) / fx) * depth, ((0 - cy) / fy) * depth, depth],
      [((cameraWidth - cx) / fx) * depth, ((0 - cy) / fy) * depth, depth],
      [((cameraWidth - cx) / fx) * depth, ((cameraHeight - cy) / fy) * depth, depth],
      [((0 - cx) / fx) * depth, ((cameraHeight - cy) / fy) * depth, depth],
    ];
    const center = this.project(camera.center, width, height);
    if (!center) {
      return;
    }
    const corners = localCorners
      .map((corner) => transformPoint(inverse.rotation, inverse.translation, corner))
      .map((point) => this.project(point, width, height));
    if (corners.some((corner) => !corner)) {
      return;
    }

    corners.forEach((corner) => this.drawLine(center, corner, COLORS.frustums, 1));
    this.drawLine(corners[0], corners[1], COLORS.frustums, 1);
    this.drawLine(corners[1], corners[2], COLORS.frustums, 1);
    this.drawLine(corners[2], corners[3], COLORS.frustums, 1);
    this.drawLine(corners[3], corners[0], COLORS.frustums, 1);
  }

  renderSparsePoints(width, height) {
    if (!this.options.points) {
      return;
    }
    for (const point of this.scene.reconstruction.sparse_points) {
      const projected = this.project(point.xyz, width, height);
      if (!projected) {
        continue;
      }
      const radius = clamp(2.8 / Math.sqrt(projected.z / Math.max(this.sceneScale, 0.001) + 0.2), 0.65, 2.2);
      const color = point.color ? rgba(point.color, 0.78) : "rgba(214,224,236,0.45)";
      this.drawPixel(projected, color, radius);
    }
  }

  renderSupportPoints(width, height) {
    if (!this.options.support) {
      return;
    }
    const selected = this.getEntityById(this.selectedEntityId);
    if (!selected) {
      return;
    }
    const pointIds = new Set(selected.support_point3d_ids || []);
    for (const point of this.scene.reconstruction.support_points || []) {
      if (!pointIds.has(point.point_id)) {
        continue;
      }
      const projected = this.project(point.xyz, width, height);
      if (!projected) {
        continue;
      }
      this.drawMarker(projected, COLORS.supportHalo, 5.4);
      this.drawMarker(projected, point.color ? rgba(point.color, 0.98) : COLORS.support, 2.2);
    }
  }

  renderRelations(width, height) {
    if (!this.options.relations) {
      return;
    }
    const entityById = new Map(this.scene.entities.map((entity) => [entity.entity_id, entity]));
    this.scene.relations.forEach((relation) => {
      const left = entityById.get(relation.subject);
      const right = entityById.get(relation.object);
      if (!left || !right) {
        return;
      }
      const start = this.project(left.world_pose_estimate, width, height);
      const end = this.project(right.world_pose_estimate, width, height);
      if (!start || !end) {
        return;
      }
      const selected = left.entity_id === this.selectedEntityId || right.entity_id === this.selectedEntityId;
      this.drawLine(start, end, selected ? COLORS.relationSelected : COLORS.relations, selected ? 1.6 : 1);
    });
  }

  renderPlaces(width, height) {
    if (!this.options.places) {
      return;
    }
    this.scene.places.forEach((place) => {
      const projected = this.project(place.anchor_pose, width, height);
      if (!projected) {
        return;
      }
      const extent = place.extent_3d?.length ? place.extent_3d : [this.sceneScale * 0.04, this.sceneScale * 0.02, this.sceneScale * 0.04];
      const box = this.projectBox(place.anchor_pose, extent, width, height);
      if (box) {
        this.drawBox(box, COLORS.placeExtent, 1);
      }
      this.ctx.save();
      this.ctx.strokeStyle = COLORS.places;
      this.ctx.lineWidth = 1.2;
      this.ctx.beginPath();
      this.ctx.arc(projected.x, projected.y, 7, 0, Math.PI * 2);
      this.ctx.stroke();
      this.ctx.restore();
      if (this.options.labels) {
        this.drawLabel(place.place_id, projected.x + 9, projected.y - 9, "rgba(198,210,255,0.88)");
      }
    });
  }

  renderEntities(width, height) {
    if (!this.options.entities) {
      return;
    }
    this.scene.entities.forEach((entity) => {
      const projected = this.project(entity.world_pose_estimate, width, height);
      if (!projected) {
        return;
      }
      const isSelected = entity.entity_id === this.selectedEntityId;
      const box = this.projectBox(entity.world_pose_estimate, entity.extent_3d, width, height);
      if (box) {
        this.drawBox(box, isSelected ? "rgba(255,240,160,0.76)" : COLORS.entityBox, isSelected ? 1.8 : 1.1);
      }
      this.drawMarker(projected, isSelected ? COLORS.selected : COLORS.entities, isSelected ? 6 : 4.5);
      if (this.options.labels) {
        this.drawLabel(entity.entity_id, projected.x + 8, projected.y - 8);
      }
    });
  }

  renderCameras(width, height) {
    const projectedCameras = this.scene.reconstruction.cameras
      .map((camera) => ({ ...camera, projected: this.project(camera.center, width, height) }))
      .filter((item) => item.projected);

    if (this.options.path) {
      for (let index = 1; index < projectedCameras.length; index += 1) {
        this.drawLine(projectedCameras[index - 1].projected, projectedCameras[index].projected, COLORS.path, 1.2);
      }
    }

    if (this.options.frustums) {
      projectedCameras.forEach((camera) => this.drawCameraFrustum(camera, width, height));
    }

    if (this.options.cameras) {
      projectedCameras.forEach((camera) => this.drawMarker(camera.projected, COLORS.cameras, 3));
    }

    if (this.options.frames) {
      const frameStep = Math.max(1, Math.ceil(projectedCameras.length / this.frameDrawLimit));
      projectedCameras.forEach((camera, index) => {
        if (index % frameStep !== 0) {
          return;
        }
        const image = this.frameImages.get(camera.image_name);
        if (!image || !image.complete) {
          return;
        }
        this.drawFrameBillboard(camera.projected, image);
      });
    }
  }

  render() {
    const rect = this.canvas.getBoundingClientRect();
    const width = rect.width;
    const height = rect.height;
    this.ctx.clearRect(0, 0, width, height);
    this.drawAxis(width, height);

    if (!this.scene) {
      this.ctx.fillStyle = "rgba(228,236,244,0.58)";
      this.ctx.font = '16px "Avenir Next", sans-serif';
      this.ctx.fillText("No scene loaded.", 28, 38);
      this.emitStateChange();
      return;
    }

    this.renderSparsePoints(width, height);
    this.renderSupportPoints(width, height);
    this.renderRelations(width, height);
    this.renderPlaces(width, height);
    this.renderEntities(width, height);
    this.renderCameras(width, height);

    this.emitStateChange();
  }
}
