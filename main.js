// MAC0623 — A3 Waypoint navigation
//
// Extends the A2 WebXR testbed: same rig / CSV download pattern, new task.
// The participant docks their viewpoint onto a beacon, not a cube onto a pose.
//
// Technique 1 — World-in-Miniature (Class 9).
//   Left grip shows a scaled clone of the world. The orange cone is the head.
//   Right trigger grabs it; the viewpoint jumps only when the trigger is released.
//   The miniature is parented to the left grip with no extra yaw, so turning
//   the wrist turns the map. The magenta cone marks world north (−Z).
//
// Technique 2 — ray teleport. Design draft: design-technique2.txt.
//   Right controller −Z ray ∩ floor, trigger snaps the head there.
//   One egocentric frame, no continuous motion.
//
// Path rule (both techniques): horizontal head travel per frame, so a jump
// counts as a segment of that length and physical walking counts too.
//
// Parameters, stated for the report:
//   Environment: 20 × 20 m. A2 docking bounds were about 2 m across.
//   Beacon spawns at least 6 m from the head (arm reach cannot finish a trial).
//   Confirm radius: 1.0 m horizontal (middle of the 0.75–1.5 m suggestion).
//   The cyan ring on the ground is that radius.

import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

const ENV_SIZE = 20;
const ENV_HALF = ENV_SIZE / 2;
const CONFIRM_RADIUS = 1.0;
const MIN_BEACON_DIST = 6;
const WIM_SCALE = 0.35 / ENV_SIZE;
const MARKER_HEIGHT = 0.08;
const EYE_HEIGHT = 1.6;
const SESSION_LENGTH = 4;

const participantIdInput = document.getElementById("participantId");
const orderSelect = document.getElementById("orderSelect");
const techniqueSelect = document.getElementById("techniqueSelect");
const startSessionBtn = document.getElementById("startSessionBtn");
const trialCountEl = document.getElementById("trialCount");
const confirmBtn = document.getElementById("confirmBtn");
const downloadBtn = document.getElementById("downloadBtn");
const statusEl = document.getElementById("status");
const liveErrorsEl = document.getElementById("liveErrors");

const CSV_HEADER = [
  "participant_id",
  "technique",
  "trial_number",
  "presentation_order",
  "completion_time_s",
  "path_length",
  "straight_line_distance",
  "path_ratio",
];

const rows = [];
const _head = new THREE.Vector3();
const _forward = new THREE.Vector3();
const _aimPoint = new THREE.Vector3();
const _raycaster = new THREE.Raycaster();
const _rayMatrix = new THREE.Matrix4();
const lastHead = new THREE.Vector3();

let scene, camera, renderer;
let world, floorMesh, beacon, beaconBulbMat, beaconRingMat;
let xrRig, mini, marker;
let leftGrip = null;
const controllers = [];

let technique = 1;
let markerDragging = false;
let markerDragController = null;
let desktopDrag = false;
let leftGripHeld = false;
let teleportPoint = null;

let logging = false;
let sessionActive = false;
let sessionComplete = false;
let sessionOrder = [];
let sessionIndex = 0;
let trialCountByTechnique = { 1: 0, 2: 0 };
let straightLineDistance = 0;
let pathLength = 0;
let pathReady = false;
let trialStartTime = 0;
let lastConfirmAt = 0;
let pendingTrialReset = false;

let yaw = 0;
let pitch = 0;
let lookDragging = false;
let xrExitWasDown = false;
let aWasDown = false;

let vrHud = null;

function participantId() {
  return (participantIdInput.value.trim() || "UNKNOWN").replace(/,/g, "");
}

function withinRadius() {
  return distanceToBeacon() <= CONFIRM_RADIUS;
}

function distanceToBeacon() {
  camera.getWorldPosition(_head);
  return Math.hypot(_head.x - beacon.position.x, _head.z - beacon.position.z);
}

function isLeft(obj) {
  if (!obj) return false;
  if (obj.userData.handedness === "left") return true;
  if (obj.userData.handedness === "right") return false;
  return obj.userData.index === 0;
}

function buildWorld() {
  world = new THREE.Group();
  world.name = "world";

  floorMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(ENV_SIZE, ENV_SIZE),
    new THREE.MeshStandardMaterial({ color: 0x3d4a3f, roughness: 1 })
  );
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.name = "floor";
  world.add(floorMesh);

  world.add(new THREE.GridHelper(ENV_SIZE, 20, 0x6a746c, 0x2a332c));

  const curbMat = new THREE.MeshStandardMaterial({ color: 0x9aa59a });
  const curbH = 0.08;
  const curbT = 0.12;
  const curbs = [
    [0, -ENV_HALF, ENV_SIZE, curbT],
    [0, ENV_HALF, ENV_SIZE, curbT],
    [-ENV_HALF, 0, curbT, ENV_SIZE],
    [ENV_HALF, 0, curbT, ENV_SIZE],
  ];
  for (const [x, z, w, d] of curbs) {
    const curb = new THREE.Mesh(new THREE.BoxGeometry(w, curbH, d), curbMat);
    curb.position.set(x, curbH / 2, z);
    world.add(curb);
  }

  const pillar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.7, 0.8, 5, 16),
    new THREE.MeshStandardMaterial({ color: 0xd12b2b })
  );
  pillar.position.set(-6, 2.5, -6);
  pillar.name = "pillar";
  world.add(pillar);

  const archMat = new THREE.MeshStandardMaterial({ color: 0x2f6fed });
  const arch = new THREE.Group();
  arch.name = "arch";
  const columnGeom = new THREE.CylinderGeometry(0.28, 0.28, 3.2, 12);
  const leftCol = new THREE.Mesh(columnGeom, archMat);
  leftCol.position.set(-1.3, 1.6, 0);
  const rightCol = new THREE.Mesh(columnGeom, archMat);
  rightCol.position.set(1.3, 1.6, 0);
  const lintel = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.45, 0.55), archMat);
  lintel.position.set(0, 3.35, 0);
  arch.add(leftCol, rightCol, lintel);
  arch.position.set(6, 0, -2);
  world.add(arch);

  const wall = new THREE.Mesh(
    new THREE.BoxGeometry(6, 3, 0.45),
    new THREE.MeshStandardMaterial({ color: 0xf0d030 })
  );
  wall.position.set(-1, 1.5, 6.5);
  wall.name = "wall";
  world.add(wall);

  beaconBulbMat = new THREE.MeshStandardMaterial({
    color: 0xe9ffff,
    emissive: 0x22ddff,
    emissiveIntensity: 0.9,
  });
  beaconRingMat = new THREE.MeshBasicMaterial({
    color: 0x22ddff,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.9,
  });

  beacon = new THREE.Group();
  beacon.name = "beacon";
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 3.2, 10),
    new THREE.MeshStandardMaterial({
      color: 0x087080,
      emissive: 0x0a6a78,
      emissiveIntensity: 0.6,
    })
  );
  pole.position.y = 1.6;
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.28, 16, 12), beaconBulbMat);
  bulb.position.y = 3.35;
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(CONFIRM_RADIUS - 0.06, CONFIRM_RADIUS, 48),
    beaconRingMat
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.04;
  beacon.add(pole, bulb, ring);
  world.add(beacon);
}

function buildMini() {
  mini = world.clone(true);
  mini.name = "wim";
  const clonedLights = [];
  mini.traverse((obj) => {
    if (obj.isLight) clonedLights.push(obj);
    obj.castShadow = false;
    obj.receiveShadow = false;
  });
  clonedLights.forEach((light) => light.removeFromParent());
  mini.scale.setScalar(WIM_SCALE);

  const markerGeom = new THREE.ConeGeometry(MARKER_HEIGHT / 2.2, MARKER_HEIGHT, 10);
  markerGeom.translate(0, MARKER_HEIGHT / 2, 0);
  const markerMat = new THREE.MeshBasicMaterial({ color: 0xff8800 });
  const body = new THREE.Mesh(markerGeom, markerMat);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.012, 0.012, 0.045),
    markerMat
  );
  nose.position.set(0, MARKER_HEIGHT * 0.45, 0.03);
  marker = new THREE.Group();
  marker.name = "userMarker";
  marker.add(body, nose);
  // Undo the miniature shrink so the cone is MARKER_HEIGHT tall in the hand.
  marker.scale.setScalar(1 / WIM_SCALE);
  mini.add(marker);

  const northGeom = new THREE.ConeGeometry(0.025, 0.06, 8);
  northGeom.translate(0, 0.03, 0);
  const north = new THREE.Mesh(
    northGeom,
    new THREE.MeshBasicMaterial({ color: 0xff2bd6 })
  );
  north.scale.setScalar(1 / WIM_SCALE);
  north.position.set(0, 0, -ENV_HALF + 1.2);
  north.name = "north";
  mini.add(north);
}

function mountMiniDesktop() {
  camera.add(mini);
  mini.position.set(0.22, -0.18, -0.6);
  mini.quaternion.identity();
  mini.visible = technique === 1;
}

function attachMiniToLeftHand() {
  if (!leftGrip) return;
  leftGrip.add(mini);
  mini.position.set(0, 0.12, -0.05);
  mini.quaternion.identity();
  mini.visible = leftGripHeld && technique === 1;
}

function refreshMiniVisibility() {
  if (!renderer.xr.isPresenting) {
    mini.visible = technique === 1;
    return;
  }
  mini.visible = technique === 1 && leftGripHeld;
}

function syncMiniBeacon() {
  const cloned = mini.getObjectByName("beacon");
  if (!cloned) return;
  cloned.position.copy(beacon.position);
  cloned.quaternion.copy(beacon.quaternion);
}

function syncMarker() {
  if (markerDragging || marker.parent !== mini) return;
  camera.getWorldPosition(_head);
  world.worldToLocal(_head);
  marker.position.set(_head.x, 0, _head.z);
  camera.getWorldDirection(_forward);
  marker.rotation.y = Math.atan2(_forward.x, _forward.z);
}

function moveHeadTo(x, z) {
  camera.getWorldPosition(_head);
  xrRig.position.x += x - _head.x;
  xrRig.position.z += z - _head.z;
}

function releaseMarker() {
  mini.attach(marker);
  mini.updateWorldMatrix(true, true);
  marker.scale.setScalar(1 / WIM_SCALE);
  const limit = ENV_HALF - 0.4;
  marker.position.y = 0;
  marker.position.x = THREE.MathUtils.clamp(marker.position.x, -limit, limit);
  marker.position.z = THREE.MathUtils.clamp(marker.position.z, -limit, limit);
  const target = world.localToWorld(marker.position.clone());
  moveHeadTo(target.x, target.z);
  markerDragging = false;
  desktopDrag = false;
  markerDragController = null;
}

function parkMarker() {
  if (marker.parent !== mini) mini.attach(marker);
  marker.scale.setScalar(1 / WIM_SCALE);
  marker.position.y = 0;
  markerDragging = false;
  desktopDrag = false;
  markerDragController = null;
}

function tryGrabMarker(controller) {
  if (!mini.visible || technique !== 1) return;
  const hits = controllerRaycast(controller, [marker]);
  if (hits.length === 0) return;
  markerDragging = true;
  markerDragController = controller;
  controller.attach(marker);
}

function controllerRaycast(controller, objects) {
  _raycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
  _rayMatrix.identity().extractRotation(controller.matrixWorld);
  _raycaster.ray.direction.set(0, 0, -1).applyMatrix4(_rayMatrix);
  return _raycaster.intersectObjects(objects, true);
}

function rightController() {
  return (
    controllers.find((controller) => controller.userData.handedness === "right") ||
    controllers[1] ||
    null
  );
}

function setControllerRayLength(controller, length) {
  if (!controller) return;
  const ray = controller.getObjectByName("ray");
  if (ray) ray.scale.z = length;
}

function updateTeleportAim() {
  teleportPoint = null;
  const controller = rightController();
  if (!controller || !renderer.xr.isPresenting) return;
  const hits = controllerRaycast(controller, [floorMesh]);
  if (hits.length === 0) {
    setControllerRayLength(controller, 8);
    return;
  }
  const hit = hits[0];
  if (Math.abs(hit.point.x) > ENV_HALF || Math.abs(hit.point.z) > ENV_HALF) {
    setControllerRayLength(controller, 8);
    return;
  }
  teleportPoint = _aimPoint.copy(hit.point);
  setControllerRayLength(controller, Math.max(hit.distance, 0.2));
}

function tryTeleport() {
  if (technique !== 2 || !teleportPoint) return;
  moveHeadTo(teleportPoint.x, teleportPoint.z);
}

function spawnBeacon(fromX, fromZ) {
  const margin = CONFIRM_RADIUS + 0.5;
  const min = -ENV_HALF + margin;
  const max = ENV_HALF - margin;
  let placed = null;
  for (let i = 0; i < 80; i += 1) {
    const x = THREE.MathUtils.randFloat(min, max);
    const z = THREE.MathUtils.randFloat(min, max);
    if (Math.hypot(x - fromX, z - fromZ) >= MIN_BEACON_DIST) {
      placed = { x, z };
      break;
    }
  }
  if (!placed) {
    const dirX = fromX >= 0 ? -1 : 1;
    const dirZ = fromZ >= 0 ? -1 : 1;
    placed = {
      x: THREE.MathUtils.clamp(fromX + dirX * MIN_BEACON_DIST, min, max),
      z: THREE.MathUtils.clamp(fromZ + dirZ * MIN_BEACON_DIST, min, max),
    };
  }
  beacon.position.set(placed.x, 0, placed.z);
  syncMiniBeacon();
}

function startTrial() {
  camera.getWorldPosition(_head);
  spawnBeacon(_head.x, _head.z);
  lastHead.copy(_head);
  pathLength = 0;
  pathReady = true;
  straightLineDistance = Math.hypot(
    _head.x - beacon.position.x,
    _head.z - beacon.position.z
  );
  trialStartTime = performance.now();
  updateTrialLabel();
}

function updatePath() {
  camera.getWorldPosition(_head);
  if (!pathReady) {
    lastHead.copy(_head);
    pathReady = true;
    return;
  }
  pathLength += Math.hypot(_head.x - lastHead.x, _head.z - lastHead.z);
  lastHead.copy(_head);
}

function updateTrialLabel() {
  if (!trialCountEl) return;
  if (sessionActive) {
    const name = technique === 1 ? "WIM" : "Teleport";
    trialCountEl.textContent = `Trial ${sessionIndex + 1}/${SESSION_LENGTH} · ${name}`;
  } else if (sessionComplete) {
    trialCountEl.textContent = "Session complete";
  } else {
    trialCountEl.textContent = `Practice · ${technique === 1 ? "WIM" : "Teleport"}`;
  }
}

function setTechnique(next) {
  technique = next;
  if (techniqueSelect) techniqueSelect.value = String(next);
  refreshMiniVisibility();
  teleportPoint = null;
}

function setSessionUiLocked(locked) {
  orderSelect.disabled = locked;
  techniqueSelect.disabled = locked;
  startSessionBtn.disabled = locked;
  participantIdInput.disabled = locked;
}

function startSession(options = {}) {
  if (sessionActive) return;
  if (!options.force && !renderer.xr.isPresenting) {
    setStatus("Entra en VR antes de Start session. Practice no se guarda.");
    return;
  }
  if (!options.force && rows.length > 0) {
    const ok = window.confirm(
      "Esto borra las filas actuales. Descarga el CSV antes si todavía lo necesitas. ¿Continuar?"
    );
    if (!ok) return;
  }
  const order = options.order || orderSelect.value;
  sessionOrder = order === "BAAB" ? [2, 1, 1, 2] : [1, 2, 2, 1];
  sessionIndex = 0;
  trialCountByTechnique = { 1: 0, 2: 0 };
  rows.length = 0;
  logging = true;
  sessionActive = true;
  sessionComplete = false;
  setSessionUiLocked(true);
  setTechnique(sessionOrder[0]);
  startTrial();
  setStatus("Sesión iniciada.");
}

function confirmTrial() {
  const now = performance.now();
  if (now - lastConfirmAt < 400) return;
  if (sessionComplete) {
    setStatus("Sesión guardada en memoria. Download CSV antes de la siguiente.");
    return;
  }
  if (!withinRadius()) {
    setStatus(`Fuera del radio (${distanceToBeacon().toFixed(2)} m). Entra en el anillo cian.`);
    return;
  }
  lastConfirmAt = now;

  if (!logging) {
    startTrial();
    return;
  }

  const completionTimeS = (now - trialStartTime) / 1000;
  const techniqueId = sessionOrder[sessionIndex];
  trialCountByTechnique[techniqueId] += 1;
  const ratio =
    straightLineDistance > 0 ? pathLength / straightLineDistance : 0;

  rows.push({
    participant_id: participantId(),
    technique: techniqueId,
    trial_number: trialCountByTechnique[techniqueId],
    presentation_order: sessionIndex + 1,
    completion_time_s: completionTimeS.toFixed(3),
    path_length: pathLength.toFixed(4),
    straight_line_distance: straightLineDistance.toFixed(4),
    path_ratio: ratio.toFixed(4),
  });

  sessionIndex += 1;
  if (sessionIndex >= sessionOrder.length) {
    logging = false;
    sessionActive = false;
    sessionComplete = true;
    setSessionUiLocked(false);
    updateTrialLabel();
    setStatus("Sesión completa (4 trials). Download CSV.");
    return;
  }

  setTechnique(sessionOrder[sessionIndex]);
  startTrial();
}

function buildCsv() {
  const lines = [CSV_HEADER.join(",")];
  for (const row of rows) {
    lines.push(CSV_HEADER.map((key) => row[key]).join(","));
  }
  return lines.join("\n");
}

function handleDownloadClick() {
  const csv = buildCsv();
  const blob = new Blob([csv], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `a3_${participantId()}_${Date.now()}.csv`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function setStatus(text) {
  statusEl.textContent = text;
}

function updateStatus() {
  const distance = distanceToBeacon();
  const inside = distance <= CONFIRM_RADIUS;
  beaconBulbMat.emissive.setHex(inside ? 0x44ff66 : 0x22ddff);
  beaconRingMat.color.setHex(inside ? 0x44ff66 : 0x22ddff);

  const name = technique === 1 ? "WIM" : "Teleport";
  const line1 = sessionActive
    ? `${name} · trial ${sessionIndex + 1}/${SESSION_LENGTH}`
    : sessionComplete
      ? "Sesión completa — sal y descarga el CSV"
      : renderer.xr.isPresenting
        ? `Practice · ${name} · pulsa A para grabar`
        : `Practice · ${name}`;
  const line2 = inside
    ? "Dentro del radio — grip derecho para confirmar"
    : `${distance.toFixed(2)} m hasta el faro`;

  statusEl.textContent = `${line1} · ${line2}`;
  statusEl.classList.toggle("in-tolerance", inside);
  if (liveErrorsEl) {
    liveErrorsEl.textContent = `${line1} · ${line2}`;
    liveErrorsEl.classList.toggle("in-tolerance", inside);
    liveErrorsEl.style.display = renderer.xr.isPresenting ? "none" : "";
  }
  updateVrHud(line1, line2, inside);
}

function buildVrHud() {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 128;
  const ctx = canvas.getContext("2d");
  const texture = new THREE.CanvasTexture(canvas);
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(0.46, 0.115),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
  );
  mesh.position.set(0, -0.28, -1.05);
  mesh.renderOrder = 1000;
  camera.add(mesh);
  vrHud = { ctx, texture, canvas, text: "" };
}

function updateVrHud(line1, line2, inside) {
  const text = `${line1}|${line2}|${inside}`;
  if (!vrHud || vrHud.text === text) return;
  vrHud.text = text;
  const ctx = vrHud.ctx;
  ctx.clearRect(0, 0, 512, 128);
  ctx.fillStyle = inside ? "rgba(18, 64, 36, 0.88)" : "rgba(8, 10, 14, 0.78)";
  ctx.fillRect(0, 0, 512, 128);
  ctx.fillStyle = inside ? "#d8ffe4" : "#ffffff";
  ctx.font = "28px sans-serif";
  ctx.fillText(line1, 18, 48);
  ctx.font = "24px sans-serif";
  ctx.fillText(line2, 18, 92);
  vrHud.texture.needsUpdate = true;
}

function applyLook() {
  xrRig.rotation.y = yaw;
  camera.rotation.x = pitch;
  camera.rotation.y = 0;
  camera.rotation.z = 0;
}

function setupController(index, color) {
  const controller = renderer.xr.getController(index);
  const grip = renderer.xr.getControllerGrip(index);
  controller.userData.index = index;
  grip.userData.index = index;
  controller.userData.handedness = index === 0 ? "left" : "right";
  grip.userData.handedness = controller.userData.handedness;

  const ray = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, -1),
    ]),
    new THREE.LineBasicMaterial({ color })
  );
  ray.name = "ray";
  ray.scale.z = index === 0 ? 0.6 : 2;
  controller.add(ray);

  const markerDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.015, 10, 8),
    new THREE.MeshBasicMaterial({ color })
  );
  grip.add(markerDot);

  xrRig.add(controller);
  xrRig.add(grip);
  controllers.push(controller);

  const onConnected = (event) => {
    const handedness = event.data && event.data.handedness;
    if (!handedness) return;
    controller.userData.handedness = handedness;
    grip.userData.handedness = handedness;
    if (handedness === "left") {
      leftGrip = grip;
      if (renderer.xr.isPresenting) attachMiniToLeftHand();
    }
  };
  controller.addEventListener("connected", onConnected);
  grip.addEventListener("connected", onConnected);

  controller.addEventListener("selectstart", onSelectStart);
  controller.addEventListener("selectend", onSelectEnd);
  controller.addEventListener("squeezestart", onSqueezeStart);
  controller.addEventListener("squeezeend", onSqueezeEnd);

  if (index === 0) leftGrip = grip;
  return controller;
}

function onSelectStart(event) {
  const controller = event.target;
  if (isLeft(controller)) return;
  if (technique === 1) tryGrabMarker(controller);
  else tryTeleport();
}

function onSelectEnd(event) {
  if (!markerDragging) return;
  if (markerDragController && event.target !== markerDragController) return;
  releaseMarker();
}

function onSqueezeStart(event) {
  if (isLeft(event.target)) {
    leftGripHeld = true;
    refreshMiniVisibility();
    return;
  }
  confirmTrial();
}

function onSqueezeEnd(event) {
  if (!isLeft(event.target)) return;
  leftGripHeld = false;
  refreshMiniVisibility();
}

function isExitPressed(session) {
  for (const source of session.inputSources) {
    if (source.handedness !== "left") continue;
    const gamepad = source.gamepad;
    if (!gamepad || gamepad.buttons.length <= 4) continue;
    if (gamepad.buttons[4].pressed) return true;
  }
  return false;
}

function isRightADown(xrSession) {
  for (const source of xrSession.inputSources) {
    if (source.handedness !== "right") continue;
    const gamepad = source.gamepad;
    if (!gamepad || gamepad.buttons.length <= 4) continue;
    if (gamepad.buttons[4].pressed) return true;
  }
  return false;
}

function updateVrExit() {
  if (!renderer.xr.isPresenting) {
    xrExitWasDown = false;
    aWasDown = false;
    return;
  }
  const xrSession = renderer.xr.getSession();
  if (!xrSession) return;
  const pressed = isExitPressed(xrSession);
  if (pressed && !xrExitWasDown) xrSession.end();
  xrExitWasDown = pressed;

  const aDown = isRightADown(xrSession);
  if (aDown && !aWasDown && !sessionActive && !sessionComplete && rows.length === 0) {
    startSession();
  }
  aWasDown = aDown;
}

function setupWebXR() {
  renderer.xr.enabled = true;
  document.body.appendChild(VRButton.createButton(renderer));
  setupController(0, 0xff6666);
  setupController(1, 0x66aaff);

  renderer.xr.addEventListener("sessionstart", () => {
    xrRig.rotation.set(0, 0, 0);
    yaw = 0;
    pitch = 0;
    attachMiniToLeftHand();
    leftGripHeld = false;
    pendingTrialReset = true;
  });

  renderer.xr.addEventListener("sessionend", () => {
    camera.position.set(0, EYE_HEIGHT, 0);
    camera.rotation.set(0, 0, 0);
    xrRig.rotation.set(0, 0, 0);
    yaw = 0;
    pitch = 0;
    parkMarker();
    mountMiniDesktop();
    pendingTrialReset = true;
    xrExitWasDown = false;
  });
}

function mouseRay(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );
  _raycaster.setFromCamera(ndc, camera);
}

function onPointerDown(event) {
  if (renderer.xr.isPresenting) return;
  if (event.button === 2) {
    lookDragging = true;
    return;
  }
  if (event.button !== 0) return;
  mouseRay(event);
  if (technique === 1) {
    const markerHits = _raycaster.intersectObject(marker, true);
    const miniFloor = mini.getObjectByName("floor");
    const floorHits = miniFloor ? _raycaster.intersectObject(miniFloor, false) : [];
    if (markerHits.length === 0 && floorHits.length === 0) return;
    markerDragging = true;
    desktopDrag = true;
    if (floorHits.length > 0) {
      const local = mini.worldToLocal(floorHits[0].point.clone());
      marker.position.set(local.x, 0, local.z);
    }
    return;
  }
  const hits = _raycaster.intersectObject(floorMesh, false);
  if (hits.length > 0) moveHeadTo(hits[0].point.x, hits[0].point.z);
}

function onPointerMove(event) {
  if (renderer.xr.isPresenting) return;
  if (lookDragging) {
    yaw -= event.movementX * 0.005;
    pitch = THREE.MathUtils.clamp(pitch - event.movementY * 0.005, -1.2, 1.2);
    return;
  }
  if (!desktopDrag) return;
  mouseRay(event);
  const miniFloor = mini.getObjectByName("floor");
  if (!miniFloor) return;
  const hits = _raycaster.intersectObject(miniFloor, false);
  if (hits.length === 0) return;
  const local = mini.worldToLocal(hits[0].point.clone());
  marker.position.set(local.x, 0, local.z);
}

function onPointerUp(event) {
  if (event.button === 2) lookDragging = false;
  if (event.button === 0 && desktopDrag) releaseMarker();
}

function onKeyDown(event) {
  const tag = event.target && event.target.tagName;
  if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
  if (event.key === "Enter") {
    event.preventDefault();
    confirmTrial();
    return;
  }
  if (renderer.xr.isPresenting) return;
  const step = 0.08;
  if (event.key === "ArrowLeft") yaw += step;
  if (event.key === "ArrowRight") yaw -= step;
  if (event.key === "ArrowUp") pitch = Math.min(1.2, pitch + step);
  if (event.key === "ArrowDown") pitch = Math.max(-1.2, pitch - step);
}

function suggestOrderFromId() {
  if (sessionActive) return;
  const n = parseInt(participantIdInput.value.replace(/\D/g, ""), 10);
  if (!n) return;
  orderSelect.value = n % 2 === 0 ? "BAAB" : "ABBA";
}

function animate() {
  if (pendingTrialReset) {
    pendingTrialReset = false;
    if (!sessionComplete) startTrial();
  }
  updatePath();
  if (technique === 1) syncMarker();
  if (technique === 2 && renderer.xr.isPresenting) updateTeleportAim();
  else teleportPoint = null;
  refreshMiniVisibility();
  updateVrExit();
  if (!renderer.xr.isPresenting) applyLook();
  updateStatus();
  renderer.render(scene, camera);
}

function main() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x9eb4c4);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445544, 1.15));
  const sun = new THREE.DirectionalLight(0xffffff, 0.85);
  sun.position.set(8, 14, 6);
  scene.add(sun);

  buildWorld();
  buildMini();
  scene.add(world);

  camera = new THREE.PerspectiveCamera(
    70,
    window.innerWidth / window.innerHeight,
    0.05,
    200
  );
  camera.position.set(0, EYE_HEIGHT, 0);

  xrRig = new THREE.Group();
  xrRig.name = "xrRig";
  xrRig.add(camera);
  scene.add(xrRig);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = false;
  document.body.appendChild(renderer.domElement);

  buildVrHud();
  mountMiniDesktop();
  setupWebXR();

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  renderer.domElement.addEventListener("pointerdown", onPointerDown);
  window.addEventListener("pointermove", onPointerMove);
  window.addEventListener("pointerup", onPointerUp);
  renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());
  window.addEventListener("keydown", onKeyDown);

  confirmBtn.addEventListener("click", confirmTrial);
  downloadBtn.addEventListener("click", handleDownloadClick);
  startSessionBtn.addEventListener("click", () => startSession());
  techniqueSelect.addEventListener("change", () => {
    if (sessionActive) {
      techniqueSelect.value = String(technique);
      return;
    }
    sessionComplete = false;
    setTechnique(Number(techniqueSelect.value));
    startTrial();
  });
  participantIdInput.addEventListener("change", suggestOrderFromId);
  suggestOrderFromId();

  setTechnique(1);
  startTrial();

  if (new URLSearchParams(location.search).has("selftest")) {
    const result = runSelfTest();
    const pre = document.createElement("pre");
    pre.id = "selftest";
    pre.textContent = result.text;
    if (!result.ok) pre.classList.add("fail");
    document.body.appendChild(pre);
    document.title = result.ok ? "PASS" : "FAIL";
  }

  renderer.setAnimationLoop(animate);
}

function runSelfTest() {
  const fails = [];
  const check = (cond, msg) => {
    if (!cond) fails.push(msg);
  };

  camera.position.set(0.25, EYE_HEIGHT, -0.4);
  xrRig.position.set(0, 0, 0);
  xrRig.rotation.set(0, 0, 0);
  moveHeadTo(3, -2);
  camera.getWorldPosition(_head);
  check(Math.abs(_head.x - 3) < 0.02 && Math.abs(_head.z + 2) < 0.02, `head landing ${_head.x.toFixed(3)},${_head.z.toFixed(3)}`);
  check(Math.abs(xrRig.position.x - 3) > 0.1, "rig origin should not be the head");

  camera.position.set(0, EYE_HEIGHT, 0);
  xrRig.position.set(0, 0, 0);
  marker.position.set(4, 0.2, -3);
  releaseMarker();
  camera.getWorldPosition(_head);
  check(Math.abs(_head.x - 4) < 0.05 && Math.abs(_head.z + 3) < 0.05, `wim release ${_head.x.toFixed(3)},${_head.z.toFixed(3)}`);
  check(Math.abs(marker.scale.x - 1 / WIM_SCALE) < 0.01, `marker scale ${marker.scale.x}`);

  const fake = new THREE.Object3D();
  scene.add(fake);
  fake.position.set(0.1, 1.2, -0.2);
  marker.position.set(1, 0, 2);
  mini.updateWorldMatrix(true, true);
  fake.attach(marker);
  fake.position.x += WIM_SCALE * 2;
  mini.attach(marker);
  marker.scale.setScalar(1 / WIM_SCALE);
  check(Math.abs(marker.position.x - 3) < 0.08, `attach roundtrip x ${marker.position.x.toFixed(3)}`);
  check(Math.abs(marker.position.z - 2) < 0.08, `attach roundtrip z ${marker.position.z.toFixed(3)}`);
  fake.removeFromParent();

  beacon.position.set(1.5, 0, -2.5);
  syncMiniBeacon();
  const clonedBeacon = mini.getObjectByName("beacon");
  check(clonedBeacon && Math.abs(clonedBeacon.position.x - 1.5) < 1e-6, "mini beacon sync");

  xrRig.position.set(0, 0, 0);
  camera.position.set(0, EYE_HEIGHT, 0);
  startTrial();
  camera.getWorldPosition(_head);
  const straight = Math.hypot(_head.x - beacon.position.x, _head.z - beacon.position.z);
  check(straight + 1e-4 >= MIN_BEACON_DIST, `beacon dist ${straight.toFixed(2)}`);
  check(rows.length === 0, "practice must not log");
  lastConfirmAt = 0;
  confirmTrial();
  check(rows.length === 0, "out-of-range confirm must not log");

  const sx = _head.x;
  const sz = _head.z;
  moveHeadTo(sx + 3, sz + 4);
  updatePath();
  check(Math.abs(pathLength - 5) < 0.05, `jump path ${pathLength.toFixed(3)}`);

  startSession({ force: true, order: "ABBA" });
  check(technique === 1 && sessionActive, "ABBA starts on WIM");
  camera.getWorldPosition(_head);
  const expectedStraight = Math.hypot(_head.x - beacon.position.x, _head.z - beacon.position.z);
  moveHeadTo(beacon.position.x, beacon.position.z);
  updatePath();
  lastConfirmAt = 0;
  confirmTrial();
  check(rows.length === 1, "first logged row");
  if (rows[0]) {
    check(rows[0].technique === 1, "row technique");
    check(rows[0].trial_number === 1, "row trial_number");
    check(rows[0].presentation_order === 1, "row presentation_order");
    check(Math.abs(Number(rows[0].straight_line_distance) - expectedStraight) < 0.05, "straight_line");
    const ratio = Number(rows[0].path_ratio);
    const recomputed = Number(rows[0].path_length) / Number(rows[0].straight_line_distance);
    check(Math.abs(ratio - recomputed) < 1e-3, "path_ratio formula");
    check(Math.abs(ratio - 1) < 0.05, `single jump ratio ${ratio}`);
    CSV_HEADER.forEach((key) => check(rows[0][key] !== undefined && rows[0][key] !== "", `missing ${key}`));
  }

  lastConfirmAt = 0;
  confirmTrial();
  check(rows.length === 1, "second confirm outside radius must not log");
  check(technique === 2, "second trial is teleport");

  for (let i = 0; i < 3; i += 1) {
    moveHeadTo(beacon.position.x, beacon.position.z);
    updatePath();
    lastConfirmAt = 0;
    confirmTrial();
  }
  check(rows.length === 4, `expected 4 rows, got ${rows.length}`);
  check(sessionComplete && !sessionActive, "session closes");
  const techniques = rows.map((row) => row.technique).join(",");
  const trials = rows.map((row) => row.trial_number).join(",");
  const order = rows.map((row) => row.presentation_order).join(",");
  check(techniques === "1,2,2,1", `ABBA techniques ${techniques}`);
  check(trials === "1,1,2,2", `trial numbers ${trials}`);
  check(order === "1,2,3,4", `presentation ${order}`);
  check(buildCsv().split("\n")[0] === CSV_HEADER.join(","), "csv header");

  startSession({ force: true, order: "BAAB" });
  check(technique === 2, "BAAB starts on teleport");
  rows.length = 0;
  logging = false;
  sessionActive = false;
  sessionComplete = false;
  setSessionUiLocked(false);

  xrRig.position.set(0, 0, 0);
  xrRig.rotation.set(0, 0, 0);
  camera.position.set(0, EYE_HEIGHT, 0);
  parkMarker();
  setTechnique(1);
  startTrial();

  const text = fails.length === 0 ? "PASS" : `FAIL\n${fails.join("\n")}`;
  return { ok: fails.length === 0, text };
}

window.__A3 = {
  rows,
  startSession,
  confirmTrial,
  moveHeadTo,
  get technique() {
    return technique;
  },
  CONFIRM_RADIUS,
  MIN_BEACON_DIST,
  ENV_SIZE,
};

main();
