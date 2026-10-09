// MAC0623 — A3 Waypoint navigation
//
// Extends the A2 WebXR testbed: same rig / CSV download pattern, new task.
// The participant docks their viewpoint onto a beacon, not a cube onto a pose.
//
// Technique 1 — World-in-Miniature (Class 9).
//   The scaled house sits on the left hand. The red person is the head.
//   Right trigger grabs it; the viewpoint jumps only when the trigger is released.
//   The miniature is parented to the left grip with no extra yaw, so turning
//   the wrist turns the map. The green circle is the beacon.
//
// Technique 2 — left-joystick locomotion.
//   Left thumbstick moves the rig on XZ in the direction the head is facing.
//   No wall collisions. Same path rule as WIM: horizontal head travel, so a
//   WIM jump and a walked path are comparable.
//
// Path rule (both techniques): horizontal head travel per frame, so a jump
// counts as a segment of that length and physical walking counts too.
//
// Parameters, stated for the report:
//   Environment: a 16 × 16 m house (four 8 m rooms) on a 20 × 20 m lot.
//   Beacon is a green floor circle, at least 6 m from the head.
//   Confirm radius: 1.0 m horizontal — the green circle itself.

import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

const ENV_SIZE = 20;
const ENV_HALF = ENV_SIZE / 2;
const CONFIRM_RADIUS = 1.0;
const MIN_BEACON_DIST = 6;
const WIM_SCALE = 0.35 / ENV_SIZE;
const MARKER_HEIGHT = 0.05;
const JOY_SPEED = 2.6;
const JOY_DEAD = 0.15;
const ROOMS = [
  { x: -4, z: -4 },
  { x: 4, z: -4 },
  { x: -4, z: 4 },
  { x: 4, z: 4 },
];
const EYE_HEIGHT = 1.6;

const participantIdInput = document.getElementById("participantId");
const techniqueSelect = document.getElementById("techniqueSelect");
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
const _raycaster = new THREE.Raycaster();
const _rayMatrix = new THREE.Matrix4();
const lastHead = new THREE.Vector3();

let scene, camera, renderer;
let world, floorMesh, beacon;
let xrRig, mini, marker;
let leftGrip = null;
const controllers = [];

let technique = 1;
let markerDragging = false;
let markerDragController = null;
let desktopDrag = false;
const keysDown = new Set();

let trialCountByTechnique = { 1: 0, 2: 0 };
let presentationOrder = 0;
let straightLineDistance = 0;
let pathLength = 0;
let pathReady = false;
let trialStartTime = 0;
let lastConfirmAt = -1e9;
let pendingTrialReset = false;

let yaw = 0;
let pitch = 0;
let lookDragging = false;
let xrExitWasDown = false;

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

  const mats = new Map();
  const mat = (color) => {
    let material = mats.get(color);
    if (!material) {
      material = new THREE.MeshStandardMaterial({ color, roughness: 0.86, metalness: 0.02 });
      mats.set(color, material);
    }
    return material;
  };
  const box = (w, h, d, x, y, z, color) => {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(color));
    mesh.position.set(x, y, z);
    world.add(mesh);
    return mesh;
  };
  const cyl = (radius, height, x, y, z, color, segments = 12) => {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(radius, radius, height, segments),
      mat(color)
    );
    mesh.position.set(x, y, z);
    world.add(mesh);
    return mesh;
  };

  floorMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(ENV_SIZE, ENV_SIZE),
    mat(0x6d9a52)
  );
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.name = "floor";
  world.add(floorMesh);

  // 16 m house, four 8 m rooms. Open ceiling so the miniature stays readable.
  // NW living, NE bedroom, SW kitchen, SE study. Walls sit on x/z = 0 and ±8.
  const WALL_H = 2.55;
  const WALL_T = 0.16;
  const DOOR_W = 1.45;
  const DOOR_H = 2.05;
  const HALF = 8;
  const headerH = WALL_H - DOOR_H;

  const wallX = (z, x0, x1) => {
    const span = Math.abs(x1 - x0);
    if (span < 0.05) return;
    box(span, WALL_H, WALL_T, (x0 + x1) / 2, WALL_H / 2, z, 0xf4f0e6);
  };
  const wallZ = (x, z0, z1) => {
    const span = Math.abs(z1 - z0);
    if (span < 0.05) return;
    box(WALL_T, WALL_H, span, x, WALL_H / 2, (z0 + z1) / 2, 0xf4f0e6);
  };
  const doorX = (z, x) => {
    box(DOOR_W, headerH, WALL_T, x, DOOR_H + headerH / 2, z, 0xf4f0e6);
  };
  const doorZ = (x, z) => {
    box(WALL_T, headerH, DOOR_W, x, DOOR_H + headerH / 2, z, 0xf4f0e6);
  };

  box(7.55, 0.04, 7.55, -4, 0.03, -4, 0xd2b48c);
  box(7.55, 0.04, 7.55, 4, 0.03, -4, 0x8eb6d9);
  box(7.55, 0.04, 7.55, -4, 0.03, 4, 0xf0d56a);
  box(7.55, 0.04, 7.55, 4, 0.03, 4, 0xa56b3c);

  wallZ(-HALF, -HALF, HALF);
  wallZ(HALF, -HALF, HALF);
  wallX(-HALF, -HALF, HALF);
  wallX(HALF, -HALF, -DOOR_W / 2);
  wallX(HALF, DOOR_W / 2, HALF);
  doorX(HALF, 0);

  const gap = DOOR_W / 2;
  wallZ(0, -HALF, -4 - gap);
  wallZ(0, -4 + gap, 4 - gap);
  wallZ(0, 4 + gap, HALF);
  doorZ(0, -4);
  doorZ(0, 4);
  wallX(0, -HALF, -4 - gap);
  wallX(0, -4 + gap, 4 - gap);
  wallX(0, 4 + gap, HALF);
  doorX(0, -4);
  doorX(0, 4);

  const windowMat = 0x9fd4ee;
  box(1.3, 0.9, 0.04, -4, 1.55, -HALF + 0.02, windowMat);
  box(1.3, 0.9, 0.04, 4, 1.55, -HALF + 0.02, windowMat);
  box(0.04, 0.9, 1.3, -HALF + 0.02, 1.55, -4, windowMat);
  box(0.04, 0.9, 1.3, HALF - 0.02, 1.55, -4, windowMat);
  box(0.04, 0.9, 1.3, -HALF + 0.02, 1.55, 4, windowMat);
  box(0.04, 0.9, 1.3, HALF - 0.02, 1.55, 4, windowMat);

  // Living room — red sofa, lamp, rug.
  box(2.2, 0.02, 1.6, -4, 0.06, -5.1, 0xc0392b);
  box(2.1, 0.32, 0.72, -4, 0.28, -6.85, 0xd12b2b);
  box(2.1, 0.42, 0.16, -4, 0.58, -7.18, 0xa11f1f);
  box(0.9, 0.28, 0.55, -4, 0.24, -5.15, 0x6b4226);
  cyl(0.06, 1.35, -6.7, 0.75, -6.7, 0x222222);
  cyl(0.22, 0.16, -6.7, 1.48, -6.7, 0xff4444, 16);
  cyl(0.12, 0.16, -1.7, 0.14, -6.6, 0x6b4226);
  cyl(0.22, 0.28, -1.7, 0.36, -6.6, 0x2f8f4e, 10);

  // Bedroom — blue bed and wardrobe.
  box(1.7, 0.55, 0.12, 4, 0.72, -7.2, 0x1d4e89);
  box(1.55, 0.28, 2.05, 4, 0.32, -6.05, 0x3d7edb);
  box(0.55, 0.1, 0.32, 4, 0.51, -6.85, 0xf7f7f7);
  box(0.42, 0.32, 0.38, 5.55, 0.26, -6.7, 0x2456a8);
  box(0.4, 1.7, 1.35, 6.85, 0.95, -4.2, 0x1a4f9c);

  // Kitchen — yellow counters, fridge, table.
  box(0.55, 0.9, 3.2, -7.15, 0.55, 3.4, 0xf0d030);
  box(2.4, 0.9, 0.55, -5.2, 0.55, 6.85, 0xe6c200);
  box(0.7, 1.85, 0.7, -6.15, 1.02, 6.55, 0xf4f4f4);
  box(1.15, 0.08, 0.7, -3.6, 0.78, 3.6, 0xc48a3a);
  cyl(0.04, 0.7, -4.05, 0.4, 3.35, 0x6b4226);
  cyl(0.04, 0.7, -3.15, 0.4, 3.35, 0x6b4226);
  cyl(0.04, 0.7, -4.05, 0.4, 3.85, 0x6b4226);
  cyl(0.04, 0.7, -3.15, 0.4, 3.85, 0x6b4226);
  box(0.35, 0.45, 0.35, -3.6, 0.32, 2.55, 0xd35400);

  // Study — desk, chair, bookshelf.
  box(1.4, 0.08, 0.65, 4.2, 0.78, 5.6, 0x6b4226);
  cyl(0.04, 0.7, 3.65, 0.4, 5.35, 0x5c3a1e);
  cyl(0.04, 0.7, 4.75, 0.4, 5.35, 0x5c3a1e);
  cyl(0.04, 0.7, 3.65, 0.4, 5.85, 0x5c3a1e);
  cyl(0.04, 0.7, 4.75, 0.4, 5.85, 0x5c3a1e);
  box(0.38, 0.42, 0.38, 4.2, 0.28, 4.85, 0x8d6a45);
  box(0.32, 1.9, 1.5, 6.85, 1.05, 6.15, 0x5c3a1e);
  box(0.08, 0.28, 1.2, 6.66, 1.55, 6.15, 0xc0392b);
  box(0.08, 0.22, 1.15, 6.66, 1.15, 6.15, 0x2471a3);
  box(0.08, 0.22, 1.1, 6.66, 0.75, 6.15, 0x1e8449);
  cyl(0.14, 0.18, 1.7, 0.16, 6.4, 0x6b4226);
  cyl(0.26, 0.36, 1.7, 0.42, 6.4, 0x27ae60, 10);

  beacon = new THREE.Group();
  beacon.name = "beacon";
  const disc = new THREE.Mesh(
    new THREE.CylinderGeometry(CONFIRM_RADIUS, CONFIRM_RADIUS, 0.05, 40),
    new THREE.MeshBasicMaterial({ color: 0x22c55e })
  );
  disc.position.y = 0.08;
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(CONFIRM_RADIUS * 0.78, CONFIRM_RADIUS, 40),
    new THREE.MeshBasicMaterial({ color: 0xc8ffd4, side: THREE.DoubleSide })
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.12;
  beacon.add(disc, ring);
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

  marker = buildPersonMarker();
  // Undo the miniature shrink so the figure is MARKER_HEIGHT tall in the hand.
  marker.scale.setScalar(1 / WIM_SCALE);
  mini.add(marker);
}

function buildPersonMarker() {
  const mat = new THREE.MeshBasicMaterial({ color: 0xe10600 });
  const h = MARKER_HEIGHT;
  const person = new THREE.Group();
  person.name = "userMarker";

  const leg = new THREE.BoxGeometry(h * 0.12, h * 0.32, h * 0.12);
  const leftLeg = new THREE.Mesh(leg, mat);
  leftLeg.position.set(-h * 0.09, h * 0.16, 0);
  const rightLeg = new THREE.Mesh(leg, mat);
  rightLeg.position.set(h * 0.09, h * 0.16, 0);

  const torso = new THREE.Mesh(new THREE.BoxGeometry(h * 0.34, h * 0.32, h * 0.16), mat);
  torso.position.y = h * 0.46;
  const head = new THREE.Mesh(new THREE.SphereGeometry(h * 0.13, 12, 10), mat);
  head.position.y = h * 0.74;

  const arm = new THREE.BoxGeometry(h * 0.08, h * 0.26, h * 0.08);
  const leftArm = new THREE.Mesh(arm, mat);
  leftArm.position.set(-h * 0.22, h * 0.48, 0);
  const rightArm = new THREE.Mesh(arm, mat);
  rightArm.position.set(h * 0.22, h * 0.48, 0);

  const hit = new THREE.Mesh(
    new THREE.SphereGeometry(h * 1.15, 8, 6),
    new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false })
  );
  hit.position.y = h * 0.45;
  person.add(leftLeg, rightLeg, torso, head, leftArm, rightArm, hit);
  return person;
}

function mountMiniDesktop() {
  camera.add(mini);
  // Plan view in the lower-right. In VR the grip pose replaces this.
  mini.position.set(0.58, -0.2, -0.95);
  mini.rotation.set(Math.PI / 2, 0, 0);
  mini.visible = technique === 1;
}

function attachMiniToLeftHand() {
  if (!leftGrip) return;
  leftGrip.add(mini);
  mini.position.set(0, 0.12, -0.05);
  mini.quaternion.identity();
  mini.visible = technique === 1;
}

function refreshMiniVisibility() {
  mini.visible = technique === 1;
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

const _joyRight = new THREE.Vector3();
const _joyQuat = new THREE.Quaternion();
const _joyEuler = new THREE.Euler(0, 0, 0, "YXZ");

function readLeftStick() {
  const session = renderer.xr.getSession();
  if (!session) return null;
  for (const source of session.inputSources) {
    if (source.handedness !== "left" || !source.gamepad) continue;
    const axes = source.gamepad.axes;
    if (!axes || axes.length < 2) return { x: 0, y: 0 };
    if (axes.length >= 4 && Math.hypot(axes[2], axes[3]) >= Math.hypot(axes[0], axes[1])) {
      return { x: axes[2], y: axes[3] };
    }
    return { x: axes[0], y: axes[1] };
  }
  return null;
}

function updateJoystick(dt) {
  if (technique !== 2) return;
  let x = 0;
  let y = 0;
  if (renderer.xr.isPresenting) {
    const stick = readLeftStick();
    if (!stick) return;
    x = stick.x;
    y = stick.y;
    if (Math.hypot(x, y) < JOY_DEAD) return;
  } else {
    if (keysDown.has("KeyA")) x -= 1;
    if (keysDown.has("KeyD")) x += 1;
    if (keysDown.has("KeyW")) y -= 1;
    if (keysDown.has("KeyS")) y += 1;
    if (x === 0 && y === 0) return;
    const len = Math.hypot(x, y);
    x /= len;
    y /= len;
  }

  let yawAngle = yaw;
  if (renderer.xr.isPresenting) {
    camera.getWorldQuaternion(_joyQuat);
    _joyEuler.setFromQuaternion(_joyQuat);
    yawAngle = _joyEuler.y;
  }
  _forward.set(-Math.sin(yawAngle), 0, -Math.cos(yawAngle));
  _joyRight.set(Math.cos(yawAngle), 0, -Math.sin(yawAngle));
  const step = JOY_SPEED * dt;
  xrRig.position.addScaledVector(_joyRight, x * step);
  xrRig.position.addScaledVector(_forward, -y * step);
}

function spawnBeacon(fromX, fromZ) {
  const candidates = [];
  for (const room of ROOMS) {
    for (let i = 0; i < 8; i += 1) {
      const x = room.x + THREE.MathUtils.randFloat(-1.6, 1.6);
      const z = room.z + THREE.MathUtils.randFloat(-1.6, 1.6);
      if (Math.hypot(x - fromX, z - fromZ) >= MIN_BEACON_DIST) candidates.push({ x, z });
    }
  }
  const placed = candidates.length
    ? candidates[Math.floor(Math.random() * candidates.length)]
    : { x: fromX >= 0 ? -6 : 6, z: fromZ >= 0 ? -6 : 6 };
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
  const name = technique === 1 ? "WIM" : "Joystick";
  trialCountEl.textContent = `${name} · trial ${trialCountByTechnique[technique] + 1}`;
}

function setTechnique(next) {
  technique = next === 2 ? 2 : 1;
  if (techniqueSelect) techniqueSelect.value = String(technique);
  refreshMiniVisibility();
}

function confirmTrial() {
  const now = performance.now();
  if (now - lastConfirmAt < 400) return;
  if (!withinRadius()) {
    setStatus(`Fuera del círculo (${distanceToBeacon().toFixed(2)} m).`);
    return;
  }
  lastConfirmAt = now;

  const completionTimeS = (now - trialStartTime) / 1000;
  trialCountByTechnique[technique] += 1;
  presentationOrder += 1;
  const ratio = straightLineDistance > 0 ? pathLength / straightLineDistance : 0;

  rows.push({
    participant_id: participantId(),
    technique,
    trial_number: trialCountByTechnique[technique],
    presentation_order: presentationOrder,
    completion_time_s: completionTimeS.toFixed(3),
    path_length: pathLength.toFixed(4),
    straight_line_distance: straightLineDistance.toFixed(4),
    path_ratio: ratio.toFixed(4),
  });

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
  const text = `dPos ${distance.toFixed(3)}`;

  statusEl.textContent = text;
  statusEl.classList.toggle("in-tolerance", inside);
  if (liveErrorsEl) {
    liveErrorsEl.textContent = text;
    liveErrorsEl.classList.toggle("in-tolerance", inside);
    liveErrorsEl.style.display = renderer.xr.isPresenting ? "none" : "";
  }
  updateVrHud(text, inside);
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
  mesh.visible = false;
  camera.add(mesh);
  vrHud = { ctx, texture, canvas, mesh, text: "" };
}

function updateVrHud(text, inside) {
  if (!vrHud) return;
  vrHud.mesh.visible = renderer.xr.isPresenting;
  const key = `${text}|${inside}`;
  if (vrHud.text === key) return;
  vrHud.text = key;
  const ctx = vrHud.ctx;
  ctx.clearRect(0, 0, 512, 128);
  ctx.fillStyle = inside ? "rgba(18, 64, 36, 0.88)" : "rgba(8, 10, 14, 0.78)";
  ctx.fillRect(0, 0, 512, 128);
  ctx.fillStyle = inside ? "#d8ffe4" : "#ffffff";
  ctx.font = "bold 42px sans-serif";
  ctx.fillText(text, 18, 78);
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

  if (index === 0) leftGrip = grip;
  return controller;
}

function onSelectStart(event) {
  const controller = event.target;
  if (isLeft(controller) || technique !== 1) return;
  tryGrabMarker(controller);
}

function onSelectEnd(event) {
  if (!markerDragging) return;
  if (markerDragController && event.target !== markerDragController) return;
  releaseMarker();
}

function onSqueezeStart(event) {
  if (isLeft(event.target)) return;
  confirmTrial();
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

function updateVrExit() {
  if (!renderer.xr.isPresenting) {
    xrExitWasDown = false;
    return;
  }
  const xrSession = renderer.xr.getSession();
  if (!xrSession) return;
  const pressed = isExitPressed(xrSession);
  if (pressed && !xrExitWasDown) xrSession.end();
  xrExitWasDown = pressed;
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
  }
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
  if (!renderer.xr.isPresenting) keysDown.add(event.code);
  if (renderer.xr.isPresenting) return;
  const step = 0.08;
  if (event.key === "ArrowLeft") yaw += step;
  if (event.key === "ArrowRight") yaw -= step;
  if (event.key === "ArrowUp") pitch = Math.min(1.2, pitch + step);
  if (event.key === "ArrowDown") pitch = Math.max(-1.2, pitch - step);
}

function onKeyUp(event) {
  keysDown.delete(event.code);
}

function onParticipantChange() {
  trialCountByTechnique = { 1: 0, 2: 0 };
  presentationOrder = 0;
  startTrial();
}

const clock = new THREE.Clock();

function animate() {
  const dt = Math.min(clock.getDelta(), 0.05);
  if (pendingTrialReset) {
    pendingTrialReset = false;
    startTrial();
  }
  updatePath();
  if (technique === 1) syncMarker();
  updateJoystick(dt);
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
  xrRig.position.set(-4, 0, -4);
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
  window.addEventListener("keyup", onKeyUp);

  confirmBtn.addEventListener("click", confirmTrial);
  downloadBtn.addEventListener("click", handleDownloadClick);
  techniqueSelect.addEventListener("change", () => {
    setTechnique(Number(techniqueSelect.value));
    startTrial();
  });
  participantIdInput.addEventListener("change", onParticipantChange);

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
  lastConfirmAt = -1e9;
  confirmTrial();
  check(rows.length === 0, "out-of-range confirm must not log");

  const sx = _head.x;
  const sz = _head.z;
  moveHeadTo(sx + 3, sz + 4);
  updatePath();
  check(Math.abs(pathLength - 5) < 0.05, `jump path ${pathLength.toFixed(3)}`);

  setTechnique(1);
  startTrial();
  camera.getWorldPosition(_head);
  const expectedStraight = Math.hypot(_head.x - beacon.position.x, _head.z - beacon.position.z);
  moveHeadTo(beacon.position.x, beacon.position.z);
  updatePath();
  lastConfirmAt = -1e9;
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

  lastConfirmAt = -1e9;
  confirmTrial();
  check(rows.length === 1, "second confirm outside radius must not log");

  setTechnique(2);
  startTrial();
  moveHeadTo(beacon.position.x, beacon.position.z);
  updatePath();
  lastConfirmAt = -1e9;
  confirmTrial();
  check(rows.length === 2, `expected 2 rows, got ${rows.length}`);
  check(rows[1] && rows[1].technique === 2, "second row is joystick");
  check(rows[1] && rows[1].trial_number === 1, "joystick trial_number restarts at 1");
  check(rows[1] && rows[1].presentation_order === 2, "presentation order follows confirms");
  check(buildCsv().split("\n")[0] === CSV_HEADER.join(","), "csv header");
  check(techniqueSelect.options.length === 2, "technique select has two options");
  check(techniqueSelect.options[0].value === "1" && techniqueSelect.options[1].value === "2", "WIM then Joystick");

  rows.length = 0;
  trialCountByTechnique = { 1: 0, 2: 0 };
  presentationOrder = 0;

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
