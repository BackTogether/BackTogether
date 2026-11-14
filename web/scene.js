/**
 * 3D stage for the "find my child" page: the Bobac3 robot on its pad with the Hover Camera X1
 * hovering just above its deck, and the choreography that plays when a parent sends a request
 * (spin up -> take off -> fly a search grid -> child found, the Bobac3 drives there -> drive
 * home with the drone following -> settle back over the deck).
 *
 * Everything is built from primitives, so there is no model to download: the only dependency is
 * the vendored three.js in vendor/, which keeps the demo working without wifi.
 *
 * Usage:
 *   const stage = createStage(canvas);
 *   stage.setPhase("launch");          // idle | launch | search | found | recall
 *   stage.telemetry();                 // { altitude, speed, battery, coverage, ... }
 */

import * as THREE from "./vendor/three.module.min.js";

const COLORS = {
  sky: "#070b18",
  horizon: "#1b2b52",
  ground: "#101a2e",
  grid: "#2a3f6b",
  accent: "#38e1c6",
  accentWarm: "#ffb547",
  pad: "#1a2a47",
  baseShell: "#e8edf6",
  baseDark: "#1d2535",
  droneShell: "#f2f4f8",
  droneDark: "#20242e",
  lens: "#05070d",
  screen: "#7d8490",
  tent: "#223457",
  crowd: "#33496f",
  child: "#7f9fd6",
};

/* -------------------------------------------------------------- dimensions */

/**
 * Bobac3, in metres. The body is the manual's 377 mm across; heights and sensor positions come
 * from the joints in bobac3_description/xacro (base_link sits 0.0125 above the
 * floor). The vendor meshes are not in the repo, so the shapes between those points are ours.
 * Angles are measured from the front of the robot, positive to its left, as in ROS.
 */
const BOBAC = {
  radius: 0.1885,
  skirtY: 0.035,
  lidarY: 0.2205,
  lidarOffset: 0.14,
  lidarSlot: 0.038,
  sensorY: 0.1615,
  baseCameraY: 0.047,
  coverY: 0.307,
  deckY: 0.319,
  wheelRadius: 0.0325,
  wheelOffset: 0.157,
  wheelAngles: [0, (2 * Math.PI) / 3, (-2 * Math.PI) / 3],
  smokeAngle: 0.4363,
  ultrasonicAngle: 2.7925,
  headCameraPitch: 0.785,
  // Not in the xacro: the screen is placed by eye on the tower, facing forward, high enough
  // that the head camera clipped to its top edge sits at the xacro's 0.745 m.
  towerZ: -0.115,
  screenY: 0.632,
  screenZ: -0.068,
  screenTilt: 0.35,
  screen: { width: 0.28, height: 0.175 }, // 13" at 1440 x 900
  deckSpotZ: 0.08,
};

/**
 * Hover Camera X1, in metres. No dimensions exist in the repo: these follow the published size
 * (127 x 145 x 30 mm unfolded). It is always shown unfolded.
 */
const X1 = {
  length: 0.127,
  span: 0.145,
  thickness: 0.012,
  spineWidth: 0.022,
  spineHeight: 0.018,
  frameBar: 0.004,
  rotorRadius: 0.025,
  rotorZ: 0.031,
  slats: 9,
  dockHeight: 0.075, // how far above the deck it hovers while it waits
};

// Where the Bobac3 parks, and where the child is standing in the crowd.
const HOME = { x: 0, z: 0, yaw: 2.4 };
const CHILD = { x: 1.29, z: 2.37, height: 0.7 };
const SEARCH_AREA = { width: 17, depth: 12, lanes: 5 };

/* ------------------------------------------------------- timing and easing */

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const easeInOut = (t) => (t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2);
const easeOut = (t) => 1 - Math.pow(1 - t, 3);
const smoothstep = (t) => t * t * (3 - 2 * t);

/** Frame-rate independent approach of `current` towards `target` (lambda = 1/e per second). */
const damp = (current, target, lambda, dt) => current + (target - current) * (1 - Math.exp(-lambda * dt));

/** Signed smallest rotation that takes `from` to `to`. */
const shortestAngle = (from, to) => {
  const delta = (to - from) % (2 * Math.PI);
  return ((2 * delta) % (2 * Math.PI)) - delta;
};
const dampAngle = (current, target, lambda, dt) =>
  current + shortestAngle(current, target) * (1 - Math.exp(-lambda * dt));

/** Every speed, rate and damping the choreography uses. Step durations are in the tables below. */
const MOTION = {
  reducedStep: 0.45, // longest step, in seconds, with prefers-reduced-motion
  base: {
    driveSpeed: 0.4, // m/s, the navigation cap on the real robot
    turnSpeed: 0.8, // rad/s, same
    ramp: 0.25, // share of a move spent accelerating (and again braking)
    standoff: 0.7, // m, how far from the child it stops
    accelTilt: 0.05, // rad of body pitch per m/s^2 (the wheels are on springs)
    tiltDamping: 6,
  },
  drone: {
    rotorSpeed: 46, // rad/s at full rpm
    airScale: 1.7, // drawn this much bigger in the air: at true size it is a 14 cm dot
    scaleFrom: 0.15, // m above the deck where it starts growing
    scaleTo: 1.15,
    hopHeight: 0.5, // m, the X1 pops up and settles before it climbs
    hopShare: 0.3, // share of the take-off step spent on the hop
    takeoffTurn: Math.PI * 0.75,
    cruiseAltitude: 4.2,
    cruiseSpeed: 3.4, // m/s
    reducedCruiseSpeed: 6.5,
    hoverAltitude: 2.4, // m, over the child
    bobAmplitude: 0.06, // m
    bobRate: 1.7, // rad/s
    swayAngle: 0.03, // rad of roll drift while hovering
    swayRate: 1.1,
    tiltPerSpeed: 0.05, // rad of lean per m/s
    maxTilt: 0.22,
    tiltDamping: 4,
    yawDamping: 3,
    velocityDamping: 8,
    followBack: 1.0, // m behind the Bobac3
    followHeight: 1.5, // m above the floor
    followDamping: 1.8,
    gimbalDown: 0.9, // rad
    gimbalDamping: 3,
    landHover: 0.3, // m above its waiting spot where it pauses before settling
    landHoverShare: 0.7, // share of the landing step spent getting there
    idleRpm: 0.6, // rotor speed while it waits over the deck
    idleBob: 0.2, // share of the hover bob it keeps there
    batteryDrain: 0.09, // % per second: moves visibly in a demo without emptying
  },
  camera: { damping: 1.6, targetDamping: 2.4, manualHold: 6000, autoSpin: 0.045 },
  world: { pulseRate: 2.2, scanFade: 3, spotFade: 4, trailEvery: 0.07 },
};

/** Position profile of a move that accelerates, cruises and brakes, for t in 0..1. */
const trapezoid = (t, ramp = MOTION.base.ramp) => {
  const peak = 1 / (1 - ramp);
  if (t < ramp) return (peak * t * t) / (2 * ramp);
  if (t > 1 - ramp) return 1 - (peak * (1 - t) * (1 - t)) / (2 * ramp);
  return peak * (t - ramp / 2);
};

// Durations in seconds. A phase runs through its steps one after the other.
const LAUNCH_STEPS = [
  { id: "ack", label: "Request received", duration: 1.4 },
  { id: "spinup", label: "Rotors spinning up", duration: 1.3 },
  { id: "takeoff", label: "Lift-off from the Bobac3", duration: 3.6 },
];
const SPOT_STEP = { id: "spot", label: "Child spotted from the air", duration: 2.6 };
const ARRIVE_STEP = { id: "arrive", label: "Bobac3 is with the child", duration: 3 };
const RETURN_STEP = { id: "return", label: "Returning to the Bobac3", duration: 3 };
const LAND_STEP = { id: "land", label: "Settling over the Bobac3 deck", duration: 2.8 };

// What follows a phase once its last step is done. Not listed = hold the last step.
const NEXT_PHASE = { launch: "search", recall: "idle" };

/** A step that turns the Bobac3 on the spot, timed so it peaks at its real turning speed. */
function turnStep(id, label, from, yaw) {
  const delta = shortestAngle(from.yaw, yaw);
  const duration = Math.abs(delta) / (MOTION.base.turnSpeed * (1 - MOTION.base.ramp));
  return { id, label, duration, base: { from: { ...from }, to: { ...from, yaw: from.yaw + delta } } };
}

/** A step that drives the Bobac3 straight ahead to (x, z), timed for its real driving speed. */
function driveStep(id, label, from, x, z) {
  const distance = Math.hypot(x - from.x, z - from.z);
  const duration = distance / (MOTION.base.driveSpeed * (1 - MOTION.base.ramp));
  return { id, label, duration, base: { from: { ...from }, to: { x, z, yaw: from.yaw } } };
}

function foundSteps(pose) {
  const bearing = Math.atan2(CHILD.x - pose.x, CHILD.z - pose.z);
  const distance = Math.hypot(CHILD.x - pose.x, CHILD.z - pose.z) - MOTION.base.standoff;
  const turn = turnStep("turn", "Bobac3 turning towards the child", pose, bearing);
  const drive = driveStep(
    "drive",
    "Bobac3 driving to the child",
    turn.base.to,
    pose.x + Math.sin(bearing) * distance,
    pose.z + Math.cos(bearing) * distance
  );
  return [SPOT_STEP, turn, drive, ARRIVE_STEP];
}

function recallSteps(pose) {
  const away = Math.hypot(pose.x - HOME.x, pose.z - HOME.z) > 0.05;
  if (!away) return [RETURN_STEP, LAND_STEP];

  const bearing = Math.atan2(HOME.x - pose.x, HOME.z - pose.z);
  const turn = turnStep("turnHome", "Bobac3 turning back to the pad", pose, bearing);
  const drive = driveStep("driveHome", "Bobac3 driving home, drone following", turn.base.to, HOME.x, HOME.z);
  const face = turnStep("faceHome", "Bobac3 lining up on the pad", drive.base.to, HOME.yaw);
  return [turn, drive, face, LAND_STEP];
}

/* ---------------------------------------------------------------- textures */

function radialTexture(inner, outer, stops) {
  const size = 256;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext("2d");
  const gradient = ctx.createRadialGradient(size / 2, size / 2, inner * size, size / 2, size / 2, outer * size);
  for (const [offset, color] of stops) gradient.addColorStop(offset, color);
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function skyTexture() {
  const canvas = document.createElement("canvas");
  canvas.width = 16;
  canvas.height = 256;
  const ctx = canvas.getContext("2d");
  const gradient = ctx.createLinearGradient(0, 0, 0, 256);
  gradient.addColorStop(0, "#04060f");
  gradient.addColorStop(0.55, COLORS.sky);
  gradient.addColorStop(0.82, COLORS.horizon);
  gradient.addColorStop(1, "#243a68");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, 16, 256);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

function glowSprite(color, size) {
  const map = radialTexture(0, 0.5, [
    [0, "rgba(255,255,255,1)"],
    [0.25, "rgba(255,255,255,0.55)"],
    [1, "rgba(255,255,255,0)"],
  ]);
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map,
      color: new THREE.Color(color),
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      transparent: true,
    })
  );
  sprite.scale.setScalar(size);
  return sprite;
}

/** Flat ring lying on the ground or a deck, drawn as light rather than as a surface. */
function glowRing(inner, outer, color, opacity) {
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(inner, outer, 56),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  ring.rotation.x = -Math.PI / 2;
  return ring;
}

/* ------------------------------------------------------------------ world */

function buildGround(scene) {
  const ground = new THREE.Mesh(
    new THREE.CircleGeometry(60, 72),
    new THREE.MeshStandardMaterial({
      color: COLORS.ground,
      roughness: 0.95,
      metalness: 0.05,
    })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  const grid = new THREE.GridHelper(80, 80, new THREE.Color(COLORS.grid), new THREE.Color(COLORS.grid));
  grid.material.transparent = true;
  grid.material.opacity = 0.18;
  grid.position.y = 0.002;
  scene.add(grid);

  // Soft pool of light under the pad, so the robot does not stand on a flat colour.
  const pool = new THREE.Mesh(
    new THREE.CircleGeometry(4.5, 48),
    new THREE.MeshBasicMaterial({
      map: radialTexture(0, 0.5, [
        [0, "rgba(56,225,198,0.13)"],
        [0.45, "rgba(56,225,198,0.04)"],
        [1, "rgba(56,225,198,0)"],
      ]),
      transparent: true,
      depthWrite: false,
    })
  );
  pool.rotation.x = -Math.PI / 2;
  pool.position.y = 0.004;
  scene.add(pool);
}

/** Tents, a stage and stylised people: enough to read as "an event", cheap to draw. */
function buildEvent(scene) {
  const tentGeometry = new THREE.ConeGeometry(2.4, 2.4, 4);
  const tentMaterial = new THREE.MeshStandardMaterial({ color: COLORS.tent, roughness: 0.8 });
  const crowdGeometry = new THREE.CapsuleGeometry(0.17, 1.25, 4, 8);
  const crowdMaterial = new THREE.MeshStandardMaterial({ color: COLORS.crowd, roughness: 0.9 });

  const props = new THREE.Group();
  for (let i = 0; i < 9; i += 1) {
    const angle = (i / 9) * Math.PI * 2 + 0.4;
    const radius = 15 + ((i * 7) % 6);
    const tent = new THREE.Mesh(tentGeometry, tentMaterial);
    tent.position.set(Math.cos(angle) * radius, 1.2, Math.sin(angle) * radius);
    tent.rotation.y = angle;
    tent.castShadow = true;
    props.add(tent);
  }

  const stage = new THREE.Mesh(
    new THREE.BoxGeometry(9, 1.1, 4),
    new THREE.MeshStandardMaterial({ color: "#19243c", roughness: 0.7 })
  );
  stage.position.set(-2, 0.55, -19);
  stage.castShadow = true;
  props.add(stage);

  // Deterministic scatter (no Math.random) so every reload looks the same.
  for (let i = 0; i < 85; i += 1) {
    const angle = i * 2.399963;
    const radius = 3.4 + (i % 13) * 1.35;
    const person = new THREE.Mesh(crowdGeometry, crowdMaterial);
    person.position.set(Math.cos(angle) * radius, 0.82, Math.sin(angle) * radius);
    person.rotation.y = angle;
    props.add(person);
  }

  scene.add(props);
  return props;
}

/** The child the search is for: a small figure, with a ring that lights up once found. */
function buildChild(scene) {
  const radius = 0.1;
  const figure = new THREE.Mesh(
    new THREE.CapsuleGeometry(radius, CHILD.height - 2 * radius, 4, 8),
    new THREE.MeshStandardMaterial({ color: COLORS.child, roughness: 0.8 })
  );
  figure.position.set(CHILD.x, CHILD.height / 2, CHILD.z);
  figure.castShadow = true;
  scene.add(figure);

  const ring = glowRing(0.3, 0.38, COLORS.accentWarm, 0);
  ring.position.set(CHILD.x, 0.02, CHILD.z);
  scene.add(ring);
  return { ring };
}

/** Parking mark on the floor. It is flat, so the Bobac3 can drive off it. */
function buildPad(scene) {
  const pad = new THREE.Group();

  const slab = new THREE.Mesh(
    new THREE.CylinderGeometry(0.95, 1.0, 0.012, 48),
    new THREE.MeshStandardMaterial({ color: COLORS.pad, roughness: 0.6, metalness: 0.2 })
  );
  slab.position.y = 0.006;
  slab.receiveShadow = true;
  pad.add(slab);

  const ring = glowRing(0.72, 0.86, COLORS.accent, 0.55);
  ring.position.y = 0.015;
  pad.add(ring);

  for (let i = 0; i < 4; i += 1) {
    const mark = new THREE.Mesh(
      new THREE.PlaneGeometry(0.3, 0.05),
      new THREE.MeshBasicMaterial({ color: COLORS.accent, transparent: true, opacity: 0.35 })
    );
    mark.rotation.x = -Math.PI / 2;
    const angle = (i / 4) * Math.PI * 2;
    mark.rotation.z = -angle;
    mark.position.set(Math.cos(angle) * 0.64, 0.014, Math.sin(angle) * 0.64);
    pad.add(mark);
  }

  scene.add(pad);
  return { group: pad, ring };
}

/* ------------------------------------------------------------------ bobac3 */

/**
 * Empty group on the robot's vertical axis at `angle` from the front, `radius` out and `y` up,
 * with its +z pointing outwards. Children placed in it sit on the body facing away from it.
 */
function radialMount(angle, radius, y) {
  const mount = new THREE.Group();
  mount.position.set(Math.sin(angle) * radius, y, Math.cos(angle) * radius);
  mount.rotation.y = angle;
  return mount;
}

/** Round sensor window lying flat on the body. */
function sensorDisc(angle, y, radius, material) {
  const mount = radialMount(angle, BOBAC.radius, y);
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 0.008, 18), material);
  disc.rotation.x = Math.PI / 2;
  mount.add(disc);
  return mount;
}

/** Omni wheel on a radial axle: a disc with rollers around the rim, so its spin is visible. */
function buildWheel(angle, material) {
  const mount = radialMount(angle, BOBAC.wheelOffset, BOBAC.wheelRadius);
  const spinner = new THREE.Group();
  spinner.rotation.x = Math.PI / 2;
  mount.add(spinner);

  const radius = BOBAC.wheelRadius;
  const tyre = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.86, radius * 0.86, 0.026, 20), material);
  tyre.castShadow = true;
  spinner.add(tyre);

  const rollerGeometry = new THREE.SphereGeometry(radius * 0.24, 8, 6);
  const rollers = 8;
  for (let i = 0; i < rollers; i += 1) {
    const around = (i / rollers) * Math.PI * 2;
    const roller = new THREE.Mesh(rollerGeometry, material);
    roller.position.set(Math.cos(around) * radius * 0.8, 0, Math.sin(around) * radius * 0.8);
    spinner.add(roller);
  }
  return { mount, spinner, angle };
}

/**
 * Bobac3 as we have it: the round base with its sensors and the rear tower carrying the head
 * camera and the touchscreen. There is no arm on our robot, so none is modelled.
 */
function buildBase() {
  const base = new THREE.Group();
  base.rotation.order = "YXZ"; // heading first, then the pitch of the body on its springs

  const shell = new THREE.MeshStandardMaterial({ color: COLORS.baseShell, roughness: 0.35, metalness: 0.15 });
  const dark = new THREE.MeshStandardMaterial({ color: COLORS.baseDark, roughness: 0.5, metalness: 0.4 });
  const lens = new THREE.MeshStandardMaterial({ color: COLORS.lens, roughness: 0.15, metalness: 0.9 });

  const { radius } = BOBAC;
  const slotBottom = BOBAC.lidarY - BOBAC.lidarSlot / 2;
  const slotTop = BOBAC.lidarY + BOBAC.lidarSlot / 2;

  /** Vertical cylinder between two heights. */
  const drum = (bottom, top, drumRadius, material) => {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(drumRadius, drumRadius, top - bottom, 48), material);
    mesh.position.y = (bottom + top) / 2;
    mesh.castShadow = true;
    return mesh;
  };

  // The body is split by the slot the two lidars look out of.
  base.add(drum(BOBAC.skirtY, slotBottom, radius, shell));
  base.add(drum(slotBottom, slotTop, radius * 0.66, dark));
  base.add(drum(slotBottom, slotBottom + 0.002, radius * 0.99, dark)); // dark floor of the slot
  base.add(drum(slotTop, BOBAC.coverY, radius, shell));
  base.add(drum(BOBAC.skirtY, BOBAC.skirtY + 0.028, radius * 1.012, dark)); // bumper strip
  for (const z of [BOBAC.lidarOffset, -BOBAC.lidarOffset]) {
    const lidar = drum(slotBottom + 0.004, slotTop - 0.004, 0.034, dark);
    lidar.position.z = z;
    base.add(lidar);
  }

  const wheels = BOBAC.wheelAngles.map((angle) => buildWheel(angle, dark));
  for (const wheel of wheels) base.add(wheel.mount);

  // Sensor band: depth camera at the front, smoke and humidity either side of it, the two
  // ultrasonic sensors at the back, and the docking camera low at the back.
  const depthCamera = radialMount(0, radius - 0.004, BOBAC.sensorY);
  depthCamera.add(new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.026, 0.016), dark));
  for (const x of [-0.026, 0.026]) {
    const eye = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.004, 14), lens);
    eye.rotation.x = Math.PI / 2;
    eye.position.set(x, 0, 0.009);
    depthCamera.add(eye);
  }
  base.add(depthCamera);
  for (const side of [-1, 1]) {
    base.add(sensorDisc(side * BOBAC.smokeAngle, BOBAC.sensorY, 0.014, dark));
    base.add(sensorDisc(side * BOBAC.ultrasonicAngle, BOBAC.sensorY, 0.012, dark));
  }
  base.add(sensorDisc(Math.PI, BOBAC.baseCameraY, 0.011, lens));

  // Top cover: the deck the drone sits on.
  const cover = drum(BOBAC.coverY, BOBAC.deckY, radius * 0.955, dark);
  cover.receiveShadow = true;
  base.add(cover);
  const deckRing = glowRing(0.07, 0.082, COLORS.accent, 0.5);
  deckRing.position.set(0, BOBAC.deckY + 0.002, BOBAC.deckSpotZ);
  base.add(deckRing);

  // Rear tower: a column carrying the screen, with the head camera clipped to the screen's top
  // edge and looking up at faces.
  const tower = new THREE.Group();
  const columnTop = BOBAC.screenY + 0.03;
  const column = new THREE.Mesh(new THREE.BoxGeometry(0.06, columnTop - BOBAC.deckY, 0.03), shell);
  column.position.set(0, (columnTop + BOBAC.deckY) / 2, BOBAC.towerZ);
  column.castShadow = true;
  tower.add(column);

  const bracket = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.04, 0.05), dark);
  bracket.position.set(0, BOBAC.screenY, (BOBAC.towerZ + BOBAC.screenZ) / 2);
  tower.add(bracket);

  // The 13" touchscreen, switched off: a plain grey panel.
  const screen = new THREE.Mesh(
    new THREE.PlaneGeometry(BOBAC.screen.width, BOBAC.screen.height),
    new THREE.MeshBasicMaterial({ color: COLORS.screen }) // unlit, so it stays grey at night
  );
  const bezelDepth = 0.016;
  const bezelHeight = BOBAC.screen.height + 0.03;
  const display = new THREE.Group();
  display.position.set(0, BOBAC.screenY, BOBAC.screenZ);
  display.rotation.x = -BOBAC.screenTilt;
  const bezel = new THREE.Mesh(new THREE.BoxGeometry(BOBAC.screen.width + 0.03, bezelHeight, bezelDepth), dark);
  bezel.castShadow = true;
  display.add(bezel);
  screen.position.z = bezelDepth / 2 + 0.0006;
  display.add(screen);

  const cameraHeight = 0.03;
  const headCamera = new THREE.Group();
  headCamera.position.y = bezelHeight / 2 + cameraHeight / 2 - 0.004;
  headCamera.rotation.x = BOBAC.screenTilt - BOBAC.headCameraPitch; // the display is already tilted
  headCamera.add(new THREE.Mesh(new THREE.BoxGeometry(0.07, cameraHeight, 0.026), dark));
  const headLens = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.006, 16), lens);
  headLens.rotation.x = Math.PI / 2;
  headLens.position.z = 0.015;
  headCamera.add(headLens);
  display.add(headCamera);

  tower.add(display);
  base.add(tower);

  const statusMaterial = new THREE.MeshBasicMaterial({ color: COLORS.accent });
  const status = new THREE.Mesh(new THREE.TorusGeometry(radius * 1.005, 0.006, 8, 48), statusMaterial);
  status.rotation.x = Math.PI / 2;
  status.position.y = slotBottom - 0.014;
  base.add(status);

  const statusGlow = glowSprite(COLORS.accent, 0.55);
  statusGlow.position.y = slotBottom - 0.014;
  base.add(statusGlow);

  return { group: base, wheels, statusMaterial, statusGlow, deckRing };
}

/* --------------------------------------------------------- hover camera x1 */

/** One cage wing, centred on its own origin: frame, protective slats and two rotors. */
function buildWing(materials, rotors) {
  const wing = new THREE.Group();
  const width = (X1.span - X1.spineWidth) / 2;
  const { length, thickness, frameBar } = X1;

  const sideBar = new THREE.BoxGeometry(frameBar, thickness, length);
  const endBar = new THREE.BoxGeometry(width, thickness, frameBar);
  for (const sign of [-1, 1]) {
    const side = new THREE.Mesh(sideBar, materials.shell);
    side.position.x = (sign * (width - frameBar)) / 2;
    side.castShadow = true;
    wing.add(side);
    const end = new THREE.Mesh(endBar, materials.shell);
    end.position.z = (sign * (length - frameBar)) / 2;
    wing.add(end);
  }

  // The X1's propellers are fully enclosed: thin slats above and below them.
  const slatSize = 0.0012;
  const crossSlat = new THREE.BoxGeometry(width - 2 * frameBar, slatSize, slatSize);
  const longSlat = new THREE.BoxGeometry(slatSize, slatSize, length - 2 * frameBar);
  for (const sign of [-1, 1]) {
    const y = sign * (thickness / 2 - slatSize / 2);
    for (let i = 1; i <= X1.slats; i += 1) {
      const slat = new THREE.Mesh(crossSlat, materials.shell);
      slat.position.set(0, y, -length / 2 + (length / (X1.slats + 1)) * i);
      wing.add(slat);
    }
    const spine = new THREE.Mesh(longSlat, materials.shell);
    spine.position.y = y;
    wing.add(spine);
  }

  for (const z of [-X1.rotorZ, X1.rotorZ]) {
    const strut = new THREE.Mesh(new THREE.BoxGeometry(width - frameBar, 0.003, 0.005), materials.dark);
    strut.position.set(0, -0.0035, z);
    wing.add(strut);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.006, 0.006, 12), materials.dark);
    hub.position.set(0, -0.001, z);
    wing.add(hub);

    const rotor = new THREE.Group();
    rotor.position.set(0, 0.0025, z);
    rotor.add(new THREE.Mesh(new THREE.BoxGeometry(X1.rotorRadius * 2, 0.001, 0.007), materials.blade));
    // Translucent disc that fades in with rpm, to read as motion blur.
    const disc = new THREE.Mesh(
      new THREE.CircleGeometry(X1.rotorRadius, 24),
      new THREE.MeshBasicMaterial({
        color: "#cfd8ea",
        transparent: true,
        opacity: 0,
        depthWrite: false,
        side: THREE.DoubleSide,
      })
    );
    disc.rotation.x = -Math.PI / 2;
    rotor.add(disc);
    wing.add(rotor);
    rotors.push({ group: rotor, disc, direction: rotors.length % 2 === 0 ? 1 : -1 });
  }
  return wing;
}

/**
 * Hover Camera X1: a slim spine with the camera at the front and a cage wing hinged on each
 * side, lying flat.
 */
function buildDrone() {
  const drone = new THREE.Group();
  drone.rotation.order = "YXZ"; // heading first, then lean in the drone's own frame

  const materials = {
    shell: new THREE.MeshStandardMaterial({ color: COLORS.droneShell, roughness: 0.4, metalness: 0.1 }),
    dark: new THREE.MeshStandardMaterial({ color: COLORS.droneDark, roughness: 0.45, metalness: 0.5 }),
    blade: new THREE.MeshStandardMaterial({ color: "#8d97ad", roughness: 0.6 }),
    lens: new THREE.MeshStandardMaterial({ color: COLORS.lens, roughness: 0.15, metalness: 0.9 }),
  };

  // The spine's top is level with the top of the wings.
  const spineY = (X1.thickness - X1.spineHeight) / 2;
  const spine = new THREE.Mesh(new THREE.BoxGeometry(X1.spineWidth, X1.spineHeight, X1.length), materials.dark);
  spine.position.y = spineY;
  spine.castShadow = true;
  drone.add(spine);

  const sensorWindow = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.002, 0.03), materials.lens);
  sensorWindow.position.y = spineY - X1.spineHeight / 2;
  drone.add(sensorWindow);

  // Camera on its tilt gimbal at the nose.
  const gimbal = new THREE.Group();
  gimbal.position.set(0, spineY, X1.length / 2 + 0.005);
  gimbal.add(new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.016, 0.012), materials.dark));
  const gimbalLens = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.004, 14), materials.lens);
  gimbalLens.rotation.x = Math.PI / 2;
  gimbalLens.position.z = 0.007;
  gimbal.add(gimbalLens);
  drone.add(gimbal);

  const rotors = [];
  const wingWidth = (X1.span - X1.spineWidth) / 2;
  for (const side of [-1, 1]) {
    const wing = buildWing(materials, rotors);
    wing.position.x = (side * (X1.spineWidth + wingWidth)) / 2;
    drone.add(wing);
  }

  const beaconMaterial = new THREE.MeshBasicMaterial({ color: COLORS.accentWarm });
  const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.004, 10, 8), beaconMaterial);
  beacon.position.set(0, spineY, -X1.length / 2 - 0.002);
  drone.add(beacon);
  const beaconGlow = glowSprite(COLORS.accentWarm, 0.08);
  beaconGlow.position.copy(beacon.position);
  drone.add(beaconGlow);

  return { group: drone, rotors, gimbal, beaconMaterial, beaconGlow };
}

/**
 * The camera's field of view, drawn from the drone straight down to the ground. It lives in world
 * space (not under the drone) so it stays vertical while the drone banks, like a stabilised gimbal.
 */
function buildScanCone(scene) {
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(1, 1, 36, 1, true),
    new THREE.MeshBasicMaterial({
      color: COLORS.accent,
      transparent: true,
      opacity: 0,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    })
  );
  scene.add(cone);
  return cone;
}

/** Bright ring on the ground under the drone: where the camera is looking right now. */
function buildScanSpot(scene) {
  const group = new THREE.Group();

  const ring = glowRing(0.9, 1, COLORS.accent, 0);
  group.add(ring);

  const fill = new THREE.Mesh(
    new THREE.CircleGeometry(1, 48),
    new THREE.MeshBasicMaterial({
      map: radialTexture(0, 0.5, [
        [0, "rgba(56,225,198,0.5)"],
        [0.6, "rgba(56,225,198,0.12)"],
        [1, "rgba(56,225,198,0)"],
      ]),
      transparent: true,
      opacity: 0,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  fill.rotation.x = -Math.PI / 2;
  group.add(fill);

  group.position.y = 0.015;
  scene.add(group);
  return { group, ring, fill };
}

/** Fading trail of the drone's path, drawn as a line plus glowing points. */
function buildTrail(scene, maxPoints = 420) {
  const positions = new Float32Array(maxPoints * 3);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setDrawRange(0, 0);

  const line = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({
      color: COLORS.accent,
      transparent: true,
      opacity: 0.45,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  line.frustumCulled = false;
  scene.add(line);

  const points = new THREE.Points(
    geometry,
    new THREE.PointsMaterial({
      color: COLORS.accent,
      size: 0.07,
      transparent: true,
      opacity: 0.5,
      sizeAttenuation: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      map: radialTexture(0, 0.5, [
        [0, "rgba(255,255,255,1)"],
        [1, "rgba(255,255,255,0)"],
      ]),
    })
  );
  points.frustumCulled = false;
  scene.add(points);

  let count = 0;
  return {
    push(position) {
      if (count === maxPoints) {
        positions.copyWithin(0, 3);
        count -= 1;
      }
      positions.set([position.x, position.y, position.z], count * 3);
      count += 1;
      geometry.setDrawRange(0, count);
      geometry.attributes.position.needsUpdate = true;
      geometry.computeBoundingSphere();
    },
    clear() {
      count = 0;
      geometry.setDrawRange(0, 0);
    },
    setVisible(visible) {
      line.visible = visible;
      points.visible = visible;
    },
  };
}

/* ------------------------------------------------------------ search flight */

/** Lawnmower path over the search area, as a list of waypoints at cruise altitude. */
function searchWaypoints() {
  const { width, depth, lanes } = SEARCH_AREA;
  const waypoints = [];
  for (let lane = 0; lane < lanes; lane += 1) {
    const z = -depth / 2 + (depth / (lanes - 1)) * lane;
    const xs = lane % 2 === 0 ? [-width / 2, width / 2] : [width / 2, -width / 2];
    for (const x of xs) waypoints.push(new THREE.Vector3(x, MOTION.drone.cruiseAltitude, z));
  }
  return waypoints;
}

/* -------------------------------------------------------------- camera rig */

const IDLE_FRAME = { theta: -0.85, phi: 1.12, radius: 2.1, target: new THREE.Vector3(0, 0.42, 0) };

/**
 * Orbit camera with scripted framing per phase. Dragging takes over for a few seconds, then the
 * scripted framing eases back in, so the stage keeps presenting itself during a demo.
 */
class CameraRig {
  constructor(camera, canvas, reducedMotion) {
    this.camera = camera;
    this.reducedMotion = reducedMotion;
    this.theta = IDLE_FRAME.theta;
    this.phi = IDLE_FRAME.phi;
    this.radius = IDLE_FRAME.radius;
    this.target = IDLE_FRAME.target.clone();
    this.desired = { theta: this.theta, phi: this.phi, radius: this.radius, target: this.target.clone() };
    this.autoSpin = MOTION.camera.autoSpin;
    this.manualUntil = 0;
    this.#bind(canvas);
  }

  #bind(canvas) {
    let pointerId = null;
    let last = { x: 0, y: 0 };

    const onDown = (event) => {
      pointerId = event.pointerId;
      last = { x: event.clientX, y: event.clientY };
      canvas.setPointerCapture(pointerId);
      canvas.style.cursor = "grabbing";
    };
    const onMove = (event) => {
      if (event.pointerId !== pointerId) return;
      const dx = event.clientX - last.x;
      const dy = event.clientY - last.y;
      last = { x: event.clientX, y: event.clientY };
      this.theta -= dx * 0.005;
      this.phi = clamp(this.phi - dy * 0.005, 0.25, 1.45);
      this.desired.theta = this.theta;
      this.desired.phi = this.phi;
      this.manualUntil = performance.now() + MOTION.camera.manualHold;
    };
    const onUp = () => {
      if (pointerId === null) return;
      if (canvas.hasPointerCapture(pointerId)) canvas.releasePointerCapture(pointerId);
      pointerId = null;
      canvas.style.cursor = "grab";
    };

    canvas.addEventListener("pointerdown", onDown);
    canvas.addEventListener("pointermove", onMove);
    canvas.addEventListener("pointerup", onUp);
    canvas.addEventListener("pointercancel", onUp);
    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        this.radius = clamp(this.radius + event.deltaY * 0.008, 0.6, 26);
        this.desired.radius = this.radius;
        this.manualUntil = performance.now() + MOTION.camera.manualHold;
      },
      { passive: false }
    );
    canvas.style.cursor = "grab";
  }

  frame({ theta, phi, radius, target, autoSpin = MOTION.camera.autoSpin }) {
    if (theta !== undefined) this.desired.theta = theta;
    if (phi !== undefined) this.desired.phi = phi;
    if (radius !== undefined) this.desired.radius = radius;
    if (target) this.desired.target.copy(target);
    this.autoSpin = autoSpin;
  }

  update(dt) {
    const manual = performance.now() < this.manualUntil;
    if (!manual) {
      if (!this.reducedMotion) this.desired.theta += this.autoSpin * dt;
      const lambda = MOTION.camera.damping;
      this.theta = damp(this.theta, this.desired.theta, lambda, dt);
      this.phi = damp(this.phi, this.desired.phi, lambda, dt);
      this.radius = damp(this.radius, this.desired.radius, lambda, dt);
    }
    this.target.lerp(this.desired.target, 1 - Math.exp(-MOTION.camera.targetDamping * dt));

    const sinPhi = Math.sin(this.phi);
    this.camera.position.set(
      this.target.x + this.radius * sinPhi * Math.cos(this.theta),
      this.target.y + this.radius * Math.cos(this.phi),
      this.target.z + this.radius * sinPhi * Math.sin(this.theta)
    );
    this.camera.lookAt(this.target);
  }
}

/* ------------------------------------------------------------------- stage */

class Stage {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.reducedMotion = Boolean(options.reducedMotion);
    this.onStep = options.onStep || (() => {});
    this.onPhase = options.onPhase || (() => {});

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: "high-performance",
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.scene.background = skyTexture();
    this.scene.fog = new THREE.Fog(new THREE.Color(COLORS.sky), 18, 62);

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.05, 200);
    this.rig = new CameraRig(this.camera, canvas, this.reducedMotion);

    this.#buildLights();
    buildGround(this.scene);
    buildEvent(this.scene);
    this.child = buildChild(this.scene);
    this.pad = buildPad(this.scene);
    this.base = buildBase();
    this.scene.add(this.base.group);
    this.drone = buildDrone();
    this.scene.add(this.drone.group);
    this.trail = buildTrail(this.scene);
    this.trail.setVisible(false);
    this.scanSpot = buildScanSpot(this.scene);
    this.scanCone = buildScanCone(this.scene);

    this.waypoints = searchWaypoints();
    this.state = {
      phase: "idle",
      steps: [],
      stepIndex: 0,
      stepElapsed: 0,
      stepStart: new THREE.Vector3(), // where the drone was when the current step began
      rpm: MOTION.drone.idleRpm,
      altitude: 0,
      battery: 100,
      coverage: 0,
      speed: 0,
      legIndex: -1, // -1 = flying from the take-off point to the first waypoint
      legProgress: 0,
      searchEntry: new THREE.Vector3(),
      elapsed: 0,
    };

    this.basePose = { ...HOME };
    this.baseMotion = { ...HOME, speed: 0, tilt: 0 }; // last frame's pose, to measure velocity

    this.dronePosition = this.#deckPosition(new THREE.Vector3());
    this.droneYaw = HOME.yaw;
    this.dronePrevious = this.dronePosition.clone();
    this.droneVelocity = new THREE.Vector3();
    this.lastTrailAt = 0;
    this.scratch = new THREE.Vector3();

    this.#place();

    this.clock = new THREE.Clock();
    this.running = false;
    this.#observeSize();
  }

  #buildLights() {
    this.scene.add(new THREE.HemisphereLight(new THREE.Color("#8fb6ff"), new THREE.Color("#0b1120"), 0.65));

    const key = new THREE.DirectionalLight(new THREE.Color("#dbe7ff"), 1.5);
    key.position.set(6, 11, 5);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 1;
    key.shadow.camera.far = 40;
    key.shadow.camera.left = -14;
    key.shadow.camera.right = 14;
    key.shadow.camera.top = 14;
    key.shadow.camera.bottom = -14;
    key.shadow.bias = -0.0008;
    this.scene.add(key);

    const rim = new THREE.DirectionalLight(new THREE.Color(COLORS.accent), 0.5);
    rim.position.set(-7, 4, -6);
    this.scene.add(rim);

    this.padLight = new THREE.PointLight(new THREE.Color(COLORS.accent), 2.2, 6, 2);
    this.padLight.position.set(0, 0.12, 0); // low, so it lights the floor and not the deck
    this.scene.add(this.padLight);
  }

  #observeSize() {
    const resize = () => {
      const width = this.canvas.clientWidth || 1;
      const height = this.canvas.clientHeight || 1;
      this.renderer.setSize(width, height, false);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
    };
    resize();
    if ("ResizeObserver" in window) {
      this.resizeObserver = new ResizeObserver(resize);
      this.resizeObserver.observe(this.canvas);
    } else {
      window.addEventListener("resize", resize);
    }
  }

  /* --------------------------------------------------------------- phases */

  setPhase(phase) {
    if (phase === this.state.phase) return;
    if (phase === "found" && this.state.phase !== "search") return;
    const previous = this.state.phase;
    this.state.phase = phase;
    this.state.stepIndex = 0;
    this.state.stepElapsed = 0;
    this.state.steps = [];

    const statusColor = phase === "idle" || phase === "search" ? COLORS.accent : COLORS.accentWarm;
    this.base.statusMaterial.color.set(statusColor);
    this.base.statusGlow.material.color.set(statusColor);

    if (phase === "idle") {
      this.state.rpm = MOTION.drone.idleRpm;
      this.state.battery = 100;
      this.state.coverage = 0;
      Object.assign(this.basePose, HOME);
      this.#deckPosition(this.dronePosition);
      this.droneYaw = HOME.yaw;
      this.trail.clear();
      this.trail.setVisible(false);
      this.rig.frame(IDLE_FRAME);
    } else if (phase === "launch") {
      this.state.steps = LAUNCH_STEPS;
      this.trail.setVisible(true);
    } else if (phase === "search") {
      this.state.searchEntry.copy(this.dronePosition);
      this.state.legIndex = -1;
      this.state.legProgress = 0;
      this.onStep({ id: "search", label: "Scanning the event zone", index: LAUNCH_STEPS.length, phase });
    } else if (phase === "found") {
      this.state.steps = foundSteps(this.basePose);
    } else if (phase === "recall") {
      this.state.steps = recallSteps(this.basePose);
    }

    if (this.state.steps.length) this.#beginStep();
    this.onPhase(phase, previous);
  }

  #beginStep() {
    const step = this.state.steps[this.state.stepIndex];
    this.state.stepStart.copy(this.dronePosition);
    this.onStep({ id: step.id, label: step.label, index: this.state.stepIndex, phase: this.state.phase });
  }

  /** Advances the current phase's steps and applies the one that is playing. */
  #runSteps(dt) {
    const { steps } = this.state;
    const step = steps[this.state.stepIndex];
    const duration = this.reducedMotion ? Math.min(step.duration, MOTION.reducedStep) : step.duration;
    this.state.stepElapsed += dt;
    if (this.state.stepElapsed < duration) {
      this.#applyStep(step, this.state.stepElapsed / duration, dt);
      return;
    }

    // Finish the step exactly on its end pose before moving on.
    this.#applyStep(step, 1, dt);
    if (this.state.stepIndex < steps.length - 1) {
      this.state.stepIndex += 1;
      this.state.stepElapsed = 0;
      this.#beginStep();
      return;
    }
    const next = NEXT_PHASE[this.state.phase];
    if (next) this.setPhase(next);
    else this.state.stepElapsed = duration;
  }

  #applyStep(step, progress, dt) {
    if (step.base) {
      const { from, to } = step.base;
      const eased = trapezoid(progress);
      this.basePose.x = from.x + (to.x - from.x) * eased;
      this.basePose.z = from.z + (to.z - from.z) * eased;
      this.basePose.yaw = from.yaw + (to.yaw - from.yaw) * eased;
    }
    if (this.state.phase === "launch") this.#updateLaunch(step, progress);
    else if (this.state.phase === "found") this.#updateFound(step, progress, dt);
    else if (this.state.phase === "recall") this.#updateRecall(step, progress, dt);
  }

  /* ------------------------------------------------------------ animation */

  /** World position of the drone's waiting spot over the deck, written into `out`. */
  #deckPosition(out) {
    const { x, z, yaw } = this.basePose;
    return out.set(
      x + Math.sin(yaw) * BOBAC.deckSpotZ,
      BOBAC.deckY + X1.dockHeight,
      z + Math.cos(yaw) * BOBAC.deckSpotZ
    );
  }

  #baseTarget(height) {
    return new THREE.Vector3(this.basePose.x, height, this.basePose.z);
  }

  #place() {
    const { drone: motion } = MOTION;
    const deckY = BOBAC.deckY + X1.dockHeight;
    const airborne = smoothstep(
      clamp((this.dronePosition.y - deckY - motion.scaleFrom) / (motion.scaleTo - motion.scaleFrom), 0, 1)
    );
    const bob = this.reducedMotion ? 0 : Math.sin(this.state.elapsed * motion.bobRate) * motion.bobAmplitude;
    this.drone.group.position.copy(this.dronePosition);
    this.drone.group.position.y += bob * Math.max(airborne, motion.idleBob);
    this.drone.group.rotation.y = this.droneYaw;
    this.drone.group.scale.setScalar(1 + (motion.airScale - 1) * airborne);
    this.airborne = airborne;

    this.base.group.position.set(this.basePose.x, 0, this.basePose.z);
    this.base.group.rotation.y = this.basePose.yaw;
  }

  #updateLaunch(step, progress) {
    const deck = this.#deckPosition(this.scratch);
    const { drone: motion } = MOTION;
    this.dronePosition.copy(deck);
    this.droneYaw = this.basePose.yaw;

    if (step.id === "ack") {
      this.rig.frame({ radius: 1.7, phi: 1.2, target: this.#baseTarget(0.5), autoSpin: 0.12 });
    } else if (step.id === "spinup") {
      this.state.rpm = motion.idleRpm + (1 - motion.idleRpm) * easeOut(progress);
      this.rig.frame({ radius: 0.9, phi: 1.25, target: deck, autoSpin: 0.1 });
    } else if (step.id === "takeoff") {
      this.state.rpm = 1;
      const climbHeight = motion.cruiseAltitude - deck.y;
      const height =
        progress < motion.hopShare
          ? motion.hopHeight * easeOut(progress / motion.hopShare)
          : motion.hopHeight +
            (climbHeight - motion.hopHeight) * easeInOut((progress - motion.hopShare) / (1 - motion.hopShare));
      const climb = height / climbHeight;
      this.dronePosition.y = deck.y + height;
      this.droneYaw = this.basePose.yaw + climb * motion.takeoffTurn;
      this.rig.frame({
        radius: 1.7 + climb * 8,
        phi: 1.18 - climb * 0.35,
        target: this.#baseTarget(0.5 + climb * (motion.cruiseAltitude - 1.4)),
        autoSpin: 0.06,
      });
    }
  }

  #updateSearch(dt) {
    const { drone: motion } = MOTION;
    const { waypoints, state } = this;
    state.rpm = 1;

    const transit = state.legIndex < 0;
    const from = transit ? state.searchEntry : waypoints[state.legIndex];
    const to = transit ? waypoints[0] : waypoints[(state.legIndex + 1) % waypoints.length];
    const cruise = this.reducedMotion ? motion.reducedCruiseSpeed : motion.cruiseSpeed;
    state.legProgress += (cruise * dt) / Math.max(from.distanceTo(to), 0.001);

    if (state.legProgress >= 1) {
      state.legProgress = 0;
      state.legIndex = (state.legIndex + 1) % waypoints.length;
      if (state.legIndex === 0) this.trail.clear();
    }

    this.dronePosition.lerpVectors(from, to, state.legProgress);
    this.droneYaw = dampAngle(this.droneYaw, Math.atan2(to.x - from.x, to.z - from.z), motion.yawDamping, dt);
    const flown = Math.max(0, state.legIndex) + (transit ? 0 : state.legProgress);
    state.coverage = clamp((flown / (waypoints.length - 1)) * 100, 0, 100);

    // Chase the drone: the rig damps towards the target, which gives a trailing camera.
    this.rig.frame({
      radius: 6.8,
      phi: 0.82,
      target: new THREE.Vector3(this.dronePosition.x, this.dronePosition.y - 2.4, this.dronePosition.z),
      autoSpin: 0.02,
    });
  }

  /** The drone holds over the child as a marker while the Bobac3 drives there. */
  #updateFound(step, progress, dt) {
    this.state.rpm = 1;
    const hover = this.scratch.set(CHILD.x, MOTION.drone.hoverAltitude, CHILD.z);
    if (step.id === "spot") this.dronePosition.lerpVectors(this.state.stepStart, hover, easeInOut(progress));
    else this.dronePosition.copy(hover);

    // Keep the drone's camera side towards the Bobac3.
    const toBase = Math.atan2(this.basePose.x - this.dronePosition.x, this.basePose.z - this.dronePosition.z);
    this.droneYaw = dampAngle(this.droneYaw, toBase, MOTION.drone.yawDamping, dt);

    if (step.id === "spot") {
      this.rig.frame({
        radius: 6.5,
        phi: 0.95,
        target: new THREE.Vector3(CHILD.x / 2, 1.2, CHILD.z / 2),
        autoSpin: 0.03,
      });
    } else if (step.id === "turn") {
      this.rig.frame({ radius: 2.4, phi: 1.15, target: this.#baseTarget(0.45), autoSpin: 0.05 });
    } else if (step.id === "drive") {
      this.rig.frame({ radius: 3.4, phi: 1.1, target: this.#baseTarget(0.5), autoSpin: 0.05 });
    } else {
      this.rig.frame({
        radius: 2.6,
        phi: 1.2,
        target: new THREE.Vector3((this.basePose.x + CHILD.x) / 2, 0.5, (this.basePose.z + CHILD.z) / 2),
        autoSpin: 0.08,
      });
    }
  }

  /** Drone trails the Bobac3 from behind and above, like the X1's follow mode. */
  #followBase(dt) {
    const { drone: motion } = MOTION;
    const { x, z, yaw } = this.basePose;
    const lambda = motion.followDamping;
    this.dronePosition.x = damp(this.dronePosition.x, x - Math.sin(yaw) * motion.followBack, lambda, dt);
    this.dronePosition.y = damp(this.dronePosition.y, motion.followHeight, lambda, dt);
    this.dronePosition.z = damp(this.dronePosition.z, z - Math.cos(yaw) * motion.followBack, lambda, dt);
    this.droneYaw = dampAngle(this.droneYaw, yaw, motion.yawDamping, dt);
  }

  #updateRecall(step, progress, dt) {
    const { drone: motion } = MOTION;
    const deck = this.#deckPosition(this.scratch);

    if (step.base) {
      this.state.rpm = 1;
      this.#followBase(dt);
      this.rig.frame({ radius: 4.2, phi: 1.0, target: this.#baseTarget(0.8), autoSpin: 0.05 });
    } else if (step.id === "return") {
      this.state.rpm = Math.max(this.state.rpm, easeOut(progress));
      const above = new THREE.Vector3(deck.x, motion.cruiseAltitude, deck.z);
      this.dronePosition.lerpVectors(this.state.stepStart, above, easeInOut(progress));
      this.rig.frame({ radius: 12, phi: 0.95, target: this.#baseTarget(3), autoSpin: 0.05 });
    } else if (step.id === "land") {
      const start = this.state.stepStart;
      const share = motion.landHoverShare;
      const over = easeInOut(clamp(progress / share, 0, 1));
      const down = easeInOut(clamp((progress - share) / (1 - share), 0, 1));
      const pauseY = deck.y + motion.landHover;
      this.dronePosition.set(
        start.x + (deck.x - start.x) * over,
        start.y + (pauseY - start.y) * over + (deck.y - pauseY) * down,
        start.z + (deck.z - start.z) * over
      );
      this.droneYaw = dampAngle(this.droneYaw, this.basePose.yaw, motion.yawDamping, dt);
      this.state.rpm = 1 - (1 - motion.idleRpm) * down;
      this.rig.frame({ radius: 2.6, phi: 1.2, target: this.#baseTarget(BOBAC.deckY + 0.3), autoSpin: 0.08 });
    }
  }

  #updateDroneVisuals(dt) {
    const { drone: motion, world } = MOTION;
    const { rpm, phase } = this.state;

    for (const rotor of this.drone.rotors) {
      rotor.group.rotation.y += rotor.direction * rpm * motion.rotorSpeed * dt;
      rotor.disc.material.opacity = clamp((rpm - 0.25) * 0.5, 0, 0.34);
    }

    // Lean into the direction of travel, in the drone's own frame, plus a slow hover sway.
    this.scratch.copy(this.dronePosition).sub(this.dronePrevious).divideScalar(Math.max(dt, 1e-3));
    this.droneVelocity.lerp(this.scratch, 1 - Math.exp(-motion.velocityDamping * dt));
    this.dronePrevious.copy(this.dronePosition);
    const sin = Math.sin(this.droneYaw);
    const cos = Math.cos(this.droneYaw);
    const forward = this.droneVelocity.x * sin + this.droneVelocity.z * cos;
    const sideways = this.droneVelocity.x * cos - this.droneVelocity.z * sin;
    const lean = (speed) => clamp(speed * motion.tiltPerSpeed, -motion.maxTilt, motion.maxTilt) * this.airborne;
    const sway = this.reducedMotion ? 0 : Math.sin(this.state.elapsed * motion.swayRate) * motion.swayAngle;
    const rotation = this.drone.group.rotation;
    rotation.x = damp(rotation.x, lean(forward), motion.tiltDamping, dt);
    rotation.z = damp(rotation.z, -lean(sideways) + sway * this.airborne, motion.tiltDamping, dt);

    const looking = this.airborne > 0.5 && phase !== "launch";
    this.drone.gimbal.rotation.x = damp(
      this.drone.gimbal.rotation.x,
      looking ? motion.gimbalDown : 0,
      motion.gimbalDamping,
      dt
    );

    // Beacon: slow pulse while it waits, fast blink on a mission.
    const blink = phase === "idle" ? 0.9 : 2.6;
    const pulse = 0.35 + 0.65 * Math.abs(Math.sin(this.state.elapsed * blink * Math.PI));
    this.drone.beaconGlow.material.opacity = pulse;
    this.drone.beaconMaterial.color
      .set(phase === "search" ? COLORS.accent : COLORS.accentWarm)
      .multiplyScalar(0.6 + pulse * 0.4);

    // Scan cone from the drone down to the ground, only while searching.
    const scanning = phase === "search";
    const height = Math.max(this.dronePosition.y - 0.05, 0.2);
    const spotRadius = Math.max(0.35, height * 0.18);

    // Apex at the drone, base exactly on the ring below it.
    this.scanCone.scale.set(spotRadius, height, spotRadius);
    this.scanCone.position.set(this.dronePosition.x, height / 2, this.dronePosition.z);
    this.scanCone.rotation.y += dt * 0.8;
    this.scanCone.material.opacity = damp(this.scanCone.material.opacity, scanning ? 0.07 : 0, world.scanFade, dt);
    this.scanSpot.group.position.set(this.dronePosition.x, 0.015, this.dronePosition.z);
    this.scanSpot.group.scale.setScalar(spotRadius);
    const spotOpacity = scanning ? 0.3 + 0.12 * Math.sin(this.state.elapsed * 3) : 0;
    this.scanSpot.ring.material.opacity = damp(this.scanSpot.ring.material.opacity, spotOpacity, world.spotFade, dt);
    this.scanSpot.fill.material.opacity = damp(
      this.scanSpot.fill.material.opacity,
      scanning ? 0.3 : 0,
      world.spotFade,
      dt
    );

    if (scanning && this.state.elapsed - this.lastTrailAt > world.trailEvery) {
      this.lastTrailAt = this.state.elapsed;
      this.trail.push(this.dronePosition);
    }

    this.state.speed = this.droneVelocity.length();
    this.state.altitude = Math.max(0, this.dronePosition.y - BOBAC.deckY - X1.dockHeight);
    if (phase !== "idle") {
      this.state.battery = clamp(this.state.battery - dt * motion.batteryDrain, 0, 100);
    }
  }

  #updateBaseVisuals(dt) {
    const { base: motion } = MOTION;
    const pose = this.basePose;
    const last = this.baseMotion;
    const safeDt = Math.max(dt, 1e-3);

    // Speed along the heading and turn rate, measured from the pose change this frame.
    const speed = ((pose.x - last.x) * Math.sin(pose.yaw) + (pose.z - last.z) * Math.cos(pose.yaw)) / safeDt;
    const turnRate = shortestAngle(last.yaw, pose.yaw) / safeDt;
    const acceleration = (speed - last.speed) / safeDt;

    // Three omni wheels: each rolls along its tangent, so driving straight leaves the front
    // wheel still (its rollers slide) while turning spins all three the same way.
    for (const wheel of this.base.wheels) {
      const rim = -speed * Math.sin(wheel.angle) + turnRate * BOBAC.wheelOffset;
      wheel.spinner.rotation.y += (rim / BOBAC.wheelRadius) * dt;
    }

    // The body leans back when it pulls away and forward when it brakes.
    const tilt = clamp(-acceleration * motion.accelTilt, -0.04, 0.04);
    last.tilt = damp(last.tilt, tilt, motion.tiltDamping, dt);
    this.base.group.rotation.x = last.tilt;
    Object.assign(last, { x: pose.x, z: pose.z, yaw: pose.yaw, speed });
  }

  #updateWorld(dt) {
    const { phase, elapsed } = this.state;
    const pulse = 0.4 + 0.3 * Math.sin(elapsed * MOTION.world.pulseRate);
    this.pad.ring.material.opacity = 0.3 + pulse * 0.4;
    this.padLight.intensity = 1.4 + pulse * 1.8;
    this.base.deckRing.material.opacity = phase === "idle" ? 0.3 + pulse * 0.3 : 0.6;
    this.child.ring.material.opacity = damp(
      this.child.ring.material.opacity,
      phase === "found" ? 0.45 + pulse * 0.5 : 0,
      MOTION.world.spotFade,
      dt
    );
  }

  #frame = () => {
    if (!this.running) return;
    const dt = Math.min(this.clock.getDelta(), 1 / 20);
    this.state.elapsed += dt;

    if (this.state.phase === "search") this.#updateSearch(dt);
    else if (this.state.steps.length) this.#runSteps(dt);

    this.#place();
    this.#updateDroneVisuals(dt);
    this.#updateBaseVisuals(dt);
    this.#updateWorld(dt);
    this.rig.update(dt);
    this.renderer.render(this.scene, this.camera);
    this.raf = requestAnimationFrame(this.#frame);
  };

  start() {
    if (this.running) return;
    this.running = true;
    this.clock.getDelta();
    this.raf = requestAnimationFrame(this.#frame);
  }

  stop() {
    this.running = false;
    if (this.raf) cancelAnimationFrame(this.raf);
  }

  telemetry() {
    return {
      phase: this.state.phase,
      altitude: this.state.altitude,
      speed: this.state.speed,
      battery: this.state.battery,
      coverage: this.state.coverage,
      step: this.state.steps[this.state.stepIndex]?.label || null,
    };
  }
}

/** Creates the stage, or returns null when WebGL is unavailable (the page then stays usable). */
export function createStage(canvas, options = {}) {
  try {
    const stage = new Stage(canvas, options);
    stage.start();
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) stage.stop();
      else stage.start();
    });
    return stage;
  } catch (error) {
    console.warn("3D stage unavailable:", error);
    return null;
  }
}

export { LAUNCH_STEPS };
