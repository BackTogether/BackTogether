/**
 * 3D stage for the "find my child" page: the Bobac3 mobile base on its landing pad with the
 * folded Hover Air X1 on the deck, and the launch choreography that plays when a parent sends
 * a request (unfold -> spin up -> take off -> fly a search grid -> come back on cancel).
 *
 * Everything is built from primitives, so there is no model to download: the only dependency is
 * the vendored three.js in vendor/, which keeps the demo working without wifi.
 *
 * Usage:
 *   const stage = await createStage(canvas);
 *   stage.setPhase("launch");          // idle | launch | search | recall
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
  tent: "#223457",
  crowd: "#33496f",
};

// Metres. The base is the real Bobac3 footprint (robot_radius 0.25 m in the nav stack).
const BASE_RADIUS = 0.26;
const BASE_HEIGHT = 0.32;
const DECK_Y = BASE_HEIGHT + 0.09;
const CRUISE_ALTITUDE = 4.2;
const SEARCH_AREA = { width: 17, depth: 12, lanes: 5 };

// Phase durations in seconds. Order matters: launch runs through them one after the other.
const LAUNCH_STEPS = [
  { id: "ack", label: "Request received", duration: 1.4 },
  { id: "unfold", label: "Hover Air X1 unfolding", duration: 1.8 },
  { id: "spinup", label: "Rotors spinning up", duration: 1.3 },
  { id: "takeoff", label: "Lift-off from the Bobac3 base", duration: 3.2 },
];
const RECALL_STEPS = [
  { id: "return", label: "Returning to the base", duration: 3.0 },
  { id: "land", label: "Landing on the deck", duration: 2.2 },
  { id: "fold", label: "Folding the arms", duration: 1.4 },
];

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const easeInOut = (t) => (t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2);
const easeOut = (t) => 1 - Math.pow(1 - t, 3);

/** Frame-rate independent approach of `current` towards `target` (lambda = 1/e per second). */
const damp = (current, target, lambda, dt) => current + (target - current) * (1 - Math.exp(-lambda * dt));

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

  // Soft pool of light under the pad, so the base does not float on a flat colour.
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

function buildPad(scene) {
  const pad = new THREE.Group();

  const slab = new THREE.Mesh(
    new THREE.CylinderGeometry(0.95, 1.0, 0.07, 48),
    new THREE.MeshStandardMaterial({ color: COLORS.pad, roughness: 0.6, metalness: 0.2 })
  );
  slab.position.y = 0.04;
  slab.receiveShadow = true;
  pad.add(slab);

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.72, 0.86, 64),
    new THREE.MeshBasicMaterial({
      color: COLORS.accent,
      transparent: true,
      opacity: 0.55,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.092;
  pad.add(ring);

  const marks = new THREE.Group();
  for (let i = 0; i < 4; i += 1) {
    const mark = new THREE.Mesh(
      new THREE.PlaneGeometry(0.3, 0.05),
      new THREE.MeshBasicMaterial({ color: COLORS.accent, transparent: true, opacity: 0.35 })
    );
    mark.rotation.x = -Math.PI / 2;
    const angle = (i / 4) * Math.PI * 2;
    mark.rotation.z = -angle;
    mark.position.set(Math.cos(angle) * 0.64, 0.09, Math.sin(angle) * 0.64);
    marks.add(mark);
  }
  pad.add(marks);

  scene.add(pad);
  return { group: pad, ring };
}

/* ------------------------------------------------------------- bobac3 base */

function buildBase() {
  const base = new THREE.Group();

  const shell = new THREE.MeshStandardMaterial({ color: COLORS.baseShell, roughness: 0.35, metalness: 0.15 });
  const dark = new THREE.MeshStandardMaterial({ color: COLORS.baseDark, roughness: 0.5, metalness: 0.4 });

  const chassis = new THREE.Mesh(new THREE.CylinderGeometry(BASE_RADIUS, BASE_RADIUS * 0.94, BASE_HEIGHT, 40), shell);
  chassis.position.y = BASE_HEIGHT / 2 + 0.06;
  chassis.castShadow = true;
  base.add(chassis);

  const skirt = new THREE.Mesh(new THREE.CylinderGeometry(BASE_RADIUS * 1.02, BASE_RADIUS * 1.02, 0.05, 40), dark);
  skirt.position.y = 0.1;
  base.add(skirt);

  // Omni wheels: four, 45 degrees apart from the forward axis.
  const wheelGeometry = new THREE.CylinderGeometry(0.062, 0.062, 0.045, 18);
  for (let i = 0; i < 4; i += 1) {
    const angle = Math.PI / 4 + (i / 4) * Math.PI * 2;
    const wheel = new THREE.Mesh(wheelGeometry, dark);
    wheel.position.set(Math.cos(angle) * (BASE_RADIUS - 0.03), 0.062, Math.sin(angle) * (BASE_RADIUS - 0.03));
    wheel.rotation.z = Math.PI / 2;
    wheel.rotation.y = angle;
    wheel.castShadow = true;
    base.add(wheel);
  }

  // Front and rear lidar pucks.
  for (const z of [BASE_RADIUS - 0.02, -(BASE_RADIUS - 0.02)]) {
    const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.05, 20), dark);
    puck.position.set(0, 0.2, z);
    base.add(puck);
  }

  // Landing deck the drone sits on.
  const deck = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.24, 0.025, 36), dark);
  deck.position.y = DECK_Y - 0.012;
  deck.receiveShadow = true;
  base.add(deck);

  const deckRing = new THREE.Mesh(
    new THREE.RingGeometry(0.16, 0.21, 40),
    new THREE.MeshBasicMaterial({
      color: COLORS.accent,
      transparent: true,
      opacity: 0.5,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  deckRing.rotation.x = -Math.PI / 2;
  deckRing.position.y = DECK_Y + 0.008;
  base.add(deckRing);

  // Mast with the head camera (the real base carries its camera on a short mast).
  const mast = new THREE.Group();
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.026, 0.42, 16), dark);
  pole.position.set(0, DECK_Y + 0.21, -0.17);
  mast.add(pole);

  const head = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.09, 0.07), shell);
  head.position.set(0, DECK_Y + 0.43, -0.17);
  head.castShadow = true;
  mast.add(head);

  const lens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.022, 0.022, 0.02, 16),
    new THREE.MeshStandardMaterial({ color: "#0b1020", roughness: 0.2, metalness: 0.8 })
  );
  lens.rotation.x = Math.PI / 2;
  lens.position.set(0, DECK_Y + 0.43, -0.135);
  mast.add(lens);
  base.add(mast);

  const statusMaterial = new THREE.MeshBasicMaterial({ color: COLORS.accent });
  const status = new THREE.Mesh(new THREE.TorusGeometry(BASE_RADIUS * 0.98, 0.012, 8, 44), statusMaterial);
  status.rotation.x = Math.PI / 2;
  status.position.y = BASE_HEIGHT - 0.02;
  base.add(status);

  const statusGlow = glowSprite(COLORS.accent, 0.7);
  statusGlow.position.y = BASE_HEIGHT - 0.02;
  base.add(statusGlow);

  return { group: base, mast, head, statusMaterial, statusGlow, deckRing };
}

/* ------------------------------------------------------- hover air x1 drone */

/**
 * Folding quadcopter: a flat body with two arm groups that each carry a propeller guard and two
 * rotors. Folded, the arms are rotated back over the body (the X1 folds into a flat brick);
 * unfolded they lie flat and the rotors can spin.
 */
function buildDrone() {
  const drone = new THREE.Group();

  const shell = new THREE.MeshStandardMaterial({ color: COLORS.droneShell, roughness: 0.4, metalness: 0.1 });
  const dark = new THREE.MeshStandardMaterial({ color: COLORS.droneDark, roughness: 0.45, metalness: 0.5 });
  const bladeMaterial = new THREE.MeshStandardMaterial({ color: "#8d97ad", roughness: 0.6 });

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.034, 0.17), shell);
  body.castShadow = true;
  drone.add(body);

  const spine = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.012, 0.145), dark);
  spine.position.y = 0.021;
  drone.add(spine);

  // Gimbal camera at the front, tilted down while searching.
  const gimbal = new THREE.Group();
  const gimbalBall = new THREE.Mesh(new THREE.SphereGeometry(0.023, 18, 14), dark);
  gimbal.add(gimbalBall);
  const gimbalLens = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.012, 0.012, 16),
    new THREE.MeshStandardMaterial({ color: "#05070d", roughness: 0.15, metalness: 0.9 })
  );
  gimbalLens.rotation.x = Math.PI / 2;
  gimbalLens.position.z = 0.018;
  gimbal.add(gimbalLens);
  gimbal.position.set(0, -0.005, 0.088);
  drone.add(gimbal);

  const rotors = [];
  const arms = [];

  for (const side of [-1, 1]) {
    const arm = new THREE.Group();
    arm.position.set(side * 0.045, 0.012, 0);
    drone.add(arm);
    arms.push({ group: arm, side });

    const spar = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.01, 0.028), dark);
    spar.position.x = side * 0.055;
    arm.add(spar);

    // Guard frame around each rotor pair (the X1's propellers are fully enclosed).
    for (const z of [-0.062, 0.062]) {
      const guard = new THREE.Mesh(new THREE.TorusGeometry(0.058, 0.006, 8, 34), shell);
      guard.rotation.x = Math.PI / 2;
      guard.position.set(side * 0.1, 0.004, z);
      guard.castShadow = true;
      arm.add(guard);

      const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.014, 0.016, 14), dark);
      hub.position.set(side * 0.1, 0.012, z);
      arm.add(hub);

      const rotor = new THREE.Group();
      rotor.position.set(side * 0.1, 0.02, z);
      for (let b = 0; b < 2; b += 1) {
        const blade = new THREE.Mesh(new THREE.BoxGeometry(0.094, 0.0025, 0.013), bladeMaterial);
        blade.rotation.y = b * Math.PI;
        blade.rotation.z = 0.12;
        rotor.add(blade);
      }
      // Translucent disc that fades in with rpm, to read as motion blur.
      const disc = new THREE.Mesh(
        new THREE.CircleGeometry(0.05, 28),
        new THREE.MeshBasicMaterial({
          color: "#cfd8ea",
          transparent: true,
          opacity: 0,
          depthWrite: false,
          side: THREE.DoubleSide,
        })
      );
      disc.rotation.x = -Math.PI / 2;
      disc.position.y = 0.004;
      rotor.add(disc);

      arm.add(rotor);
      rotors.push({ group: rotor, disc, direction: z > 0 ? 1 : -1 });
    }
  }

  const beaconMaterial = new THREE.MeshBasicMaterial({ color: COLORS.accentWarm });
  const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 10), beaconMaterial);
  beacon.position.set(0, 0.026, -0.07);
  drone.add(beacon);
  const beaconGlow = glowSprite(COLORS.accentWarm, 0.16);
  beaconGlow.position.copy(beacon.position);
  drone.add(beaconGlow);

  return { group: drone, arms, rotors, gimbal, beaconMaterial, beaconGlow };
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

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.9, 1, 56),
    new THREE.MeshBasicMaterial({
      color: COLORS.accent,
      transparent: true,
      opacity: 0,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    })
  );
  ring.rotation.x = -Math.PI / 2;
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
    for (const x of xs) waypoints.push(new THREE.Vector3(x, CRUISE_ALTITUDE, z));
  }
  return waypoints;
}

/* -------------------------------------------------------------- camera rig */

/**
 * Orbit camera with scripted framing per phase. Dragging takes over for a few seconds, then the
 * scripted framing eases back in, so the stage keeps presenting itself during a demo.
 */
class CameraRig {
  constructor(camera, canvas, reducedMotion) {
    this.camera = camera;
    this.reducedMotion = reducedMotion;
    this.theta = -0.85;
    this.phi = 1.1;
    this.radius = 2.9;
    this.target = new THREE.Vector3(0, 0.42, 0);
    this.desired = { theta: this.theta, phi: this.phi, radius: this.radius, target: this.target.clone() };
    this.autoSpin = 0.045;
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
      this.manualUntil = performance.now() + 6000;
    };
    const onUp = (event) => {
      if (pointerId === null) return;
      if (canvas.hasPointerCapture(pointerId)) canvas.releasePointerCapture(pointerId);
      pointerId = null;
      canvas.style.cursor = "grab";
      void event;
    };

    canvas.addEventListener("pointerdown", onDown);
    canvas.addEventListener("pointermove", onMove);
    canvas.addEventListener("pointerup", onUp);
    canvas.addEventListener("pointercancel", onUp);
    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        this.radius = clamp(this.radius + event.deltaY * 0.008, 1.4, 26);
        this.desired.radius = this.radius;
        this.manualUntil = performance.now() + 6000;
      },
      { passive: false }
    );
    canvas.style.cursor = "grab";
  }

  frame({ theta, phi, radius, target, autoSpin = 0.045 }) {
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
      const lambda = 1.6;
      this.theta = damp(this.theta, this.desired.theta, lambda, dt);
      this.phi = damp(this.phi, this.desired.phi, lambda, dt);
      this.radius = damp(this.radius, this.desired.radius, lambda, dt);
    }
    this.target.lerp(this.desired.target, 1 - Math.exp(-2.4 * dt));

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

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.1, 200);
    this.rig = new CameraRig(this.camera, canvas, this.reducedMotion);

    this.#buildLights();
    buildGround(this.scene);
    buildEvent(this.scene);
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
      fold: 1, // 1 = folded, 0 = unfolded
      rpm: 0,
      altitude: 0,
      battery: 100,
      coverage: 0,
      speed: 0,
      legIndex: 0,
      legProgress: 0,
      recallFrom: new THREE.Vector3(),
      elapsed: 0,
    };
    this.dronePosition = new THREE.Vector3(0, DECK_Y + 0.03, 0);
    this.lastDt = 1 / 60;
    this.droneYaw = 0;
    this.lastTrailAt = 0;

    this.#applyFold(1);
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
    key.shadow.mapSize.set(1024, 1024);
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
    this.padLight.position.set(0, 0.45, 0);
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
    const previous = this.state.phase;
    this.state.phase = phase;
    this.state.stepIndex = 0;
    this.state.stepElapsed = 0;

    if (phase === "idle") {
      this.state.steps = [];
      this.state.fold = 1;
      this.state.rpm = 0;
      this.state.battery = 100;
      this.state.coverage = 0;
      this.state.legIndex = 0;
      this.state.legProgress = 0;
      this.dronePosition.set(0, DECK_Y + 0.03, 0);
      this.droneYaw = 0;
      this.trail.clear();
      this.trail.setVisible(false);
      this.base.statusMaterial.color.set(COLORS.accent);
      this.base.statusGlow.material.color.set(COLORS.accent);
      this.rig.frame({ theta: -0.85, phi: 1.1, radius: 2.9, target: new THREE.Vector3(0, 0.42, 0) });
    } else if (phase === "launch") {
      this.state.steps = LAUNCH_STEPS;
      this.trail.setVisible(true);
      this.base.statusMaterial.color.set(COLORS.accentWarm);
      this.base.statusGlow.material.color.set(COLORS.accentWarm);
      this.#announceStep();
    } else if (phase === "search") {
      this.state.steps = [];
      this.base.statusMaterial.color.set(COLORS.accent);
      this.base.statusGlow.material.color.set(COLORS.accent);
      this.onStep({ id: "search", label: "Scanning the event zone", index: LAUNCH_STEPS.length });
    } else if (phase === "recall") {
      this.state.steps = RECALL_STEPS;
      this.state.recallFrom.copy(this.dronePosition);
      this.base.statusMaterial.color.set(COLORS.accentWarm);
      this.base.statusGlow.material.color.set(COLORS.accentWarm);
      this.#announceStep();
    }

    this.onPhase(phase, previous);
  }

  #announceStep() {
    const step = this.state.steps[this.state.stepIndex];
    if (step) this.onStep({ ...step, index: this.state.stepIndex });
  }

  #advanceSteps(dt) {
    const steps = this.state.steps;
    if (!steps.length) return null;
    const step = steps[this.state.stepIndex];
    const duration = this.reducedMotion ? Math.min(step.duration, 0.45) : step.duration;
    this.state.stepElapsed += dt;
    if (this.state.stepElapsed >= duration) {
      this.state.stepElapsed = 0;
      if (this.state.stepIndex < steps.length - 1) {
        this.state.stepIndex += 1;
        this.#announceStep();
      } else {
        this.setPhase(this.state.phase === "launch" ? "search" : "idle");
        return null;
      }
    }
    return { step, progress: clamp(this.state.stepElapsed / duration, 0, 1) };
  }

  /* ------------------------------------------------------------ animation */

  #applyFold(fold) {
    // Folded: arms swung back over the body. Unfolded: flat, rotors clear of the shell.
    for (const arm of this.drone.arms) {
      arm.group.rotation.z = arm.side * fold * 2.5;
      arm.group.rotation.y = arm.side * fold * 0.25;
      arm.group.position.y = 0.012 + fold * 0.016;
    }
  }

  #place() {
    this.drone.group.position.copy(this.dronePosition);
    this.drone.group.rotation.y = this.droneYaw;
  }

  #updateLaunch(active) {
    if (!active) return;
    const { step, progress } = active;
    if (step.id === "ack") {
      this.base.mast.rotation.y = Math.sin(progress * Math.PI) * 0.35;
      this.rig.frame({ radius: 1.9, phi: 1.2, target: new THREE.Vector3(0, DECK_Y, 0), autoSpin: 0.12 });
    } else if (step.id === "unfold") {
      this.state.fold = 1 - easeInOut(progress);
      this.rig.frame({ radius: 1.15, phi: 1.1, target: new THREE.Vector3(0, DECK_Y + 0.04, 0), autoSpin: 0.18 });
    } else if (step.id === "spinup") {
      this.state.fold = 0;
      this.state.rpm = easeOut(progress);
      this.dronePosition.y = DECK_Y + 0.03 + Math.sin(progress * Math.PI * 6) * 0.004 * progress;
      this.rig.frame({ radius: 1.5, phi: 1.28, target: new THREE.Vector3(0, DECK_Y + 0.06, 0), autoSpin: 0.1 });
    } else if (step.id === "takeoff") {
      this.state.rpm = 1;
      const climb = easeInOut(progress);
      const previousY = this.dronePosition.y;
      this.dronePosition.y = DECK_Y + 0.03 + climb * (CRUISE_ALTITUDE - DECK_Y - 0.03);
      this.dronePosition.x = Math.sin(progress * Math.PI) * 0.25;
      this.state.speed = Math.max(0, (this.dronePosition.y - previousY) / Math.max(this.lastDt, 1e-3));
      this.droneYaw = climb * Math.PI * 0.75;
      this.rig.frame({
        radius: 2.2 + climb * 7.5,
        phi: 1.18 - climb * 0.35,
        target: new THREE.Vector3(0, 0.6 + climb * (CRUISE_ALTITUDE - 1.4), 0),
        autoSpin: 0.06,
      });
    }
  }

  #updateSearch(dt) {
    this.state.rpm = 1;
    const legs = this.waypoints.length - 1;
    const from = this.waypoints[this.state.legIndex];
    const to = this.waypoints[(this.state.legIndex + 1) % this.waypoints.length];
    const legLength = from.distanceTo(to);
    const cruise = this.reducedMotion ? 6.5 : 3.4; // m/s
    this.state.legProgress += (cruise * dt) / Math.max(legLength, 0.001);

    if (this.state.legProgress >= 1) {
      this.state.legProgress = 0;
      this.state.legIndex = (this.state.legIndex + 1) % legs;
      if (this.state.legIndex === 0) this.trail.clear();
    }

    const eased = this.state.legProgress;
    this.dronePosition.lerpVectors(from, to, eased);
    this.dronePosition.y = CRUISE_ALTITUDE + Math.sin(this.state.elapsed * 1.1) * 0.12;
    this.droneYaw = Math.atan2(to.x - from.x, to.z - from.z);
    this.state.speed = cruise;
    this.state.coverage = clamp(
      ((this.state.legIndex + this.state.legProgress) / legs) * 100,
      0,
      100
    );

    // Chase the drone: the rig damps towards the target, which gives a trailing camera.
    this.rig.frame({
      radius: 6.8,
      phi: 0.82,
      target: new THREE.Vector3(this.dronePosition.x, this.dronePosition.y - 2.4, this.dronePosition.z),
      autoSpin: 0.02,
    });
  }

  #updateRecall(active) {
    if (!active) return;
    const { step, progress } = active;
    const deck = new THREE.Vector3(0, DECK_Y + 0.03, 0);
    if (step.id === "return") {
      const eased = easeInOut(progress);
      this.dronePosition.lerpVectors(this.state.recallFrom, new THREE.Vector3(0, CRUISE_ALTITUDE, 0), eased);
      this.state.speed = 4;
      this.rig.frame({ radius: 12, phi: 0.95, target: new THREE.Vector3(0, 3, 0), autoSpin: 0.05 });
    } else if (step.id === "land") {
      const eased = easeInOut(progress);
      this.dronePosition.set(0, CRUISE_ALTITUDE + eased * (deck.y - CRUISE_ALTITUDE), 0);
      this.state.rpm = 1 - eased * 0.85;
      this.state.speed = 1.2 * (1 - eased);
      this.rig.frame({ radius: 3.2, phi: 1.2, target: new THREE.Vector3(0, DECK_Y + 0.3, 0), autoSpin: 0.08 });
    } else if (step.id === "fold") {
      this.dronePosition.copy(deck);
      this.state.rpm = Math.max(0, 0.15 * (1 - progress));
      this.state.fold = easeInOut(progress);
      this.state.speed = 0;
      this.rig.frame({ radius: 1.6, phi: 1.12, target: new THREE.Vector3(0, DECK_Y, 0), autoSpin: 0.12 });
    }
  }

  #updateDroneVisuals(dt) {
    const { rpm, phase } = this.state;

    for (const rotor of this.drone.rotors) {
      rotor.group.rotation.y += rotor.direction * rpm * 46 * dt;
      rotor.disc.material.opacity = clamp((rpm - 0.25) * 0.5, 0, 0.34);
    }

    this.#applyFold(this.state.fold);

    const flying = phase === "search" || (phase !== "idle" && this.dronePosition.y > DECK_Y + 0.3);
    const tilt = flying ? -0.12 : 0;
    this.drone.group.rotation.x = damp(this.drone.group.rotation.x, tilt, 4, dt);
    this.drone.group.rotation.z = flying ? Math.sin(this.state.elapsed * 1.7) * 0.03 : 0;
    this.drone.gimbal.rotation.x = damp(this.drone.gimbal.rotation.x, phase === "search" ? -0.9 : 0, 3, dt);

    // Beacon: slow pulse on the pad, fast blink in the air.
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
    this.scanCone.material.opacity = damp(this.scanCone.material.opacity, scanning ? 0.07 : 0, 3, dt);
    this.scanSpot.group.position.set(this.dronePosition.x, 0.015, this.dronePosition.z);
    this.scanSpot.group.scale.setScalar(spotRadius);
    const spotOpacity = scanning ? 0.3 + 0.12 * Math.sin(this.state.elapsed * 3) : 0;
    this.scanSpot.ring.material.opacity = damp(this.scanSpot.ring.material.opacity, spotOpacity, 4, dt);
    this.scanSpot.fill.material.opacity = damp(this.scanSpot.fill.material.opacity, scanning ? 0.3 : 0, 4, dt);

    if (scanning && this.state.elapsed - this.lastTrailAt > 0.07) {
      this.lastTrailAt = this.state.elapsed;
      this.trail.push(this.dronePosition);
    }

    this.state.altitude = Math.max(0, this.dronePosition.y - DECK_Y);
    if (phase !== "idle") {
      // ~18 min of flight time, so the readout moves visibly during a demo without emptying.
      this.state.battery = clamp(this.state.battery - dt * 0.09, 0, 100);
    }
  }

  #updateWorld(dt) {
    const pulse = 0.4 + 0.3 * Math.sin(this.state.elapsed * 2.2);
    this.pad.ring.material.opacity = 0.3 + pulse * 0.4;
    this.padLight.intensity = 1.4 + pulse * 1.8;
    this.base.deckRing.material.opacity = this.state.phase === "idle" ? 0.3 + pulse * 0.3 : 0.6;
    if (this.state.phase === "search") {
      this.base.mast.rotation.y = damp(
        this.base.mast.rotation.y,
        Math.atan2(this.dronePosition.x, this.dronePosition.z) * 0.6,
        1.5,
        dt
      );
    }
  }

  #frame = () => {
    if (!this.running) return;
    const dt = Math.min(this.clock.getDelta(), 1 / 20);
    this.lastDt = dt;
    this.state.elapsed += dt;

    if (this.state.phase === "launch") this.#updateLaunch(this.#advanceSteps(dt));
    else if (this.state.phase === "search") this.#updateSearch(dt);
    else if (this.state.phase === "recall") this.#updateRecall(this.#advanceSteps(dt));
    else {
      this.dronePosition.y = damp(this.dronePosition.y, DECK_Y + 0.03, 3, dt);
      this.state.speed = 0;
    }

    this.#updateDroneVisuals(dt);
    this.#updateWorld(dt);
    this.#place();
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

  totalLaunchSteps() {
    return LAUNCH_STEPS.length;
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

export { LAUNCH_STEPS, RECALL_STEPS };
