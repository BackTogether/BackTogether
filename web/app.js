/**
 * Page logic: the request form, the photo, the POST that will start the real drone, and the
 * launch / search views driven by the 3D stage in scene.js.
 */

import { createStage, LAUNCH_STEPS } from "./scene.js";

const CONFIG = window.FINDMYCHILD_CONFIG || {};
const SEARCH_PHASES = ["launch", "search", "recall"];
const FOUND_STEP_LABEL = "Bobac3 going to the child";
// Demo mode has no robot to report a sighting, so the child is "found" at this much coverage.
const DEMO_FOUND_COVERAGE = 20;

const $ = (selector) => document.querySelector(selector);
const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const el = {
  form: $("#request-form"),
  childName: $("#child-name"),
  photoInput: $("#photo"),
  dropzone: $("#dropzone"),
  photoPreview: $("#photo-preview"),
  photoName: $("#photo-name"),
  photoClear: $("#photo-clear"),
  submit: $("#submit"),
  formError: $("#form-error"),
  mission: $("#mission"),
  missionSteps: $("#mission-steps"),
  missionPhoto: $("#mission-photo"),
  missionChild: $("#mission-child"),
  missionId: $("#mission-id"),
  missionMode: $("#mission-mode"),
  missionElapsed: $("#mission-elapsed"),
  altitude: $("#stat-altitude"),
  speed: $("#stat-speed"),
  battery: $("#stat-battery"),
  coverage: $("#stat-coverage"),
  coverageBar: $("#coverage-bar"),
  cancel: $("#cancel-search"),
  liveStatus: $("#live-status"),
  eventName: $("#event-name"),
  year: $("#year"),
};

/** The photo the parent picked, already downscaled: { dataUrl, mediaType, name, width, height }. */
let photo = null;
let stage = null;
let mission = null; // { id, startedAt, child, mode }
let elapsedTimer = null;

/* ------------------------------------------------------------------- photo */

/** Draws the chosen image into a canvas no larger than maxSize, to keep the POST small. */
async function downscaleImage(file, maxSize, quality) {
  const bitmap = await createImageBitmap(file);
  const scale = Math.min(1, maxSize / Math.max(bitmap.width, bitmap.height));
  const width = Math.max(1, Math.round(bitmap.width * scale));
  const height = Math.max(1, Math.round(bitmap.height * scale));

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(bitmap, 0, 0, width, height);
  bitmap.close?.();

  return {
    dataUrl: canvas.toDataURL("image/jpeg", quality),
    mediaType: "image/jpeg",
    name: file.name,
    width,
    height,
  };
}

async function acceptPhoto(file) {
  if (!file) return;
  if (!file.type.startsWith("image/")) {
    showFormError("That file is not an image. A normal phone photo of your child works best.");
    return;
  }
  if (file.size > 25 * 1024 * 1024) {
    showFormError("That image is larger than 25 MB. Please pick a smaller one.");
    return;
  }
  try {
    photo = await downscaleImage(file, CONFIG.photoMaxSize || 1024, CONFIG.photoQuality || 0.82);
  } catch (error) {
    console.error(error);
    showFormError("That image could not be read. Try another photo.");
    return;
  }
  el.photoPreview.src = photo.dataUrl;
  // Not the file name: a phone photo name is long and useless to the parent.
  el.photoName.textContent = `Photo ready · ${photo.width}×${photo.height}`;
  el.dropzone.dataset.state = "filled";
  showFormError("");
}

function clearPhoto() {
  photo = null;
  el.photoInput.value = "";
  el.photoPreview.removeAttribute("src");
  el.photoName.textContent = "";
  el.dropzone.dataset.state = "empty";
}

/* -------------------------------------------------------------- the request */

function requestId() {
  const random = crypto.randomUUID ? crypto.randomUUID().slice(0, 8) : Math.random().toString(16).slice(2, 10);
  return `FMC-${new Date().toISOString().slice(0, 10).replace(/-/g, "")}-${random.toUpperCase()}`;
}

/**
 * The body POSTed to the robot bridge. Documented in README.md: the drone side reads
 * `photo.data` (base64 JPEG) to build the face model and `child.name` to label the search.
 */
function buildPayload(form, id) {
  const data = new FormData(form);

  return {
    schema: "find-my-child/search-request@2",
    request_id: id,
    requested_at: new Date().toISOString(),
    event: CONFIG.eventName || null,
    child: {
      name: String(data.get("child_name") || "").trim(),
    },
    photo: {
      filename: photo.name,
      media_type: photo.mediaType,
      width: photo.width,
      height: photo.height,
      // Base64 without the "data:image/jpeg;base64," prefix.
      data: photo.dataUrl.split(",")[1],
    },
    client: {
      page: location.href,
      user_agent: navigator.userAgent,
    },
  };
}

/** POSTs the request. Returns the mode actually used: "live" or "demo". */
async function sendRequest(payload) {
  if (!CONFIG.apiUrl) return { mode: "demo" };

  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), CONFIG.requestTimeout || 12000);
  try {
    const headers = { "Content-Type": "application/json" };
    if (CONFIG.apiToken) headers.Authorization = `Bearer ${CONFIG.apiToken}`;

    const response = await fetch(CONFIG.apiUrl, {
      method: "POST",
      headers,
      body: JSON.stringify(payload),
      signal: controller.signal,
    });
    if (!response.ok) {
      const text = await response.text().catch(() => "");
      throw new Error(`the robot bridge answered ${response.status} ${response.statusText} ${text}`.trim());
    }
    return { mode: "live", body: await response.json().catch(() => null) };
  } finally {
    clearTimeout(timeout);
  }
}

/* ------------------------------------------------------------------ the UI */

function showFormError(message) {
  el.formError.textContent = message;
  el.formError.hidden = !message;
}

function buildStepList() {
  el.missionSteps.innerHTML = "";
  const steps = [...LAUNCH_STEPS.map((step) => step.label), "Scanning the event zone", FOUND_STEP_LABEL];
  steps.forEach((label, index) => {
    const item = document.createElement("li");
    item.className = "step";
    item.dataset.index = String(index);
    item.innerHTML = `<span class="step-dot" aria-hidden="true"></span><span class="step-label">${label}</span>`;
    el.missionSteps.append(item);
  });
}

function markStep(index) {
  for (const item of el.missionSteps.children) {
    const itemIndex = Number(item.dataset.index);
    item.dataset.state = itemIndex < index ? "done" : itemIndex === index ? "active" : "todo";
  }
}

function setView(view) {
  document.body.dataset.view = view;
  if (view === "mission") {
    el.mission.hidden = false;
    el.mission.scrollIntoView({ behavior: reducedMotion ? "auto" : "smooth", block: "start" });
  }
}

function formatElapsed(ms) {
  const total = Math.floor(ms / 1000);
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

function startTelemetryLoop() {
  if (elapsedTimer) clearInterval(elapsedTimer);
  elapsedTimer = setInterval(() => {
    if (!mission) return;
    el.missionElapsed.textContent = formatElapsed(Date.now() - mission.startedAt);
    const telemetry = stage?.telemetry();
    if (!telemetry) return;
    el.altitude.textContent = `${telemetry.altitude.toFixed(1)} m`;
    el.speed.textContent = `${telemetry.speed.toFixed(1)} m/s`;
    el.battery.textContent = `${Math.round(telemetry.battery)} %`;
    el.coverage.textContent = `${Math.round(telemetry.coverage)} %`;
    el.coverageBar.style.setProperty("--value", `${telemetry.coverage}%`);
    if (mission.mode === "demo" && telemetry.phase === "search" && telemetry.coverage >= DEMO_FOUND_COVERAGE) {
      stage.setPhase("found");
    }
  }, 250);
}

function stopTelemetryLoop() {
  if (elapsedTimer) clearInterval(elapsedTimer);
  elapsedTimer = null;
}

/* --------------------------------------------------------------- the flight */

async function onSubmit(event) {
  event.preventDefault();
  if (!el.childName.value.trim()) {
    showFormError("Please type your child's name.");
    el.childName.focus();
    return;
  }
  if (!photo) {
    showFormError("A photo of your child is what the drone searches with: please add one.");
    el.dropzone.focus();
    return;
  }

  const id = requestId();
  const payload = buildPayload(el.form, id);

  el.submit.disabled = true;
  el.submit.dataset.state = "sending";
  showFormError("");

  let result;
  try {
    result = await sendRequest(payload);
  } catch (error) {
    el.submit.disabled = false;
    el.submit.dataset.state = "";
    showFormError(
      `The search could not be started: ${error.message}. Tell event staff straight away, then try again.`
    );
    return;
  }

  mission = {
    id,
    startedAt: Date.now(),
    child: payload.child.name || "your child",
    mode: result.mode,
  };

  el.missionPhoto.src = photo.dataUrl;
  el.missionChild.textContent = mission.child;
  el.missionId.textContent = id;
  el.missionMode.textContent = result.mode === "live" ? "Request sent to the robot" : "Demo mode · no robot endpoint set";
  el.missionMode.dataset.mode = result.mode;
  el.liveStatus.textContent = `Search ${id} started. The Hover Air X1 is launching from the Bobac3 base.`;

  buildStepList();
  markStep(0);
  setView("mission");
  startTelemetryLoop();
  stage?.setPhase("launch");

  el.submit.disabled = false;
  el.submit.dataset.state = "";
}

function onCancel() {
  if (!mission) return;
  el.cancel.disabled = true;
  el.liveStatus.textContent = "Search cancelled. The drone is returning to the base.";
  stage?.setPhase("recall");
}

function onPhase(phase) {
  if (phase === "search") {
    markStep(LAUNCH_STEPS.length);
    el.liveStatus.textContent = `The drone is scanning the event zone for ${mission?.child || "your child"}.`;
  }
  if (phase === "found") {
    markStep(LAUNCH_STEPS.length + 1);
    el.liveStatus.textContent = `${mission?.child || "Your child"} was spotted. The Bobac3 robot is driving there.`;
  }
  if (phase === "idle" && mission) {
    // The recall sequence finished: back to the form, ready for another request.
    mission = null;
    stopTelemetryLoop();
    el.cancel.disabled = false;
    setView("form");
    el.mission.hidden = true;
    el.liveStatus.textContent = "The drone is back over the base. You can start a new search.";
  }
  void SEARCH_PHASES;
}

/* ------------------------------------------------------------------- setup */

function wirePhotoInput() {
  el.photoInput.addEventListener("change", (event) => acceptPhoto(event.target.files?.[0]));
  el.photoClear.addEventListener("click", clearPhoto);

  el.dropzone.addEventListener("click", () => el.photoInput.click());
  el.dropzone.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      el.photoInput.click();
    }
  });
  for (const type of ["dragenter", "dragover"]) {
    el.dropzone.addEventListener(type, (event) => {
      event.preventDefault();
      el.dropzone.dataset.drag = "over";
    });
  }
  for (const type of ["dragleave", "drop"]) {
    el.dropzone.addEventListener(type, (event) => {
      event.preventDefault();
      delete el.dropzone.dataset.drag;
    });
  }
  el.dropzone.addEventListener("drop", (event) => acceptPhoto(event.dataTransfer?.files?.[0]));
}

function init() {
  el.eventName.textContent = CONFIG.eventName || "this event";
  el.year.textContent = String(new Date().getFullYear());
  wirePhotoInput();
  clearPhoto();

  el.form.addEventListener("submit", onSubmit);
  el.cancel.addEventListener("click", onCancel);

  stage = createStage($("#stage-canvas"), {
    reducedMotion,
    onStep: (step) => {
      // Only the launch and search steps are in the list; later phases just update the status.
      if (step.phase === "launch" || step.phase === "search") markStep(step.index);
      if (step.label) el.liveStatus.textContent = step.label;
    },
    onPhase,
  });

  if (!stage) {
    document.body.dataset.stage = "unavailable";
  }

  for (const link of document.querySelectorAll('a[href^="#"]')) {
    link.addEventListener("click", (event) => {
      const target = document.querySelector(link.getAttribute("href"));
      if (!target) return;
      event.preventDefault();
      target.scrollIntoView({ behavior: reducedMotion ? "auto" : "smooth", block: "start" });
      target.querySelector("input, select, textarea")?.focus({ preventScroll: true });
    });
  }
}

init();
