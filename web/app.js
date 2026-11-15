/**
 * Page logic: the request form, the photo, the POST that will start the real drone, and the
 * confirmation that the drone is on its way.
 */

const CONFIG = window.FINDMYCHILD_CONFIG || {};

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
  request: $("#request"),
  mission: $("#mission"),
  liveStatus: $("#live-status"),
  eventName: $("#event-name"),
  year: $("#year"),
};

/** The photo the parent picked, already downscaled: { dataUrl, mediaType, name, width, height }. */
let photo = null;

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

function setView(view) {
  document.body.dataset.view = view;
  el.request.hidden = view === "mission"; // once sent, only the confirmation is left on screen
  if (view === "mission") {
    el.mission.hidden = false;
    el.mission.scrollIntoView({ behavior: reducedMotion ? "auto" : "smooth", block: "start" });
  }
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

  try {
    await sendRequest(payload);
  } catch (error) {
    el.submit.disabled = false;
    el.submit.dataset.state = "";
    showFormError(
      `The search could not be started: ${error.message}. Tell event staff straight away, then try again.`
    );
    return;
  }

  el.liveStatus.textContent = "The drone is on its way. You will soon get a notification with the video call.";
  setView("mission");

  el.submit.disabled = false;
  el.submit.dataset.state = "";
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
