/**
 * Site configuration. This is the only file to edit when the robot endpoint changes: it is plain
 * JS loaded before the app, so it can be changed on a deployed site without a rebuild.
 */
window.FINDMYCHILD_CONFIG = {
  // Where a search request is POSTed: an ntfy topic, which accepts posts from this page (CORS)
  // and keeps a large body as an attachment. scripts/web_inbox.py listens to it on the robot or
  // the drone laptop and saves the child's photos in a folder named after them. Empty = demo
  // mode: nothing is sent. Anyone who reads this file can post to the topic: test photos only.
  apiUrl: "https://ntfy.sh/<NTFY_TOPIC>",

  // Optional shared secret, sent as "Authorization: Bearer <token>". Anything in this file is
  // public once deployed, so use a token that only starts a search, never an admin credential.
  apiToken: "",

  // Longest side of each uploaded photo after the browser downscales it, and its JPEG quality.
  // 1024 px keeps a face recognisable while each photo stays around 150 kB.
  photoMaxSize: 1024,
  photoQuality: 0.82,
  // Most photos one request carries.
  photoMaxCount: 5,

  // How long to wait for the POST before giving up, in milliseconds.
  requestTimeout: 12000,

  // Name of the event, shown in the header and sent with the request.
  eventName: "Leukerbad Open Air",
};
