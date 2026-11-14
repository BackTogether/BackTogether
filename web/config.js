/**
 * Site configuration. This is the only file to edit when the robot endpoint changes: it is plain
 * JS loaded before the app, so it can be changed on a deployed site without a rebuild.
 */
window.FINDMYCHILD_CONFIG = {
  // Where a search request is POSTed. Empty = demo mode: the page runs the whole launch
  // animation but sends nothing. Set it to the drone bridge, e.g.
  // "https://robot.example.org/api/v1/search-requests" or "http://<SERVER_IP>:8080/api/v1/search-requests".
  // The endpoint must send CORS headers (Access-Control-Allow-Origin) for the Pages origin,
  // and must be https when the site is served over https.
  apiUrl: "",

  // Optional shared secret, sent as "Authorization: Bearer <token>". Anything in this file is
  // public once deployed, so use a token that only starts a search, never an admin credential.
  apiToken: "",

  // Longest side of the uploaded photo after the browser downscales it, and its JPEG quality.
  // 1024 px keeps a face recognisable while the request stays around 150 kB.
  photoMaxSize: 1024,
  photoQuality: 0.82,

  // How long to wait for the POST before giving up, in milliseconds.
  requestTimeout: 12000,

  // Name of the event, shown in the header and sent with the request.
  eventName: "Leukerbad Open Air",

  // Zones the parent can pick as "last seen". Keep them in sync with the map the robot uses.
  zones: ["Main stage", "Food court", "Playground", "Ticket gate", "Parking", "Lake path"],
};
