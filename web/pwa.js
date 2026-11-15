/**
 * Registers the service worker that makes the page installable as an app (PWA). Kept apart from
 * app.js so the search logic does not change. Service workers need https or localhost.
 */
if ("serviceWorker" in navigator && (location.protocol === "https:" || location.hostname === "localhost")) {
  window.addEventListener("load", () => {
    navigator.serviceWorker.register("sw.js").catch((error) => console.warn("No offline app:", error));
  });
}
