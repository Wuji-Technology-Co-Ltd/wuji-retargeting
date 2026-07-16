import { connectWebSocket } from "./websocket_client.js?v=20260707-ar1";
import { enterXr } from "./webxr_session.js?v=20260707-ar1";
import { setError } from "./status_panel.js?v=20260707-ar1";

document.getElementById("connectButton").addEventListener("click", connectWebSocket);
document.getElementById("enterXrButton").addEventListener("click", () => {
  enterXr().catch((error) => setError(error.message));
});

if (!window.isSecureContext) {
  setError("WebXR requires a secure context such as HTTPS.");
}
