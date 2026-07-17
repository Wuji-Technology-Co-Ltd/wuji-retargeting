import { state } from "./session_state.js?v=20260716-wrist1";
import { setError, setStatus } from "./status_panel.js?v=20260716-wrist1";

export function connectWebSocket() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const url = `${scheme}://${location.hostname}:9002`;
  const socket = new WebSocket(url);
  state.socket = socket;
  socket.addEventListener("open", () => {
    setStatus("ws", "connected");
    socket.send(JSON.stringify({
      schema: "quest3_web_teleop.v1",
      type: "hello",
      client: "quest3_web",
      version: "0.1.0",
      webxr: { immersive_vr: Boolean(navigator.xr), hand_tracking: true },
    }));
    document.getElementById("enterXrButton").disabled = false;
  });
  socket.addEventListener("close", () => setStatus("ws", "disconnected"));
  socket.addEventListener("error", () => setError("WebSocket error"));
  socket.addEventListener("message", (event) => {
    try {
      const message = JSON.parse(event.data);
      if (message.type === "control_state") setStatus("control", message.state);
    } catch {
      setError("Control PC message parse error");
    }
  });
}

export function sendJson(payload) {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify(payload));
  }
}
