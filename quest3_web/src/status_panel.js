const ids = {
  ws: "wsStatus",
  xr: "xrStatus",
  left: "leftStatus",
  right: "rightStatus",
  fps: "fpsStatus",
  control: "controlStatus",
  error: "errorStatus",
};

export function setStatus(name, value) {
  const element = document.getElementById(ids[name]);
  if (element) element.textContent = value;
}

export function setError(message) {
  setStatus("error", message || "");
}
