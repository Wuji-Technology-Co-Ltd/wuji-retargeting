export const state = {
  socket: null,
  xrSession: null,
  referenceSpace: null,
  gl: null,
  seq: 0,
  lastFpsTime: performance.now(),
  sentFrames: 0,
  xrSessionId: crypto.randomUUID ? crypto.randomUUID() : String(Date.now()),
  referenceSpaceRevision: 0,
};
