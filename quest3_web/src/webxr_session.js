import { serializeHand, serializeArmWrist, emptyHand, emptyArmWrist } from "./hand_frame_serializer.js?v=20260716-wrist1";
import { state } from "./session_state.js?v=20260716-wrist1";
import { setError, setStatus } from "./status_panel.js?v=20260716-wrist1";
import { sendJson } from "./websocket_client.js?v=20260716-wrist1";

const requestedXrMode = new URLSearchParams(window.location.search).get("xr");
const XR_SESSION_MODES =
  requestedXrMode === "vr"
    ? ["immersive-vr"]
    : requestedXrMode === "ar"
      ? ["immersive-ar"]
      : ["immersive-ar", "immersive-vr"];
const XR_SESSION_OPTIONS = {
  requiredFeatures: ["hand-tracking"],
  optionalFeatures: ["local", "local-floor"],
};

let debugSeq = 0;
let xrFrameCount = 0;

function errorMessage(error) {
  return error && error.message ? error.message : String(error);
}

function inputSourcesSummary(session) {
  return Array.from(session.inputSources || []).map((source) => ({
    handedness: source.handedness || "none",
    targetRayMode: source.targetRayMode || "none",
    hasHand: Boolean(source.hand),
  }));
}

function shouldDebugFrame(frameIndex) {
  return frameIndex < 10 || frameIndex % 60 === 0;
}

function sendDebug(stage, fields = {}) {
  sendJson({
    schema: "quest3_web_teleop.v1",
    type: "xr_debug",
    seq: debugSeq++,
    stage,
    client_time_sec: performance.now() / 1000.0,
    xr_session_id: state.xrSessionId,
    ...fields,
  });
}

function transparentXrLayer(session) {
  return session.environmentBlendMode && session.environmentBlendMode !== "opaque";
}

function sourceForHand(session, handedness) {
  for (const source of session.inputSources) {
    if (source.handedness === handedness && source.hand) return source;
  }
  return null;
}

function updateFps() {
  state.sentFrames += 1;
  const now = performance.now();
  if (now - state.lastFpsTime >= 1000) {
    setStatus("fps", String(state.sentFrames));
    state.sentFrames = 0;
    state.lastFpsTime = now;
  }
}

function renderXrBackdrop(session) {
  if (!state.gl || !session.renderState.baseLayer) return;
  const transparent = transparentXrLayer(session);
  state.gl.bindFramebuffer(state.gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
  if (transparent) {
    state.gl.clearColor(0.0, 0.0, 0.0, 0.0);
  } else {
    state.gl.clearColor(0.10, 0.16, 0.20, 1.0);
  }
  state.gl.clear(state.gl.COLOR_BUFFER_BIT | state.gl.DEPTH_BUFFER_BIT);
}

function onFrame(time, frame) {
  const session = frame.session;
  const frameIndex = xrFrameCount++;
  const debugThisFrame = shouldDebugFrame(frameIndex);
  try {
    const inputSources = inputSourcesSummary(session);
    if (debugThisFrame) {
      sendDebug("frame", {
        frame_index: frameIndex,
        reference_space: state.referenceSpaceType,
        input_sources: inputSources,
      });
    }
    const viewerPose = frame.getViewerPose(state.referenceSpace);
    if (!viewerPose) {
      setStatus("xr", "no viewer pose");
      if (debugThisFrame) {
        sendDebug("viewer_pose_missing", {
          frame_index: frameIndex,
          reference_space: state.referenceSpaceType,
          input_sources: inputSources,
        });
      }
      return;
    }
    renderXrBackdrop(session);
    const leftSource = sourceForHand(session, "left");
    const rightSource = sourceForHand(session, "right");
    const left = serializeHand(leftSource, frame, state.referenceSpace);
    const right = serializeHand(rightSource, frame, state.referenceSpace);
    const leftArmWrist = serializeArmWrist(leftSource, frame, state.referenceSpace);
    const rightArmWrist = serializeArmWrist(rightSource, frame, state.referenceSpace);
    setStatus("reference", `${state.referenceSpaceType} rev=${state.referenceSpaceRevision}`);
    setStatus("left", left.valid ? "valid" : "invalid");
    setStatus("right", right.valid ? "valid" : "invalid");
    const transform = viewerPose.transform;
    const trackingFrame = {
      schema: "quest3_web_teleop.v1",
      type: "tracking_frame",
      seq: state.seq++,
      client_time_sec: performance.now() / 1000.0,
      xr_session_id: state.xrSessionId,
      hmd: {
        valid: true,
        position: [transform.position.x, transform.position.y, transform.position.z],
        orientation_xyzw: [transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w],
      },
      hands: { left, right },
      arm_wrists: { left: leftArmWrist, right: rightArmWrist },
      session: {
        active: true,
        visibility: document.visibilityState,
        reference_space: state.referenceSpaceType,
        reference_space_revision: state.referenceSpaceRevision,
      },
    };
    sendJson(trackingFrame);
    if (debugThisFrame) {
      sendDebug("tracking_frame_sent", {
        frame_index: frameIndex,
        seq: trackingFrame.seq,
        reference_space: state.referenceSpaceType,
        left_valid: left.valid,
        right_valid: right.valid,
        left_arm_wrist_valid: leftArmWrist.valid,
        right_arm_wrist_valid: rightArmWrist.valid,
        input_sources: inputSources,
      });
    }
    updateFps();
  } catch (error) {
    sendDebug("frame_error", {
      frame_index: frameIndex,
      reference_space: state.referenceSpaceType,
      error: errorMessage(error),
      input_sources: inputSourcesSummary(session),
    });
    setError(errorMessage(error));
  } finally {
    session.requestAnimationFrame(onFrame);
  }
}

function sendInactiveTrackingFrame() {
  sendJson({
    schema: "quest3_web_teleop.v1",
    type: "tracking_frame",
    seq: state.seq++,
    client_time_sec: performance.now() / 1000.0,
    xr_session_id: state.xrSessionId,
    hmd: { valid: false, position: [0, 0, 0], orientation_xyzw: [0, 0, 0, 1] },
    hands: { left: emptyHand(), right: emptyHand() },
    arm_wrists: { left: emptyArmWrist(), right: emptyArmWrist() },
    session: {
      active: false,
      visibility: document.visibilityState,
      reference_space: state.referenceSpaceType,
      reference_space_revision: state.referenceSpaceRevision,
    },
  });
}

async function createXrLayer(session) {
  const canvas = document.getElementById("xrCanvas");
  const transparent = transparentXrLayer(session);
  const gl = canvas.getContext("webgl", { xrCompatible: true, alpha: transparent });
  if (!gl) throw new Error("WebGL is unavailable for XR rendering");
  if (gl.makeXRCompatible) {
    await gl.makeXRCompatible();
  }
  session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl, { alpha: transparent }) });
  state.gl = gl;
}

async function requestXrSession() {
  const failures = [];
  for (const mode of XR_SESSION_MODES) {
    const supported = await navigator.xr.isSessionSupported(mode);
    sendDebug("session_support", { session_mode: mode, supported });
    if (!supported) continue;
    try {
      const session = await navigator.xr.requestSession(mode, XR_SESSION_OPTIONS);
      state.xrSessionMode = mode;
      return session;
    } catch (error) {
      failures.push(`${mode}: ${errorMessage(error)}`);
      sendDebug("request_session_failed", { session_mode: mode, error: errorMessage(error) });
    }
  }
  throw new Error(`No supported XR session mode. ${failures.join("; ")}`);
}

async function requestReferenceSpace(session) {
  try {
    state.referenceSpaceType = "local";
    const referenceSpace = await session.requestReferenceSpace("local");
    sendDebug("reference_space_ok", { reference_space: "local" });
    return referenceSpace;
  } catch (error) {
    sendDebug("reference_space_failed", { reference_space: "local", error: errorMessage(error) });
    try {
      state.referenceSpaceType = "local-floor";
      const referenceSpace = await session.requestReferenceSpace("local-floor");
      sendDebug("reference_space_ok", { reference_space: "local-floor" });
      return referenceSpace;
    } catch (localFloorError) {
      sendDebug("reference_space_failed", { reference_space: "local-floor", error: errorMessage(localFloorError) });
      state.referenceSpaceType = "viewer";
      const referenceSpace = await session.requestReferenceSpace("viewer");
      sendDebug("reference_space_ok", { reference_space: "viewer" });
      return referenceSpace;
    }
  }
}

export async function enterXr() {
  sendDebug("enter_xr_start", {
    visibility: document.visibilityState,
    has_navigator_xr: Boolean(navigator.xr),
  });
  if (!navigator.xr) {
    setError("WebXR is unavailable in this browser");
    sendDebug("enter_xr_error", { error: "WebXR is unavailable in this browser" });
    return;
  }
  try {
    const session = await requestXrSession();
    state.xrSession = session;
    state.xrSessionId = crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
    state.referenceSpaceRevision = 0;
    xrFrameCount = 0;
    sendDebug("request_session_ok", {
      session_mode: state.xrSessionMode,
      environment_blend_mode: session.environmentBlendMode,
      visibility_state: session.visibilityState,
      input_sources: inputSourcesSummary(session),
    });
    await createXrLayer(session);
    sendDebug("xr_layer_ok");
    state.referenceSpace = await requestReferenceSpace(session);
    state.referenceSpace.addEventListener("reset", () => {
      state.referenceSpaceRevision += 1;
      setStatus("reference", `${state.referenceSpaceType} rev=${state.referenceSpaceRevision}`);
      sendDebug("reference_space_reset", {
        reference_space: state.referenceSpaceType,
        reference_space_revision: state.referenceSpaceRevision,
      });
    });
    setStatus("reference", `${state.referenceSpaceType} rev=${state.referenceSpaceRevision}`);
    setStatus("xr", `running ${state.xrSessionMode}`);
    session.addEventListener("end", () => {
      sendDebug("session_end");
      sendInactiveTrackingFrame();
      setStatus("xr", "ended");
      setStatus("left", "invalid");
      setStatus("right", "invalid");
    });
    session.requestAnimationFrame(onFrame);
  } catch (error) {
    sendDebug("enter_xr_error", { error: errorMessage(error) });
    setError(errorMessage(error));
  }
}
