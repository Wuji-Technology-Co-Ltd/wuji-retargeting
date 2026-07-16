from pathlib import Path
import json
import re
from http.server import SimpleHTTPRequestHandler
from unittest.mock import patch
import asyncio


def test_gateway_and_cli_modules_import():
    import stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender as mock_sender
    import stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor as supervisor_cli
    import stardust_wuji_quest3_pc_retargeting.tools.run_orin_web_gateway as gateway_cli
    import stardust_wuji_quest3_pc_retargeting.web_gateway.websocket_relay as relay

    assert callable(mock_sender.build_mock_frame)
    assert callable(supervisor_cli.main)
    assert callable(gateway_cli.main)
    assert callable(relay.relay_websockets)


def test_quest3_web_static_files_exist_and_avoid_control_buttons():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    index = (root / "index.html").read_text(encoding="utf-8")
    js = "\n".join(path.read_text(encoding="utf-8") for path in (root / "src").glob("*.js"))

    assert "xrCanvas" in index
    assert "Enter XR" in index
    assert "Connect" in index
    assert "E-Stop" not in index
    assert "Start Teleop" not in index
    assert "tracking_frame" in js
    assert "XRWebGLLayer" in js
    assert "updateRenderState" in js
    assert "requiredFeatures" in js
    assert "hand-tracking" in js


def test_webxr_prefers_local_reference_space_when_entering_xr():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    webxr = (root / "src" / "webxr_session.js").read_text(encoding="utf-8")

    local_request = webxr.index('requestReferenceSpace("local")')
    local_floor_request = webxr.index('requestReferenceSpace("local-floor")')

    assert local_request < local_floor_request


def test_webxr_prefers_passthrough_ar_before_opaque_vr():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    webxr = (root / "src" / "webxr_session.js").read_text(encoding="utf-8")

    ar_mode = webxr.index('"immersive-ar"')
    vr_mode = webxr.index('"immersive-vr"')

    assert ar_mode < vr_mode
    assert "isSessionSupported(mode)" in webxr
    assert "requestSession(mode" in webxr


def test_webxr_uses_transparent_ar_layer_and_visible_vr_backdrop():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    webxr = (root / "src" / "webxr_session.js").read_text(encoding="utf-8")

    assert "environmentBlendMode" in webxr
    assert "alpha: transparent" in webxr
    assert "clearColor(0.0, 0.0, 0.0, 0.0)" in webxr
    assert "clearColor(0.10, 0.16, 0.20, 1.0)" in webxr


def test_quest3_web_modules_share_cache_busting_version():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    files = [root / "index.html", *sorted((root / "src").glob("*.js"))]
    local_module_urls = []

    for path in files:
        text = path.read_text(encoding="utf-8")
        local_module_urls.extend(re.findall(r'(?:src|from)="(\./[^"]+\.js(?:\?v=[^"]+)?)"', text))

    assert local_module_urls
    versions = {url.split("?v=", 1)[1] for url in local_module_urls if "?v=" in url}

    assert len(versions) == 1
    assert all("?v=" in url for url in local_module_urls)


def test_webxr_emits_xr_debug_for_session_and_frame_boundaries():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    webxr = (root / "src" / "webxr_session.js").read_text(encoding="utf-8")

    assert 'type: "xr_debug"' in webxr
    for stage in [
        "enter_xr_start",
        "request_session_ok",
        "xr_layer_ok",
        "reference_space_ok",
        "frame",
        "viewer_pose_missing",
        "tracking_frame_sent",
        "frame_error",
    ]:
        assert f'sendDebug("{stage}"' in webxr


def test_webxr_tracks_reference_space_reset_revision():
    root = Path(__file__).resolve().parents[1] / "quest3_web"
    webxr = (root / "src" / "webxr_session.js").read_text(encoding="utf-8")

    assert 'addEventListener("reset"' in webxr
    assert "referenceSpaceRevision += 1" in webxr
    assert "reference_space_revision: state.referenceSpaceRevision" in webxr
    assert "sendInactiveTrackingFrame();" in webxr


def test_relay_diagnostics_do_not_count_debug_or_hello_as_tracking_frames():
    from stardust_wuji_quest3_pc_retargeting.web_gateway.relay_diagnostics import (
        RelayCounters,
        record_quest_message,
    )

    counters = RelayCounters()
    lines = []

    should_forward_debug = record_quest_message(
        json.dumps({"type": "xr_debug", "stage": "enter_xr_start", "seq": 1}),
        counters,
        lines.append,
    )
    should_forward_hello = record_quest_message(
        json.dumps({"type": "hello", "client": "quest3_web"}),
        counters,
        lines.append,
    )
    should_forward_tracking = record_quest_message(
        json.dumps({"type": "tracking_frame", "seq": 1}),
        counters,
        lines.append,
    )

    assert should_forward_debug is False
    assert should_forward_hello is True
    assert should_forward_tracking is True
    assert counters.quest_to_control_messages == 3
    assert counters.quest_to_control_tracking_frames == 1
    assert any("[relay][xr_debug] stage=enter_xr_start" in line for line in lines)
    assert any("type=hello" in line for line in lines)
    assert any("tracking_frames=1" in line for line in lines)
    assert not any("quest->control frames=" in line for line in lines)


def test_static_server_marks_web_assets_no_store():
    from stardust_wuji_quest3_pc_retargeting.web_gateway.static_server import NoStoreStaticRequestHandler

    handler = NoStoreStaticRequestHandler.__new__(NoStoreStaticRequestHandler)
    sent_headers = []
    handler.send_header = lambda name, value: sent_headers.append((name, value))

    with patch.object(SimpleHTTPRequestHandler, "end_headers", lambda self: None):
        handler.end_headers()

    assert ("Cache-Control", "no-store, max-age=0") in sent_headers


def test_mock_sender_drains_supervisor_return_channel():
    async def scenario():
        import websockets

        from stardust_wuji_quest3_pc_retargeting.runtime.config import load_yaml_config
        from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import ControlPCSupervisor
        from stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender import send_mock_frames
        from stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor import serve_control_pc

        supervisor = ControlPCSupervisor(load_yaml_config("configs/arm/s1_quest3_default.yaml"), arm="left")
        supervisor.start()
        server = await serve_control_pc(supervisor, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        try:
            await asyncio.wait_for(
                send_mock_frames(f"ws://127.0.0.1:{port}", rate_hz=500.0, count=250, status_interval_sec=60.0),
                timeout=3.0,
            )
        finally:
            server.close()
            await server.wait_closed()
            supervisor.close()

    asyncio.run(scenario())
