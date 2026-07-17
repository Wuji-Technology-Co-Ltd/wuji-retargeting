from __future__ import annotations

from dataclasses import dataclass, field
from threading import Event, Lock, Thread
from time import monotonic_ns, sleep
from typing import Callable, Mapping, Protocol

import numpy as np

from stardust_wuji_quest3_pc_retargeting.conversion.webxr_to_mp21 import WebXRToMP21Converter
from stardust_wuji_quest3_pc_retargeting.safety.hand_safety_filter import HandSafetyFilter

from .command_bridge import HandBridgeFrame, HandBridgeSide, HandCommandSink
from .retarget_pipeline import RetargetPipeline


class TrackingBuffer(Protocol):
    def snapshot(self): ...


@dataclass
class HandLoopStats:
    cycles: int = 0
    processed_frames: int = 0
    published_frames: int = 0
    stale_frames: int = 0
    retarget_failures: int = 0
    sink_failures: int = 0
    loop_failures: int = 0
    process_time_ns: list[int] = field(default_factory=list)

    def record_process_time(self, value: int, maximum_samples: int = 10_000) -> None:
        self.process_time_ns.append(int(value))
        if len(self.process_time_ns) > maximum_samples:
            del self.process_time_ns[: len(self.process_time_ns) - maximum_samples]


class HandControlLoop:
    def __init__(
        self,
        tracking_buffer: TrackingBuffer,
        converters: Mapping[str, WebXRToMP21Converter],
        retargeters: Mapping[str, RetargetPipeline],
        filters: Mapping[str, HandSafetyFilter],
        sink: HandCommandSink,
        running_provider: Callable[[], bool],
        state_provider: Callable[[], str],
        poll_rate_hz: float = 120.0,
        stale_emit_sec: float = 0.20,
        clock_ns: Callable[[], int] = monotonic_ns,
        sleeper: Callable[[float], None] = sleep,
    ) -> None:
        self.tracking_buffer = tracking_buffer
        self.converters = dict(converters)
        self.retargeters = dict(retargeters)
        self.filters = dict(filters)
        if set(self.converters) != {"left", "right"}:
            raise ValueError("hand control loop requires left and right converters")
        if set(self.retargeters) != {"left", "right"}:
            raise ValueError("hand control loop requires left and right retargeters")
        if set(self.filters) != {"left", "right"}:
            raise ValueError("hand control loop requires left and right safety filters")
        self.sink = sink
        self.running_provider = running_provider
        self.state_provider = state_provider
        self.poll_rate_hz = float(poll_rate_hz)
        if self.poll_rate_hz <= 0.0:
            raise ValueError("hand control poll rate must be positive")
        self.period_sec = 1.0 / self.poll_rate_hz
        self.stale_emit_ns = int(float(stale_emit_sec) * 1e9)
        self.clock_ns = clock_ns
        self.sleeper = sleeper
        self.stats = HandLoopStats()
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._last_generation = 0
        self._stale_generation: int | None = None
        self._status_lock = Lock()
        self.last_error = ""
        self.last_frame: HandBridgeFrame | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self.run, name="HandControlLoop", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        self.sink.close()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.tick()
            except Exception as exc:
                self.stats.loop_failures += 1
                with self._status_lock:
                    self.last_error = f"hand control loop failed: {exc}"
            self.sleeper(self.period_sec)

    def tick(self, now_ns: int | None = None) -> HandBridgeFrame | None:
        now = self.clock_ns() if now_ns is None else int(now_ns)
        self.stats.cycles += 1
        snapshot = self.tracking_buffer.snapshot()
        if snapshot is None:
            return None
        age_ns = max(0, now - int(snapshot.receive_time_ns))
        is_new = snapshot.generation != self._last_generation
        should_emit_stale = (
            not is_new
            and age_ns > self.stale_emit_ns
            and self._stale_generation != snapshot.generation
        )
        if not is_new and not should_emit_stale:
            return None
        started = self.clock_ns()
        frame = self._build_frame(
            snapshot.frame,
            receive_time_ns=int(snapshot.receive_time_ns),
            frame_age_sec=age_ns / 1e9,
            force_stale=should_emit_stale,
        )
        if is_new:
            self._last_generation = snapshot.generation
            self._stale_generation = None
            self.stats.processed_frames += 1
        else:
            self._stale_generation = snapshot.generation
            self.stats.stale_frames += 1
        try:
            self.sink.publish(frame)
            self.stats.published_frames += 1
            sink_error = ""
        except Exception as exc:
            self.stats.sink_failures += 1
            sink_error = f"hand command sink failed: {exc}"
        self.stats.record_process_time(max(0, self.clock_ns() - started))
        with self._status_lock:
            self.last_frame = frame
            self.last_error = sink_error
        return frame

    def status(self) -> dict:
        with self._status_lock:
            frame = self.last_frame
            error = self.last_error
        hands = {} if frame is None else frame.hands
        process_ms = np.asarray(self.stats.process_time_ns, dtype=float) / 1e6
        return {
            "enabled": True,
            "sink": self.sink.name,
            "published_frames": int(self.stats.published_frames),
            "processed_frames": int(self.stats.processed_frames),
            "stale_frames": int(self.stats.stale_frames),
            "retarget_failures": int(self.stats.retarget_failures),
            "sink_failures": int(self.stats.sink_failures),
            "loop_failures": int(self.stats.loop_failures),
            "last_error": error,
            "last_seq": None if frame is None else int(frame.seq),
            "command_enabled": {
                side: bool(hands and hands[side].enabled)
                for side in ("left", "right")
            },
            "safety_state": {
                side: "IDLE" if not hands else str(hands[side].safety_state)
                for side in ("left", "right")
            },
            "process_p95_ms": (
                0.0 if process_ms.size == 0 else float(np.percentile(process_ms, 95))
            ),
        }

    def _build_frame(
        self,
        tracking_frame,
        *,
        receive_time_ns: int,
        frame_age_sec: float,
        force_stale: bool,
    ) -> HandBridgeFrame:
        running = bool(self.running_provider()) and not force_stale
        hands: dict[str, HandBridgeSide] = {}
        for side in ("left", "right"):
            source = tracking_frame.hands[side]
            tracking_valid = bool(
                not force_stale
                and tracking_frame.session.active
                and tracking_frame.hmd.valid
                and source.valid
            )
            mp21 = np.zeros((21, 3), dtype=float)
            raw_qpos = np.zeros(20, dtype=float)
            retarget_error = ""
            if tracking_valid:
                try:
                    mp21 = self.converters[side].convert(source)
                    raw_qpos = np.asarray(
                        self.retargeters[side].retarget(mp21),
                        dtype=float,
                    ).reshape(20)
                    if not np.isfinite(raw_qpos).all():
                        raise ValueError("retarget output contains non-finite values")
                except Exception as exc:
                    tracking_valid = False
                    retarget_error = f"retarget failed: {exc}"
                    self.stats.retarget_failures += 1
            command = self.filters[side].filter(
                raw_qpos if tracking_valid else None,
                frame_age_sec=frame_age_sec,
                deadman=running,
                tracking_valid=tracking_valid,
            )
            reason = retarget_error or command.reason
            hands[side] = HandBridgeSide(
                valid=tracking_valid,
                mp21=mp21.astype(float).tolist(),
                raw_qpos=raw_qpos.astype(float).tolist(),
                safe_qpos=list(command.qpos),
                enabled=bool(command.enabled and running),
                safety_state=command.state.value,
                reason=reason,
            )
        return HandBridgeFrame(
            seq=int(tracking_frame.seq),
            client_time_sec=float(tracking_frame.client_time_sec),
            receive_time_ns=int(receive_time_ns),
            xr_session_id=str(tracking_frame.xr_session_id),
            teleop_state=str(self.state_provider()),
            hands=hands,
        )
