from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from time import monotonic_ns
from typing import Generic, TypeVar


FrameT = TypeVar("FrameT")


@dataclass(frozen=True)
class TrackingSnapshot(Generic[FrameT]):
    frame: FrameT
    receive_time_ns: int
    generation: int


class LatestTrackingBuffer(Generic[FrameT]):
    def __init__(self) -> None:
        self._lock = Lock()
        self._snapshot: TrackingSnapshot[FrameT] | None = None
        self._generation = 0

    def publish(self, frame: FrameT, receive_time_ns: int | None = None) -> TrackingSnapshot[FrameT]:
        received = monotonic_ns() if receive_time_ns is None else int(receive_time_ns)
        with self._lock:
            self._generation += 1
            self._snapshot = TrackingSnapshot(frame, received, self._generation)
            return self._snapshot

    def snapshot(self) -> TrackingSnapshot[FrameT] | None:
        with self._lock:
            return self._snapshot

    @property
    def size(self) -> int:
        return 0 if self.snapshot() is None else 1

    @property
    def published_count(self) -> int:
        with self._lock:
            return self._generation
