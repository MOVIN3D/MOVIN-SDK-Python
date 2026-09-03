"""Dependency-free extension interfaces for mocap frame pipelines."""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class FrameProcessor(Protocol):
    """Transform a frame or the output of an earlier processor."""

    def process(self, frame: Any) -> Any:
        """Return the transformed value for downstream processors and sinks."""


@runtime_checkable
class FrameSink(Protocol):
    """Consume the output of a mocap frame pipeline."""

    def on_frame(self, frame: Any) -> Any:
        """Consume one value produced by the pipeline."""
