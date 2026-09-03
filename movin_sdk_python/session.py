"""High-level receiver and recording lifecycle for MOVIN Studio streams."""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Iterable

from .mocap_receiver import MocapReceiver
from .recording import OscRecorder


ExtensionErrorHandler = Callable[[object, Exception], None]


class MovinSession:
    """Coordinate mocap reception, raw OSC recording, and optional extensions.

    The receiver records OSC messages before frames enter the optional processor
    and sink pipeline. Retargeting or viewer failures therefore never interrupt
    raw mocap reception and recording.
    """

    def __init__(
        self,
        port: int = 11235,
        *,
        host: str = "0.0.0.0",
        receiver=None,
        processors: Iterable[object] | None = None,
        sinks: Iterable[object] | None = None,
        on_extension_error: ExtensionErrorHandler | None = None,
    ):
        self.receiver = (
            receiver
            if receiver is not None
            else MocapReceiver(host=host, port=port)
        )
        self._processors = list(processors or ())
        self._sinks = list(sinks or ())
        self._on_extension_error = on_extension_error
        self._extension_lock = threading.RLock()
        self._recorder: OscRecorder | None = None
        self._started = False
        self._logger = logging.getLogger(__name__)

        self._validate_extensions(self._processors, "processor", "process")
        self._validate_extensions(self._sinks, "sink", "on_frame")

    @staticmethod
    def _validate_extensions(extensions, kind: str, method_name: str):
        for extension in extensions:
            if not callable(getattr(extension, method_name, None)):
                raise TypeError(
                    f"{kind} {extension!r} must define {method_name}(frame)"
                )

    @property
    def is_running(self) -> bool:
        """Whether this session has started its receiver."""
        return self._started

    @property
    def is_recording(self) -> bool:
        """Whether raw OSC messages are currently being recorded."""
        return self._recorder is not None

    def start(self):
        """Start the configured receiver and return this session."""
        if not self._started:
            self.receiver.start()
            self._started = True
        return self

    def stop(self):
        """Stop reception, save any active recording, and release resources."""
        try:
            if self._started:
                self.receiver.stop()
        finally:
            self._started = False
            if self._recorder is not None:
                self.stop_recording(save=True)

    def start_recording(self, output_path: str, stream_type: str = "auto") -> OscRecorder:
        """Attach a new raw OSC recorder to the live receiver."""
        if self._recorder is not None:
            raise RuntimeError("This session is already recording")
        if not hasattr(self.receiver, "recorder"):
            raise TypeError("The configured receiver does not support raw OSC recording")

        recorder = OscRecorder(output_path, stream_type=stream_type)
        self.receiver.recorder = recorder
        self._recorder = recorder
        return recorder

    def stop_recording(self, save: bool = True) -> OscRecorder | None:
        """Detach and optionally save the current recorder."""
        recorder = self._recorder
        if recorder is None:
            return None

        if getattr(self.receiver, "recorder", None) is recorder:
            self.receiver.recorder = None
        self._recorder = None
        if save:
            recorder.save()
        return recorder

    def get_latest_raw_frame(self):
        """Return the newest receiver frame without running extensions."""
        return self.receiver.get_latest_frame()

    def get_latest_frame(self):
        """Return the newest frame after processors and notify attached sinks.

        With no processors this returns the original mocap frame, making the
        default session behavior equivalent to ``MocapReceiver``.
        """
        frame = self.get_latest_raw_frame()
        if frame is None:
            return None
        return self.process_frame(frame)

    def process_frame(self, frame: Any) -> Any:
        """Run one already-received frame through optional extensions."""
        with self._extension_lock:
            processors = tuple(self._processors)
            sinks = tuple(self._sinks)

        value = frame
        processor_failed = False
        for processor in processors:
            try:
                value = processor.process(value)
            except Exception as exc:  # extensions must not stop core capture
                processor_failed = True
                self._report_extension_error(processor, exc)
                break

        if not processor_failed:
            for sink in sinks:
                try:
                    sink.on_frame(value)
                except Exception as exc:  # notify remaining sinks independently
                    self._report_extension_error(sink, exc)
        return value

    def add_processor(self, processor):
        """Append a processor to the session pipeline."""
        self._validate_extensions((processor,), "processor", "process")
        with self._extension_lock:
            self._processors.append(processor)
        return processor

    def remove_processor(self, processor):
        """Remove a previously attached processor."""
        with self._extension_lock:
            self._processors.remove(processor)

    def add_sink(self, sink):
        """Append a sink to the session pipeline."""
        self._validate_extensions((sink,), "sink", "on_frame")
        with self._extension_lock:
            self._sinks.append(sink)
        return sink

    def remove_sink(self, sink):
        """Remove a previously attached sink."""
        with self._extension_lock:
            self._sinks.remove(sink)

    def get_receive_rate(self):
        """Delegate packet-rate reporting to the configured receiver."""
        return self.receiver.get_receive_rate()

    def _report_extension_error(self, extension: object, exc: Exception):
        self._logger.exception(
            "Optional MOVIN SDK extension %r failed; capture continues", extension
        )
        if self._on_extension_error is not None:
            try:
                self._on_extension_error(extension, exc)
            except Exception:
                self._logger.exception("MOVIN SDK extension error callback failed")

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False
