import pickle
import tempfile
import unittest
from pathlib import Path

from movin_sdk_python import MovinSession


class FakeReceiver:
    def __init__(self, frames=()):
        self.frames = list(frames)
        self.recorder = None
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def get_latest_frame(self):
        return self.frames.pop(0) if self.frames else None

    def get_receive_rate(self):
        return 60.0


class FailingStopReceiver(FakeReceiver):
    def stop(self):
        super().stop()
        raise RuntimeError("receiver stop failed")


class AddProcessor:
    def __init__(self, amount):
        self.amount = amount

    def process(self, frame):
        return {**frame, "value": frame["value"] + self.amount}


class CollectSink:
    def __init__(self):
        self.frames = []

    def on_frame(self, frame):
        self.frames.append(frame)


class FailingProcessor:
    def process(self, frame):
        raise RuntimeError("extension failed")


class MovinSessionTests(unittest.TestCase):
    def test_host_and_port_are_forwarded_to_default_receiver(self):
        session = MovinSession(host="127.0.0.1", port=12000)

        self.assertEqual(session.receiver.host, "127.0.0.1")
        self.assertEqual(session.receiver.port, 12000)

    def test_default_session_returns_raw_frame(self):
        raw = {"frame_idx": 1, "bones": []}
        session = MovinSession(receiver=FakeReceiver([raw]))

        self.assertIs(session.get_latest_frame(), raw)

    def test_processors_feed_sinks_without_mutating_raw_frame(self):
        raw = {"frame_idx": 1, "bones": [], "value": 3}
        sink = CollectSink()
        receiver = FakeReceiver([raw])
        session = MovinSession(
            receiver=receiver,
            processors=[AddProcessor(4)],
            sinks=[sink],
        )

        processed = session.get_latest_frame()

        self.assertEqual(processed["value"], 7)
        self.assertEqual(sink.frames, [processed])
        self.assertEqual(raw["value"], 3)

    def test_extension_failure_is_reported_and_capture_continues(self):
        raw = {"frame_idx": 1, "bones": []}
        errors = []
        receiver = FakeReceiver([raw, raw])
        session = MovinSession(
            receiver=receiver,
            processors=[FailingProcessor()],
            on_extension_error=lambda extension, exc: errors.append(exc),
        )

        with self.assertLogs("movin_sdk_python.session", level="ERROR"):
            self.assertIs(session.get_latest_frame(), raw)
            self.assertIs(session.get_latest_frame(), raw)

        self.assertEqual(len(errors), 2)
        self.assertTrue(all(isinstance(exc, RuntimeError) for exc in errors))

    def test_recording_is_attached_detached_and_saved(self):
        receiver = FakeReceiver()
        session = MovinSession(receiver=receiver)

        with tempfile.TemporaryDirectory() as tmp_dir:
            output_path = Path(tmp_dir) / "session.pkl"
            recorder = session.start_recording(str(output_path))
            self.assertIs(receiver.recorder, recorder)
            recorder.record("/MOVIN/Frame", ["timestamp"], wall_time=10.0)

            returned = session.stop_recording()

            self.assertIs(returned, recorder)
            self.assertIsNone(receiver.recorder)
            with output_path.open("rb") as stream:
                recording = pickle.load(stream)
            self.assertEqual(recording["num_messages"], 1)
            self.assertEqual(recording["stream_type"], "movin")

    def test_context_manager_starts_and_stops_receiver(self):
        receiver = FakeReceiver()

        with MovinSession(receiver=receiver) as session:
            self.assertTrue(session.is_running)
            self.assertTrue(receiver.started)

        self.assertFalse(session.is_running)
        self.assertTrue(receiver.stopped)

    def test_stop_saves_recording_when_receiver_cleanup_fails(self):
        receiver = FailingStopReceiver()
        session = MovinSession(receiver=receiver).start()

        with tempfile.TemporaryDirectory() as tmp_dir:
            output_path = Path(tmp_dir) / "session.pkl"
            session.start_recording(str(output_path)).record(
                "/MOVIN/Frame", ["timestamp"], wall_time=10.0
            )

            with self.assertRaisesRegex(RuntimeError, "receiver stop failed"):
                session.stop()

            self.assertTrue(output_path.exists())
            self.assertFalse(session.is_recording)
            self.assertFalse(session.is_running)

    def test_invalid_extension_is_rejected(self):
        with self.assertRaisesRegex(TypeError, "must define process"):
            MovinSession(receiver=FakeReceiver(), processors=[object()])


if __name__ == "__main__":
    unittest.main()
