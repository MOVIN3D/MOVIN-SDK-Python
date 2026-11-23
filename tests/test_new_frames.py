"""Lossless frame polling: get_new_frames() returns every queued frame, oldest first."""

import os
import socket
import tempfile
import time
import unittest
import uuid

from movin_sdk_python import MocapReceiver, MovinFrameAssembler, OscRecorder, ReplayMocapReceiver
from movin_sdk_python.mocap_receiver.osc_writer import encode_osc_message


def frame_args(frame_idx, actor="MOVINMan"):
    """One single-chunk, single-bone /MOVIN/Frame message."""
    return [
        "12:00:00", actor, frame_idx, 1, 0, 1, 1,
        0, -1, "Hips",
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0,
        1.0, 1.0, 1.0,
    ]


class AssemblerTests(unittest.TestCase):
    def test_pop_all_frames_is_oldest_first_and_clears(self):
        assembler = MovinFrameAssembler(max_ready_frames=4)
        for i in range(3):
            assembler.ingest("/MOVIN/Frame", frame_args(i), now=0.0)
        self.assertEqual([f["frame_idx"] for f in assembler.pop_all_frames()], [0, 1, 2])
        self.assertEqual(assembler.pop_all_frames(), [])
        self.assertIsNone(assembler.pop_latest_frame())


class ReceiverTests(unittest.TestCase):
    def test_live_receiver_returns_every_frame_and_counts_them(self):
        receiver = MocapReceiver(host="127.0.0.1", port=0)
        receiver.start()
        self.addCleanup(receiver.stop)
        target = receiver.sock.getsockname()[:2]
        frames = []
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            # A burst queued before the first poll arrives in one call, in order.
            for i in range(3):
                sender.sendto(encode_osc_message("/MOVIN/Frame", frame_args(i)), target)
            deadline = time.monotonic() + 2.0
            while len(frames) < 3 and time.monotonic() < deadline:
                frames += receiver.get_new_frames()
                time.sleep(0.005)
            # Then a steadily polled stream, one frame per poll.
            for i in range(3, 8):
                sender.sendto(encode_osc_message("/MOVIN/Frame", frame_args(i)), target)
                deadline = time.monotonic() + 1.0
                count = len(frames)
                while len(frames) == count and time.monotonic() < deadline:
                    frames += receiver.get_new_frames()
                    time.sleep(0.002)
        self.assertEqual([f["frame_idx"] for f in frames], list(range(8)))
        self.assertEqual(receiver.get_new_frames(), [])
        self.assertEqual(receiver.last_applied, 7)
        # The frames count toward the viewer rate MOVIN Studio is told about.
        reply = receiver.status.reply_args(uuid.uuid4().hex, "MOVINMan", None, time.monotonic())
        self.assertGreaterEqual(reply[8], 0.0)   # viewer age: a frame was consumed
        self.assertGreater(reply[5], 0.0)        # viewer fps

    def test_replay_receiver_has_the_same_call(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "rec.pkl")
            recorder = OscRecorder(path)
            for i in range(3):
                recorder.record("/MOVIN/Frame", frame_args(i), wall_time=float(i))
            recorder.save()
            receiver = ReplayMocapReceiver(path, realtime=False, loop=False)
            receiver.start()
            receiver.thread.join(timeout=5)
            frames = receiver.get_new_frames()
            receiver.stop()
        self.assertEqual([f["frame_idx"] for f in frames], [0, 1, 2])


if __name__ == "__main__":
    unittest.main()
