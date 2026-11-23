"""MOVIN Studio OSC status probe: request parsing, reply contents, and the UDP round trip.

The reply must satisfy MOVIN Studio's validation of status replies:
type tags ``sisffffffi``, the request's token echoed, version 1, fps >= 0,
ages >= -1, a character name of at most 256 characters, and same_source 0/1.
"""

import math
import socket
import time
import unittest
import uuid

from movin_sdk_python import MocapReceiver, OscReader, OscRecorder
from movin_sdk_python.mocap_receiver.osc_writer import encode_osc_message
from movin_sdk_python.mocap_receiver.stream_status import (
    OSC_STATUS_REPLY,
    OSC_STATUS_REQUEST,
    RateMeter,
    StreamStatus,
    parse_status_request,
)


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


def typetags(packet):
    """The type tag string of an encoded OSC message."""
    address_end = packet.index(b"\0")
    start = (address_end + 4) & ~3
    end = packet.index(b"\0", start)
    return packet[start + 1:end].decode()


def assert_studio_accepts(test, packet, token):
    """Mirror MOVIN Studio's receive_osc_status validation."""
    test.assertEqual(typetags(packet), "sisffffffi")
    address, values = OscReader(packet).read_message()
    test.assertEqual(address, OSC_STATUS_REPLY)
    test.assertEqual(values[0], token)
    test.assertEqual(values[1], 1)
    test.assertLessEqual(len(values[2]), 256)
    for value in values[3:6]:
        test.assertTrue(math.isfinite(value) and value >= 0)
    for value in values[6:9]:
        test.assertTrue(math.isfinite(value) and value >= -1)
    test.assertIn(values[9], (0, 1))
    return values


class OscWriterTests(unittest.TestCase):
    def test_round_trips_through_the_reader(self):
        packet = encode_osc_message("/a/b", ["text", 7, 1.5])
        self.assertEqual(len(packet) % 4, 0)
        self.assertEqual(OscReader(packet).read_message(), ("/a/b", ["text", 7, 1.5]))

    def test_explicit_type_tags_force_floats(self):
        packet = encode_osc_message("/x", [0, 0], "fi")
        self.assertEqual(typetags(packet), "fi")
        self.assertEqual(OscReader(packet).read_message()[1], [0.0, 0])

    def test_mismatched_tags_are_rejected(self):
        with self.assertRaises(ValueError):
            encode_osc_message("/x", [1, 2], "i")
        with self.assertRaises(ValueError):
            encode_osc_message("/x", [True])


class RequestParsingTests(unittest.TestCase):
    def test_accepts_studio_requests(self):
        token = uuid.uuid4().hex
        self.assertEqual(
            parse_status_request([token, 39581, "MOVINMan"]), (token, 39581, "MOVINMan")
        )

    def test_rejects_malformed_requests(self):
        token = uuid.uuid4().hex
        for args in (
            [],
            [token, 39581],
            [token, "39581", "MOVINMan"],
            ["not-a-token", 39581, "MOVINMan"],
            [token[:-1] + "g", 39581, "MOVINMan"],
            [token, 0, "MOVINMan"],
            [token, 70000, "MOVINMan"],
        ):
            self.assertIsNone(parse_status_request(args), args)


class RateMeterTests(unittest.TestCase):
    def test_rate_and_age(self):
        meter = RateMeter(window_sec=1.0)
        self.assertEqual(meter.rate(0.0), 0.0)
        self.assertEqual(meter.age(0.0), -1.0)
        for i in range(61):
            meter.tick(10.0 + i / 60.0)
        self.assertAlmostEqual(meter.rate(11.0), 60.0, places=6)
        self.assertAlmostEqual(meter.age(11.5), 0.5, places=6)
        # Outside the window the rate drops to zero; the age keeps growing.
        self.assertEqual(meter.rate(13.0), 0.0)
        self.assertAlmostEqual(meter.age(13.0), 2.0, places=6)


class StreamStatusTests(unittest.TestCase):
    def setUp(self):
        self.studio = ("127.0.0.1", 40000)
        self.status = StreamStatus()
        self.token = uuid.uuid4().hex

    def test_nothing_received_yet(self):
        values = self.status.reply_args(self.token, "MOVINMan", self.studio, now=5.0)
        self.assertEqual(values, [self.token, 1, "", 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, 0])

    def test_reports_the_requested_character_from_the_same_source(self):
        for i in range(31):
            self.status.on_frame("MOVINMan", self.studio, now=1.0 + i / 60.0)
            self.status.on_consumed(now=1.0 + i / 60.0)
        values = self.status.reply_args(self.token, "MOVINMan", self.studio, now=1.6)
        self.assertEqual(values[2], "MOVINMan")
        self.assertAlmostEqual(values[3], 60.0, places=6)
        self.assertAlmostEqual(values[5], 60.0, places=6)
        self.assertAlmostEqual(values[6], 0.1, places=6)
        self.assertAlmostEqual(values[8], 0.1, places=6)
        self.assertEqual(values[9], 1)

    def test_other_sources_and_characters(self):
        self.status.on_frame("ActorA", ("10.0.0.5", 5000), now=1.0)
        self.status.on_frame("ActorB", self.studio, now=1.0)
        # Requested character is received, but from another sender.
        self.assertEqual(self.status.reply_args(self.token, "ActorA", self.studio, 1.0)[9], 0)
        # Unknown character: report the most recently received actor instead.
        values = self.status.reply_args(self.token, "Nobody", self.studio, 1.0)
        self.assertEqual(values[2], "ActorB")
        self.assertEqual(values[9], 1)

    def test_reply_packet_passes_studio_validation(self):
        self.status.on_frame("x" * 300, self.studio, now=1.0)
        assert_studio_accepts(self, self.status.reply_packet(self.token, "", self.studio, 1.0), self.token)


class ReceiverStatusRoundTripTests(unittest.TestCase):
    """A fake MOVIN Studio streams frames and probes the receiver."""

    def setUp(self):
        self.studio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.studio.bind(("127.0.0.1", 0))
        self.studio.settimeout(1.0)
        self.reply_port = self.studio.getsockname()[1]

    def tearDown(self):
        self.studio.close()

    def start_receiver(self, **kwargs):
        receiver = MocapReceiver(host="127.0.0.1", port=0, **kwargs)
        receiver.start()
        self.addCleanup(receiver.stop)
        return receiver, receiver.sock.getsockname()[:2]

    def stream_frames(self, receiver, target, count, sender=None):
        sender = sender or self.studio
        for i in range(count):
            sender.sendto(encode_osc_message("/MOVIN/Frame", frame_args(i), None), target)
            deadline = time.monotonic() + 1.0
            while receiver.get_latest_frame() is None and time.monotonic() < deadline:
                time.sleep(0.001)
            time.sleep(1 / 120)

    def probe(self, target, character="MOVINMan", sender=None):
        token = uuid.uuid4().hex
        request = encode_osc_message(OSC_STATUS_REQUEST, [token, self.reply_port, character])
        (sender or self.studio).sendto(request, target)
        return token

    def test_studio_probe_is_answered_on_its_reply_port(self):
        receiver, target = self.start_receiver()
        self.stream_frames(receiver, target, 20)
        token = self.probe(target)
        packet, source = self.studio.recvfrom(65535)
        self.assertEqual(source, target)  # replies come from the receiver's socket
        values = assert_studio_accepts(self, packet, token)
        self.assertEqual(values[2], "MOVINMan")
        self.assertGreater(values[3], 10.0)
        self.assertGreater(values[5], 10.0)
        self.assertLess(values[6], 1.0)
        self.assertEqual(values[9], 1)

    def test_probe_from_another_socket_is_not_the_same_source(self):
        receiver, target = self.start_receiver()
        self.stream_frames(receiver, target, 3)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
            token = self.probe(target, sender=other)
            values = assert_studio_accepts(self, self.studio.recvfrom(65535)[0], token)
        self.assertEqual(values[9], 0)

    def test_malformed_probes_are_ignored(self):
        _, target = self.start_receiver()
        self.studio.sendto(
            encode_osc_message(OSC_STATUS_REQUEST, ["bad", self.reply_port, "MOVINMan"]), target
        )
        self.studio.settimeout(0.3)
        with self.assertRaises(socket.timeout):
            self.studio.recvfrom(65535)

    def test_replies_can_be_disabled(self):
        _, target = self.start_receiver(reply_status=False)
        self.probe(target)
        self.studio.settimeout(0.3)
        with self.assertRaises(socket.timeout):
            self.studio.recvfrom(65535)

    def test_probes_are_still_recorded(self):
        recorder = OscRecorder("/dev/null")
        _, target = self.start_receiver(recorder=recorder)
        token = self.probe(target)
        self.studio.recvfrom(65535)
        self.assertEqual(
            [(m["addr"], m["args"]) for m in recorder._messages],
            [(OSC_STATUS_REQUEST, [token, self.reply_port, "MOVINMan"])],
        )


if __name__ == "__main__":
    unittest.main()
