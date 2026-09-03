import socket
import struct
import time
import unittest

from movin_sdk_python import MocapReceiver


def _osc_string(value):
    encoded = value.encode("utf-8") + b"\0"
    return encoded + (b"\0" * (-len(encoded) % 4))


def _osc_message(address, args):
    tags = []
    payload = []
    for value in args:
        if isinstance(value, str):
            tags.append("s")
            payload.append(_osc_string(value))
        elif isinstance(value, int):
            tags.append("i")
            payload.append(struct.pack(">i", value))
        else:
            tags.append("f")
            payload.append(struct.pack(">f", value))
    return (
        _osc_string(address)
        + _osc_string("," + "".join(tags))
        + b"".join(payload)
    )


class MocapReceiverBindingTests(unittest.TestCase):
    def test_default_bind_address_is_unchanged(self):
        receiver = MocapReceiver()

        self.assertEqual(receiver.host, "0.0.0.0")
        self.assertEqual(receiver.port, 11235)

    def test_receiver_binds_to_configured_local_address(self):
        receiver = MocapReceiver(host="127.0.0.1", port=0)

        try:
            receiver.start()
            bound_host, bound_port = receiver.sock.getsockname()[:2]

            self.assertEqual(bound_host, "127.0.0.1")
            self.assertGreater(bound_port, 0)

            frame_args = [
                "12:00:00",
                "MOVINMan",
                42,
                1,
                0,
                1,
                1,
                0,
                -1,
                "Hips",
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                1.0,
                1.0,
            ]
            packet = _osc_message("/MOVIN/Frame", frame_args)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(packet, (bound_host, bound_port))

            deadline = time.monotonic() + 1.0
            frame = None
            while frame is None and time.monotonic() < deadline:
                frame = receiver.get_latest_frame()
                time.sleep(0.001)

            self.assertIsNotNone(frame)
            self.assertEqual(frame["frame_idx"], 42)
            self.assertEqual(frame["actor"], "MOVINMan")
            self.assertEqual(frame["bones"][0]["bone_name"], "Hips")
        finally:
            receiver.stop()

    def test_invalid_bind_address_fails_during_start(self):
        receiver = MocapReceiver(host="256.256.256.256", port=11235)

        with self.assertRaises(OSError):
            receiver.start()

        self.assertFalse(receiver.running)
        self.assertIsNone(receiver.sock)
        self.assertIsNone(receiver.thread)


if __name__ == "__main__":
    unittest.main()
