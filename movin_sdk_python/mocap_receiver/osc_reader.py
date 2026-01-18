"""
OscReader - Minimal OSC (Open Sound Control) message parser.

This module provides a lightweight OSC reader for parsing binary OSC messages
received over UDP. It's designed specifically for parsing mocap data from
MOVIN (via Unity).
"""

import struct


class OscReader:
    """
    Tiny OSC reader for parsing a single OSC message packet.
    
    OSC (Open Sound Control) is a protocol for communication among computers,
    sound synthesizers, and other multimedia devices. This reader handles
    the basic OSC message format with support for int32, float32, and string types.
    
    Example usage:
        data = sock.recvfrom(65535)[0]
        reader = OscReader(data)
        address, args = reader.read_message()
        # address = "/MOVIN/Frame"
        # args = [timestamp, actor_name, frame_idx, ...]
    """
    
    def __init__(self, data: bytes):
        """
        Initialize the OSC reader with raw packet data.
        
        Args:
            data: Raw bytes from UDP packet containing OSC message
        """
        self.data = data
        self.i = 0
        self.n = len(data)

    def _read_padded_string(self):
        """Read a null-terminated, 4-byte padded string."""
        start = self.i
        try:
            end = self.data.index(b'\x00', start)
        except ValueError:
            raise ValueError("OSC string not null-terminated")
        s = self.data[start:end].decode('utf-8', errors='replace')
        self.i = (end + 4) & ~0x03
        if self.i > self.n:
            raise ValueError("OSC string padding overflow")
        return s

    def _read_int32(self):
        """Read a big-endian 32-bit integer."""
        if self.i + 4 > self.n:
            raise ValueError("OSC int32 truncated")
        val = struct.unpack(">i", self.data[self.i:self.i+4])[0]
        self.i += 4
        return val

    def _read_float32(self):
        """Read a big-endian 32-bit float."""
        if self.i + 4 > self.n:
            raise ValueError("OSC float32 truncated")
        val = struct.unpack(">f", self.data[self.i:self.i+4])[0]
        self.i += 4
        return val

    def read_message(self):
        """
        Parse the OSC message and return address and arguments.
        
        Returns:
            Tuple of (address: str, args: list) where:
                - address is the OSC address pattern (e.g., "/MOVIN/Frame")
                - args is a list of parsed arguments (int, float, or string)
                
        Raises:
            ValueError: If the message is malformed
        """
        address = self._read_padded_string()
        if not address:
            raise ValueError("Empty OSC address")
        if self.i >= self.n:
            return address, []
        typetags = self._read_padded_string()
        if not typetags.startswith(','):
            raise ValueError("OSC typetags missing ',' prefix")
        argspec = typetags[1:]
        args = []
        for t in argspec:
            if t == 'i':
                args.append(self._read_int32())
            elif t == 'f':
                args.append(self._read_float32())
            elif t == 's':
                args.append(self._read_padded_string())
            else:
                raise ValueError(f"Unsupported OSC arg type: {t}")
        return address, args
