"""
encode_osc_message - Minimal OSC (Open Sound Control) message encoder.

The counterpart of OscReader: builds one OSC message packet with int32,
float32, and string arguments.  Used for the replies the receiver sends back
to MOVIN Studio.
"""

import struct


def _padded_string(value: str) -> bytes:
    """Encode a null-terminated string padded to a multiple of 4 bytes."""
    encoded = value.encode("utf-8") + b"\0"
    return encoded + b"\0" * (-len(encoded) % 4)


def encode_osc_message(address: str, args=(), typetags: str = None) -> bytes:
    """
    Encode one OSC message.

    Args:
        address: OSC address pattern, e.g. "/MOVIN/OSC/Status".
        args: Argument values (str, int, or float).
        typetags: Optional explicit type tags without the leading ',' (e.g.
            "sisffffffi").  Inferred from the Python types when omitted; pass
            it when a peer validates exact types, since a float argument that
            happens to be an int (0 instead of 0.0) would otherwise encode as 'i'.

    Returns:
        The encoded packet.

    Raises:
        ValueError: If the type tags and arguments do not match.
    """
    args = list(args)
    if typetags is None:
        tags = []
        for value in args:
            if isinstance(value, str):
                tags.append("s")
            elif isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"Unsupported OSC argument: {value!r}")
            elif isinstance(value, int):
                tags.append("i")
            else:
                tags.append("f")
        typetags = "".join(tags)
    if len(typetags) != len(args):
        raise ValueError(
            f"OSC type tags {typetags!r} do not match {len(args)} arguments"
        )

    payload = []
    for tag, value in zip(typetags, args):
        if tag == "s":
            payload.append(_padded_string(str(value)))
        elif tag == "i":
            payload.append(struct.pack(">i", int(value)))
        elif tag == "f":
            payload.append(struct.pack(">f", float(value)))
        else:
            raise ValueError(f"Unsupported OSC type tag: {tag!r}")
    return _padded_string(address) + _padded_string("," + typetags) + b"".join(payload)
