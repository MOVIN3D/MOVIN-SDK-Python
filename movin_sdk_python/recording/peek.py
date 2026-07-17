"""
peek_first_frame - Read the first complete frame from a recorded OSC file.

Loads a recording the same way OscPlayer does (respecting the version gate) and
feeds its messages through a MovinFrameAssembler until the first complete frame
is assembled.  Used to detect the skeleton preset from a replay recording before
building the scene, without spinning up a full ReplayMocapReceiver.
"""

from __future__ import annotations

from .osc_player import OscPlayer
from ..mocap_receiver.movin_frame_assembler import MovinFrameAssembler


def peek_first_frame(recording_path):
    """Return the first complete assembled frame from a recording, or None.

    Args:
        recording_path: Path to a pickle recording saved by OscRecorder.

    Returns:
        The first complete frame dict (same format as
        MocapReceiver.get_latest_frame()), or None if the recording contains
        no complete frame.
    """
    player = OscPlayer(recording_path)
    assembler = MovinFrameAssembler(max_ready_frames=4)
    for address, args in player.messages(realtime=False):
        assembler.ingest(address, args)
        frame = assembler.pop_latest_frame()
        if frame is not None:
            return frame
    return None
