"""
MocapReceiver - Real-time motion capture data receiver via OSC protocol.

This package provides tools for receiving motion capture data from MOVIN
(via Unity) over UDP using the OSC protocol.

Example usage:
    from mocap_receiver import MocapReceiver
    
    # Initialize receiver
    receiver = MocapReceiver(port=11235)
    
    # Start receiving
    receiver.start()
    
    # Main loop
    while running:
        frame = receiver.get_latest_frame()
        if frame:
            print(f"Frame {frame['frame_idx']}: {len(frame['bones'])} bones")
            for bone in frame['bones']:
                print(f"  {bone['bone_name']}: pos={bone['p']}, rot={bone['q']}")
    
    # Cleanup
    receiver.stop()

Frame format:
    {
        "timestamp": str,        # Timestamp from mocap system
        "actor": str,            # Actor name
        "frame_idx": int,        # Frame index
        "bones": [               # List of bone data
            {
                "bone_index": int,
                "parent_index": int,
                "bone_name": str,
                "p": (px, py, pz),           # Local position
                "rq": (w, x, y, z),          # Rest pose quaternion
                "q": (w, x, y, z),           # Local rotation quaternion
                "s": (sx, sy, sz),           # Scale
            },
            ...
        ]
    }
"""

from .mocap_receiver import MocapReceiver
from .osc_reader import OscReader

__all__ = ["MocapReceiver", "OscReader"]
