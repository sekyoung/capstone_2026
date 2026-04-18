"""
test_ik_no_source.py — verify the IK loop works standalone.

Sends fake tracking packets over UDP in a loop, sweeping the hand target
along a smooth sine-wave path.  Watch the arm move in the Meshcat window.

Usage:
    # Terminal 1 — start IK process
    python ik_process.py

    # Terminal 2 — run this script
    python test_ik_no_source.py

Options:
    --ip     127.0.0.1   target IP
    --port   10001        IK recv port
    --hz     30           packet rate
    --radius 0.10         sweep radius (metres)
"""
import argparse
import json
import math
import socket
import time


def sweep_packet(t: float, radius: float) -> dict:
    """
    Circular sweep in the XZ plane at a fixed Y height.
    Elbow hint is placed halfway between base and hand.
    """
    x = radius * math.cos(t * 2)
    y = 0.18                          # fixed height above base
    z = 0.12 + radius * math.sin(t * 2)

    return {
        "type": "tracking",
        "hand_pos":  [x, y, z],
        "hand_rot":  [1.0, 0.0, 0.0, 0.0],   # identity — no orientation constraint
        "elbow_pos": [x * 0.5, y * 0.5, z * 0.5],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ip",     default="127.0.0.1")
    p.add_argument("--port",   type=int, default=10001)
    p.add_argument("--hz",     type=float, default=30.0)
    p.add_argument("--radius", type=float, default=0.10)
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    t = 0.0

    print(f"Sending to {args.ip}:{args.port} at {args.hz:.0f} Hz  (Ctrl-C to stop)")
    print("Watch the Meshcat window — the arm should sweep in a circle.\n")

    try:
        while True:
            pkt = sweep_packet(t, args.radius)
            data = json.dumps(pkt).encode()
            sock.sendto(data, (args.ip, args.port))

            # Console readout
            hp = pkt["hand_pos"]
            print(f"\r  t={t:5.2f}  hand=({hp[0]:+.3f}, {hp[1]:+.3f}, {hp[2]:+.3f})", end="")

            time.sleep(period)
            t += period
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
