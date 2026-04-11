"""
comm.py — UDP communication managers.

IKComm   : used by ik_process.py
             receive() → latest JSON bytes from Unity, or None
             send(data) → sends joint angle bytes back to Unity

HWComm   : used by hw_process.py
             receive() → latest motor-command bytes from IK process, or None
             send(data) → sends motor feedback bytes back

Both classes drain the socket on every receive() call so only the
most recent packet is ever processed — old packets are discarded.
This is intentional: IK and motor control are rate-driven, not
queue-driven.
"""
import select
import socket
from typing import Optional


# ── IK side ───────────────────────────────────────────────────────────────────

class IKComm:
    """
    Receives tracking JSON from Unity on ik_recv_port.
    Sends joint-angle bytes back to Unity on ik_send_port.

    The reply goes to whichever IP sent the last tracking packet,
    so this works whether Unity is on the same machine or the network.
    """

    def __init__(self, ip: str, recv_port: int, send_port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((ip, recv_port))
        self._sock.setblocking(False)
        self._send_port = send_port
        self._reply_addr: Optional[tuple] = None
        print(f"[IKComm] Listening on {ip}:{recv_port}  "
              f"| Replies -> :{send_port}")

    def receive(self) -> Optional[bytes]:
        """Drain socket, return only the most recent packet."""
        latest = None
        while select.select([self._sock], [], [], 0.0)[0]:
            data, addr = self._sock.recvfrom(4096)
            latest = data
            self._reply_addr = addr
        return latest

    def send(self, data: bytes) -> None:
        """Send joint-angle bytes to Unity."""
        if self._reply_addr is None:
            return
        self._sock.sendto(data, (self._reply_addr[0], self._send_port))

    def close(self) -> None:
        self._sock.close()


# ── Hardware side ─────────────────────────────────────────────────────────────

class HWComm:
    """
    Receives motor-command bytes from the IK process on hw_recv_port.
    Sends motor-feedback bytes back on hw_send_port.

    Packet format (both directions):
      32 slots x 12 bytes = 384 bytes total
      Each slot: float pos_deg, float tau_nm, float vel  (little-endian)
      Slot index = motor ID (IDs 2-6 are used; rest are zero-padded)
    """

    PACKET_SIZE = 384   # 32 motors x 3 floats x 4 bytes

    def __init__(self, ip: str, recv_port: int, send_port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((ip, recv_port))
        self._sock.setblocking(False)
        self._send_addr = (ip, send_port)
        self._latest: bytes = bytes(self.PACKET_SIZE)
        print(f"[HWComm]  Listening on {ip}:{recv_port}  "
              f"| Feedback -> :{send_port}")

    def receive(self) -> bytes:
        """Always returns the most recent command packet (zeros until first packet arrives)."""
        try:
            while True:
                data, _ = self._sock.recvfrom(self.PACKET_SIZE + 64)
                if len(data) == self.PACKET_SIZE:
                    self._latest = data
        except (BlockingIOError, OSError):
            pass
        return self._latest

    def send(self, data: bytes) -> None:
        try:
            self._sock.sendto(data, self._send_addr)
        except BlockingIOError:
            pass

    def close(self) -> None:
        self._sock.close()


# ── Factory helpers ───────────────────────────────────────────────────────────

def make_ik_comm(cfg) -> IKComm:
    """cfg is a CommConfig instance."""
    return IKComm(cfg.ip, cfg.ik_recv_port, cfg.ik_send_port)


def make_hw_comm(cfg) -> HWComm:
    """cfg is a CommConfig instance."""
    return HWComm(cfg.ip, cfg.hw_recv_port, cfg.hw_send_port)
