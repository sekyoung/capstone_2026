"""
comm.py — communication managers.

Both IKCommManager and HWCommManager share the same protocol:
  receive()  → raw bytes or None
  send(data) → None

Higher-level parsing lives in the IK / HW modules, not here.
"""
import mmap
import select
import socket
import struct
from abc import ABC, abstractmethod
from typing import Optional


# ── Shared interface ──────────────────────────────────────────────────────────

class CommBase(ABC):
    @abstractmethod
    def receive(self) -> Optional[bytes]:
        """Return latest raw bytes if available, else None."""

    @abstractmethod
    def send(self, data: bytes) -> None:
        """Transmit data."""

    def close(self) -> None:
        pass


# ── IK side ──────────────────────────────────────────────────────────────────

class IKUDPComm(CommBase):
    """UDP transport for IK process (recv from Unity, send joint angles back)."""

    def __init__(self, ip: str, recv_port: int, send_port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((ip, recv_port))
        self._sock.setblocking(False)
        self._send_addr = (ip, send_port)
        self._latest_src: Optional[tuple] = None

    def receive(self) -> Optional[bytes]:
        latest = None
        while select.select([self._sock], [], [], 0.0)[0]:
            data, addr = self._sock.recvfrom(2048)
            latest = data
            self._latest_src = addr
        return latest

    def send(self, data: bytes) -> None:
        if self._latest_src:
            self._sock.sendto(data, (self._latest_src[0], self._send_addr[1]))

    def close(self) -> None:
        self._sock.close()


class IKSHMComm(CommBase):
    """Shared-memory transport for IK process."""

    # Packet layout: 11 floats = 44 bytes
    # [hand_x, hand_y, hand_z, qw, qx, qy, qz, elbow_x, elbow_y, elbow_z, scale]
    RECV_FORMAT = "f" * 11
    RECV_SIZE = struct.calcsize(RECV_FORMAT)

    def __init__(self, target_name: str, reply_name: str,
                 motor_cmd_name: str, target_size: int, reply_size: int,
                 motor_size: int, motor_ids: list,
                 default_torque: float, default_velocity: float):
        self._shm_target = mmap.mmap(-1, target_size, tagname=target_name)
        self._shm_reply = mmap.mmap(-1, reply_size, tagname=reply_name)
        self._shm_motor_cmd = mmap.mmap(-1, motor_size, tagname=motor_cmd_name)
        self._motor_ids = motor_ids
        self._default_torque = default_torque
        self._default_velocity = default_velocity
        self._stride = 12  # 3 × float32

    def receive(self) -> Optional[bytes]:
        self._shm_target.seek(0)
        data = self._shm_target.read(self.RECV_SIZE)
        return data if len(data) == self.RECV_SIZE else None

    def send(self, data: bytes) -> None:
        """data is raw float bytes of joint angles in radians."""
        # 1. Unity virtual robot (radians)
        self._shm_reply.seek(0)
        self._shm_reply.write(data)

        # 2. Hardware motor command buffer (degrees)
        n = len(data) // 4
        angles_rad = struct.unpack(f"{n}f", data)
        import math
        angles_deg = [math.degrees(a) for a in angles_rad]

        buf = bytearray(self._shm_motor_cmd.size())
        for i, mid in enumerate(self._motor_ids):
            if i >= len(angles_deg):
                break
            struct.pack_into("fff", buf, mid * self._stride,
                             angles_deg[i], self._default_torque, self._default_velocity)

        self._shm_motor_cmd.seek(0)
        self._shm_motor_cmd.write(buf)

    def close(self) -> None:
        for shm in (self._shm_target, self._shm_reply, self._shm_motor_cmd):
            try:
                shm.close()
            except Exception:
                pass


# ── Hardware side ─────────────────────────────────────────────────────────────

class HWSHMComm(CommBase):
    """Shared-memory transport for hardware process."""

    def __init__(self, cmd_name: str, status_name: str, mem_size: int):
        self._shm_cmd = mmap.mmap(0, mem_size, cmd_name)
        self._shm_status = mmap.mmap(0, mem_size, status_name)
        self._mem_size = mem_size

    def receive(self) -> Optional[bytes]:
        self._shm_cmd.seek(0)
        return self._shm_cmd.read(self._mem_size)

    def send(self, data: bytes) -> None:
        self._shm_status.seek(0)
        self._shm_status.write(data)

    def close(self) -> None:
        for shm in (self._shm_cmd, self._shm_status):
            try:
                shm.close()
            except Exception:
                pass


class HWUDPComm(CommBase):
    """UDP transport for hardware process."""

    def __init__(self, ip: str, recv_port: int, send_port: int, mem_size: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((ip, recv_port))
        self._sock.setblocking(False)
        self._send_addr = (ip, send_port)
        self._mem_size = mem_size
        self._latest: bytes = bytes(mem_size)

    def receive(self) -> Optional[bytes]:
        try:
            while True:
                data, _ = self._sock.recvfrom(2048)
                if len(data) == self._mem_size:
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

def make_ik_comm(cfg_comm, cfg_ik) -> CommBase:
    if cfg_comm.mode == "SHM":
        return IKSHMComm(
            target_name=cfg_comm.shm_target_name,
            reply_name=cfg_comm.shm_ik_reply_name,
            motor_cmd_name=cfg_comm.shm_motor_cmd_name,
            target_size=cfg_comm.shm_target_size,
            reply_size=cfg_comm.shm_reply_size,
            motor_size=cfg_comm.shm_motor_size,
            motor_ids=cfg_ik.motor_ids,
            default_torque=cfg_ik.default_target_torque,
            default_velocity=cfg_ik.default_target_velocity,
        )
    return IKUDPComm(cfg_comm.ip, cfg_comm.ik_recv_port, cfg_comm.ik_send_port)


def make_hw_comm(cfg_comm, cfg_hw) -> CommBase:
    if cfg_comm.mode == "SHM":
        return HWSHMComm(
            cfg_comm.shm_motor_cmd_name,
            cfg_comm.shm_motor_status_name,
            cfg_comm.shm_motor_size,
        )
    return HWUDPComm(
        cfg_comm.ip,
        cfg_hw_recv_port := cfg_comm.hw_recv_port,
        cfg_comm.hw_send_port,
        cfg_comm.shm_motor_size,
    )
