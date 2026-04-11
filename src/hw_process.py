"""
hw_process.py — hardware motor control process.

Runs a 1000 Hz real-time loop:
  1. SyncRead  — bulk-read position/current/velocity from all motors
  2. Parse feedback and update velocity estimates
  3. Read motor commands from the shared memory / UDP buffer
  4. Apply goals and SyncWrite to all motors
  5. Write feedback back to the shared buffer

Usage:
    python hw_process.py                  # defaults from config.py
    python hw_process.py --comm UDP       # force UDP mode
    python hw_process.py --port COM5      # override serial port
"""
import argparse
import struct
import sys
import threading
import time
from typing import List, Optional

from robot_arm.config import CommConfig, HardwareConfig
from robot_arm.comm import make_hw_comm

try:
    from dynamixel_sdk import (
        PortHandler, PacketHandler, GroupSyncWrite, GroupFastSyncRead
    )
except ImportError:
    # Graceful fallback for environments without the SDK installed
    raise SystemExit(
        "[ERROR] dynamixel_sdk not found.\n"
        "Install via: pip install dynamixel_sdk\n"
        "Or use the mock SDK for dry-run testing."
    )

from dynamixel_motors import (
    XM430W210, XH430W210, XC330T181, XC330T288,
    XM430W350, XH430W350, XM540W150, XM430, XC330Base
)

MOTOR_CLASS_MAP = {
    "XM430-W210": XM430W210,
    "XH430-W210": XH430W210,
    "XM430-W350": XM430W350,
    "XH430-W350": XH430W350,
    "XC330-T181": XC330T181,
    "XC330-T288": XC330T288,
    "XM540-W150": XM540W150,
    "XM430 (Base)": XM430,
    "XC330 (Base)": XC330Base,
}

ADDR_READ_START = 224
LEN_READ = 10
ADDR_WRITE_START = 235
LEN_WRITE = 10


# ── Motor manager ─────────────────────────────────────────────────────────────

class MotorManager:
    """
    Owns all motor objects and the Dynamixel sync groups.
    Separates hardware init from the control loop.
    """

    def __init__(self, cfg: HardwareConfig):
        self._cfg = cfg
        self._port = PortHandler(cfg.port)
        self._pkt = PacketHandler(2.0)
        self.motors: List = []

        if not self._port.openPort():
            raise RuntimeError(f"Cannot open port {cfg.port}")
        if not self._port.setBaudRate(cfg.baudrate):
            raise RuntimeError(f"Cannot set baudrate {cfg.baudrate}")

        self._sync_read = GroupFastSyncRead(
            self._port, self._pkt, ADDR_READ_START, LEN_READ
        )
        self._sync_write = GroupSyncWrite(
            self._port, self._pkt, ADDR_WRITE_START, LEN_WRITE
        )

        self._init_motors()

    def _init_motors(self) -> None:
        cfg = self._cfg
        print("\n[HW] Initializing motors...")
        for motor_id, model_str in cfg.actuators.items():
            cls = MOTOR_CLASS_MAP.get(model_str, XM430)
            try:
                m = cls(
                    motor_id, self._port, self._pkt,
                    limit_torque_nm=cfg.limit_torque_nm,
                    operating_mode=cfg.operating_mode,
                )
                m.configure_velocity_estimator(
                    mode=cfg.vel_est_mode,
                    window_size=cfg.vel_window_size,
                    ema_alpha=cfg.vel_ema_alpha,
                )
                self._sync_read.addParam(motor_id)
                self.motors.append(m)
                print(f"  OK  ID {motor_id}  ({model_str})")
            except Exception as exc:
                print(f"  FAIL ID {motor_id}  ({model_str}): {exc}")

        if not self.motors:
            raise RuntimeError("No motors initialized — check wiring and port.")

    def read_all(self, t_now: float) -> None:
        self._sync_read.txRxPacket()
        for m in self.motors:
            m.read_from_sync(self._sync_read)
            m.update_estimated_velocity(t_now)

    def write_all(self) -> None:
        self._sync_write.clearParam()
        for m in self.motors:
            self._sync_write.addParam(m.id, m.get_sync_write_data())
        self._sync_write.txPacket()

    def stop_all(self) -> None:
        """Send zero torque + hold current position to all motors."""
        self._sync_write.clearParam()
        for m in self.motors:
            m.updateGoalTorque(0.0)
            m.updateGoalPosition(0.0)
            self._sync_write.addParam(m.id, m.get_sync_write_data())
        try:
            self._sync_write.txPacket()
        except Exception:
            pass

    def close(self) -> None:
        self.stop_all()
        self._port.closePort()
        print("[HW] Port closed.")


# ── Feedback / command buffers ────────────────────────────────────────────────

class SharedBuffers:
    """
    Thread-safe command and feedback buffers.
    All arrays are indexed by motor ID (not list position).
    """

    def __init__(self, max_motors: int):
        self._lock = threading.Lock()
        self.cmd_pos_deg = [0.0] * max_motors
        self.cmd_tau_nm = [0.0] * max_motors
        self.cmd_vel = [0.0] * max_motors
        self.fb_pos_deg = [0.0] * max_motors
        self.fb_tau_nm = [0.0] * max_motors
        self.fb_vel_rad = [0.0] * max_motors

    def write_cmd(self, motor_id: int, pos: float, tau: float, vel: float) -> None:
        with self._lock:
            self.cmd_pos_deg[motor_id] = pos
            self.cmd_tau_nm[motor_id] = tau
            self.cmd_vel[motor_id] = vel

    def read_cmd(self, motor_id: int):
        with self._lock:
            return (
                self.cmd_pos_deg[motor_id],
                self.cmd_tau_nm[motor_id],
                self.cmd_vel[motor_id],
            )

    def write_fb(self, motor_id: int, pos: float, tau: float, vel: float) -> None:
        with self._lock:
            self.fb_pos_deg[motor_id] = pos
            self.fb_tau_nm[motor_id] = tau
            self.fb_vel_rad[motor_id] = vel

    def snapshot_fb(self):
        with self._lock:
            return (
                list(self.fb_pos_deg),
                list(self.fb_tau_nm),
                list(self.fb_vel_rad),
            )

    def snapshot_cmd(self):
        with self._lock:
            return (
                list(self.cmd_pos_deg),
                list(self.cmd_tau_nm),
                list(self.cmd_vel),
            )


# ── Control loop ──────────────────────────────────────────────────────────────

def control_loop(
    hw: MotorManager,
    buffers: SharedBuffers,
    comm,
    cfg: HardwareConfig,
    running_flag: threading.Event,
) -> None:
    period = 1.0 / cfg.target_hz
    stride = cfg.data_stride
    next_wake = time.perf_counter()
    loop_count = 0
    monitor_start = time.perf_counter()

    print(f"\n[HW] Control loop started at {cfg.target_hz:.0f} Hz")

    while running_flag.is_set():
        t_now = time.perf_counter()

        # 1. Bulk read from motors
        hw.read_all(t_now)

        # 2. Publish feedback
        for m in hw.motors:
            tau, pos, vel = m.report()
            buffers.write_fb(m.id, pos, tau, vel)

        # 3. Read external commands
        raw_cmd = comm.receive()
        if raw_cmd:
            for m in hw.motors:
                offset = m.id * stride
                if offset + stride <= len(raw_cmd):
                    p, t, v = struct.unpack_from("fff", raw_cmd, offset)
                    buffers.write_cmd(m.id, p, t, v)

        # 4. Apply goals and bulk write
        cmd_pos, cmd_tau, cmd_vel = buffers.snapshot_cmd()
        for m in hw.motors:
            mid = m.id
            m.updateGoalTorque(cmd_tau[mid])
            m.updateGoalPosition(cmd_pos[mid])
            m.updateGoalVelocity(cmd_vel[mid])

        hw.write_all()

        # 5. Send feedback back to comm (Unity or IK process)
        fb_pos, fb_tau, fb_vel = buffers.snapshot_fb()
        out = bytearray(cfg.max_motors * stride)
        for m in hw.motors:
            mid = m.id
            struct.pack_into("fff", out, mid * stride,
                             fb_pos[mid], fb_tau[mid], fb_vel[mid])
        comm.send(bytes(out))

        # 6. Periodic console status
        loop_count += 1
        if time.perf_counter() - monitor_start >= 1.0:
            elapsed = time.perf_counter() - monitor_start
            hz = loop_count / elapsed
            angles = "  ".join(
                f"M{m.id}:{fb_pos[m.id]:5.1f}°" for m in hw.motors
            )
            sys.stdout.write(f"\r[HW] {hz:6.1f} Hz  |  {angles}\033[K")
            sys.stdout.flush()
            loop_count = 0
            monitor_start = time.perf_counter()

        # 7. Busy-wait to maintain target rate
        next_wake += period
        now = time.perf_counter()
        if next_wake < now:
            next_wake = now  # catch up if we fell behind
        while time.perf_counter() < next_wake:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

def run(comm_cfg: CommConfig, hw_cfg: HardwareConfig) -> None:
    hw = MotorManager(hw_cfg)
    comm = make_hw_comm(comm_cfg, hw_cfg)
    buffers = SharedBuffers(hw_cfg.max_motors)
    running = threading.Event()
    running.set()

    thread = threading.Thread(
        target=control_loop,
        args=(hw, buffers, comm, hw_cfg, running),
        daemon=True,
    )
    thread.start()

    try:
        while running.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n[HW] Interrupt received — shutting down...")
        running.clear()
        thread.join(timeout=3.0)
    finally:
        hw.close()
        comm.close()


def _parse_args():
    p = argparse.ArgumentParser(description="Robot arm hardware process")
    p.add_argument("--comm", choices=["SHM", "UDP"], default=None)
    p.add_argument("--port", default=None, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    p.add_argument("--hz", type=float, default=None, help="Target loop rate")
    return p.parse_args()


if __name__ == "__main__":
    import platform

    args = _parse_args()

    comm_cfg = CommConfig()
    hw_cfg = HardwareConfig()

    # Default comm mode: SHM on Windows, UDP on others
    if args.comm:
        comm_cfg.mode = args.comm
    elif platform.system() != "Windows":
        comm_cfg.mode = "UDP"

    if args.port:
        hw_cfg.port = args.port
    if args.hz:
        hw_cfg.target_hz = args.hz

    run(comm_cfg, hw_cfg)
