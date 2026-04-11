"""
mock_dynamixel_sdk.py — drop-in stub for dry-run testing.

Place next to hw_process.py and import instead of dynamixel_sdk
by temporarily editing hw_process.py's import block, or by running:

    python hw_process.py --mock   (if you add --mock handling)

All methods succeed silently.  read4ByteTxRx returns a plausible centre
position (2048 = 0 deg in Dynamixel units) so initial calibration works.
"""

DXL_LOBYTE = lambda x: x & 0xFF
DXL_HIBYTE = lambda x: (x >> 8) & 0xFF
DXL_LOWORD = lambda x: x & 0xFFFF
DXL_HIWORD = lambda x: (x >> 16) & 0xFFFF


class PortHandler:
    def __init__(self, port: str):
        self.port = port

    def openPort(self) -> bool:
        print(f"[MOCK] PortHandler.openPort({self.port})")
        return True

    def setBaudRate(self, baud: int) -> bool:
        print(f"[MOCK] setBaudRate({baud})")
        return True

    def closePort(self) -> None:
        print("[MOCK] closePort()")


class PacketHandler:
    def __init__(self, version: float):
        self._version = version

    def ping(self, port, motor_id):
        return 0, 0, 0  # model_number, result, error

    def write1ByteTxRx(self, port, motor_id, addr, val):
        return 0, 0

    def write2ByteTxRx(self, port, motor_id, addr, val):
        return 0, 0

    def write4ByteTxRx(self, port, motor_id, addr, val):
        return 0, 0

    def read4ByteTxRx(self, port, motor_id, addr):
        # Return 2048 (centre position) so initialPosition is a realistic value
        return 2048, 0, 0

    def getTxRxResult(self, result: int) -> str:
        return "COMM_SUCCESS"

    def getRxPacketError(self, error: int) -> str:
        return ""


class GroupSyncWrite:
    def __init__(self, port, pkt, start_addr, data_len):
        pass

    def addParam(self, motor_id, data) -> bool:
        return True

    def txPacket(self) -> int:
        return 0

    def clearParam(self) -> None:
        pass


class GroupFastSyncRead:
    def __init__(self, port, pkt, start_addr, data_len):
        self._data: dict = {}

    def addParam(self, motor_id) -> bool:
        return True

    def txRxPacket(self) -> int:
        return 0

    def isAvailable(self, motor_id, addr, length) -> bool:
        return True

    def getData(self, motor_id, addr, length) -> int:
        # Return 2048 for position reads, 0 for current/velocity
        return 2048 if length == 4 else 0
