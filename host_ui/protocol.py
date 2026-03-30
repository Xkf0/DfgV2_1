from __future__ import annotations

from dataclasses import dataclass


class ProtocolError(ValueError):
    pass


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return data + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def verify_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    expected = crc16_modbus(frame[:-2])
    actual = frame[-2] | (frame[-1] << 8)
    return expected == actual


@dataclass(frozen=True)
class WriteRegisterRequest:
    device_addr: int
    register: int
    value: int

    def encode(self) -> bytes:
        payload = bytes(
            (
                self.device_addr & 0xFF,
                0x06,
                (self.register >> 8) & 0xFF,
                self.register & 0xFF,
                (self.value >> 8) & 0xFF,
                self.value & 0xFF,
            )
        )
        return append_crc(payload)


@dataclass(frozen=True)
class WriteRegisterResponse:
    device_addr: int
    function_code: int
    register: int
    value: int


def encode_write_register(device_addr: int, register: int, value: int) -> bytes:
    return WriteRegisterRequest(device_addr=device_addr, register=register, value=value).encode()


def decode_write_register_response(frame: bytes) -> WriteRegisterResponse:
    if len(frame) != 8:
        raise ProtocolError(f"invalid response length: {len(frame)}")
    if not verify_crc(frame):
        raise ProtocolError("crc mismatch")
    if frame[1] != 0x06:
        raise ProtocolError(f"unexpected function code: 0x{frame[1]:02X}")
    return WriteRegisterResponse(
        device_addr=frame[0],
        function_code=frame[1],
        register=(frame[2] << 8) | frame[3],
        value=(frame[4] << 8) | frame[5],
    )


def decode_write_register_request(frame: bytes) -> WriteRegisterRequest:
    if len(frame) != 8:
        raise ProtocolError(f"invalid request length: {len(frame)}")
    if not verify_crc(frame):
        raise ProtocolError("crc mismatch")
    if frame[1] != 0x06:
        raise ProtocolError(f"unexpected function code: 0x{frame[1]:02X}")
    return WriteRegisterRequest(
        device_addr=frame[0],
        register=(frame[2] << 8) | frame[3],
        value=(frame[4] << 8) | frame[5],
    )

