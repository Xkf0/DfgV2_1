from __future__ import annotations

import time
from dataclasses import dataclass

from host_ui.protocol import decode_write_register_response, encode_write_register
from host_ui.transport import MockTransport, SerialTransport, Transport


@dataclass(frozen=True)
class ActuatorSettings:
    driver_addr: int = 0x01
    pulses_per_mm: float = 200.0
    move_range_min: float = 1.0
    move_range_max: float = 210.0
    baud_rate: int = 9600


class LinearActuatorClient:
    REG_ABS_POS_H = 0x0024
    REG_ABS_POS_L = 0x0025
    REG_RUN_CMD = 0x0027
    REG_HOMING_CMD = 0x0030
    REG_ESTOP_CMD = 0x0028

    INIT_COMMANDS = (
        (0x0011, 0x0008),
        (0x0026, 0x0001),
        (0x0020, 0x0064),
        (0x0021, 0x0064),
        (0x0022, 0x0064),
        (0x0023, 0x0320),
        (0x0010, 0x0006),
        (0x0031, 0x0001),
        (0x0032, 0x0320),
        (0x0033, 0x001E),
        (0x0034, 0x0064),
    )

    def __init__(self, transport: Transport, settings: ActuatorSettings | None = None) -> None:
        self.transport = transport
        self.settings = settings or ActuatorSettings()
        self.last_position_mm: float | None = None

    @classmethod
    def from_serial(
        cls,
        port: str,
        baud_rate: int = 9600,
        settings: ActuatorSettings | None = None,
    ) -> "LinearActuatorClient":
        s = settings or ActuatorSettings(baud_rate=baud_rate)
        transport = SerialTransport(port=port, baudrate=baud_rate)
        return cls(transport=transport, settings=s)

    @classmethod
    def mock(cls, settings: ActuatorSettings | None = None) -> "LinearActuatorClient":
        return cls(transport=MockTransport(), settings=settings)

    def connect(self) -> None:
        self.transport.open()

    def disconnect(self) -> None:
        self.transport.close()

    def is_connected(self) -> bool:
        return self.transport.is_open()

    def write_register(self, reg: int, value: int) -> None:
        request = encode_write_register(self.settings.driver_addr, reg, value)
        response = self.transport.transact(request, response_size=8)
        decoded = decode_write_register_response(response)
        if decoded.register != reg or decoded.value != (value & 0xFFFF):
            raise RuntimeError(
                f"register echo mismatch: wrote reg=0x{reg:04X}, value=0x{value:04X}, "
                f"got reg=0x{decoded.register:04X}, value=0x{decoded.value:04X}"
            )

    def init_driver(self, inter_command_delay_s: float = 0.01) -> None:
        self.connect()
        for reg, value in self.INIT_COMMANDS:
            self.write_register(reg, value)
            time.sleep(inter_command_delay_s)

    def home(self) -> None:
        self.connect()
        self.write_register(self.REG_HOMING_CMD, 1)
        self.last_position_mm = 0.0

    def estop(self) -> None:
        self.connect()
        self.write_register(self.REG_ESTOP_CMD, 0x0001)

    def move_to(self, distance_mm: float) -> int:
        self.connect()
        if not (self.settings.move_range_min <= distance_mm <= self.settings.move_range_max):
            raise ValueError(
                f"distance out of range: {distance_mm} (allowed {self.settings.move_range_min}~{self.settings.move_range_max} mm)"
            )
        pulses = int(round(distance_mm * self.settings.pulses_per_mm))
        self.write_register(self.REG_ABS_POS_H, (pulses >> 16) & 0xFFFF)
        self.write_register(self.REG_ABS_POS_L, pulses & 0xFFFF)
        self.write_register(self.REG_RUN_CMD, 5)
        self.last_position_mm = distance_mm
        return pulses

