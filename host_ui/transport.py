from __future__ import annotations

import time
from abc import ABC, abstractmethod
from threading import Lock
from typing import Optional

from host_ui.protocol import (
    ProtocolError,
    append_crc,
    decode_write_register_request,
)

try:
    import serial
except Exception:  # pragma: no cover - environment optional
    serial = None


class Transport(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def is_open(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def transact(self, request: bytes, response_size: int = 8, timeout: Optional[float] = None) -> bytes:
        raise NotImplementedError


class SerialTransport(Transport):
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 0.2,
        bytesize: int = 8,
        parity: str = "N",
        stopbits: int = 1,
    ) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._bytesize = bytesize
        self._parity = parity
        self._stopbits = stopbits
        self._ser = None
        self._lock = Lock()

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=self._bytesize,
            parity=self._parity,
            stopbits=self._stopbits,
            timeout=self._timeout,
            xonxoff=False,
            rtscts=False,
        )

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def transact(self, request: bytes, response_size: int = 8, timeout: Optional[float] = None) -> bytes:
        self.open()
        with self._lock:
            if timeout is not None and self._ser is not None:
                self._ser.timeout = timeout
            self._ser.reset_input_buffer()
            self._ser.write(request)
            self._ser.flush()
            time.sleep(0.015)
            response = self._ser.read(response_size)
        if len(response) != response_size:
            raise TimeoutError(f"serial response too short: {len(response)}/{response_size}")
        return response


class MockTransport(Transport):
    """Mock transport that echoes valid write-register frames and stores register state."""

    def __init__(self) -> None:
        self._opened = False
        self.registers: dict[int, int] = {}
        self.request_log: list[bytes] = []

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def is_open(self) -> bool:
        return self._opened

    def transact(self, request: bytes, response_size: int = 8, timeout: Optional[float] = None) -> bytes:
        _ = timeout
        self.open()
        self.request_log.append(request)
        req = decode_write_register_request(request)
        self.registers[req.register] = req.value
        response = append_crc(
            bytes(
                (
                    req.device_addr & 0xFF,
                    0x06,
                    (req.register >> 8) & 0xFF,
                    req.register & 0xFF,
                    (req.value >> 8) & 0xFF,
                    req.value & 0xFF,
                )
            )
        )
        if response_size != len(response):
            raise ProtocolError(f"mock response size mismatch: asked {response_size}, actual {len(response)}")
        return response

