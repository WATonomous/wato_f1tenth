# Copyright (c) 2026-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Pure-Python Linux /dev/i2c-* driver for the TI INA226 bus-voltage monitor.

No ROS imports and no third-party I2C packages required (uses stdlib fcntl.ioctl).
Used to read the Jetson's dedicated 4S LiPo pack voltage (14.0-16.8 V) from the
JST-XH 5-pin balance plug (Pin 1 = B-, Pin 5 = B4+) on Jetson I2C Bus 7.
"""

import fcntl
import os
import struct

# Linux i2c-dev ioctl constant
I2C_SLAVE = 0x0703

# INA226 register addresses
REG_CONFIG = 0x00
REG_BUS_VOLTAGE = 0x02
REG_MANUFACTURER_ID = 0xFE
REG_DIE_ID = 0xFF

# Expected TI INA226 identification values
EXPECTED_MANUFACTURER_ID = 0x5449  # "TI"
EXPECTED_DIE_ID_MASK = 0xFFF0
EXPECTED_DIE_ID = 0x2260

# Bus voltage register LSB: 1.25 mV/bit (unsigned 15-bit, full scale 40.96 V, rated to 36 V)
BUS_VOLTAGE_LSB_V = 0.00125

# Configuration: AVG=16 samples (010), VBUSCT=1.1 ms (100), VSHCT=1.1 ms (100),
# MODE=bus voltage continuous (110) -> 0x4526.
DEFAULT_CONFIG_WORD = 0x4526


def decode_bus_voltage(raw_bytes: bytes, voltage_scale: float = 1.0) -> float:
    """Decode INA226 16-bit big-endian Bus Voltage register (0x02) into volts."""
    if len(raw_bytes) != 2:
        raise ValueError(
            f"expected 2 bytes for INA226 bus voltage, got {len(raw_bytes)}"
        )
    raw = struct.unpack(">H", raw_bytes)[0]
    # Bit 15 is reserved (0 on INA226); mask defensively to 15 bits.
    return (raw & 0x7FFF) * BUS_VOLTAGE_LSB_V * voltage_scale


def is_valid_chip_id(manufacturer_id: int, die_id: int) -> bool:
    """Return True when the I2C device identifies as a TI INA226."""
    return (
        manufacturer_id == EXPECTED_MANUFACTURER_ID
        and (die_id & EXPECTED_DIE_ID_MASK) == EXPECTED_DIE_ID
    )


class INA226:
    """Minimal Linux i2c-dev reader for a TI INA226 bus voltage channel."""

    def __init__(
        self,
        bus: int = 7,
        address: int = 0x40,
        voltage_scale: float = 1.0,
        verify_chip_id: bool = True,
        config_word: int = DEFAULT_CONFIG_WORD,
    ):
        self.bus = bus
        self.address = address
        self.voltage_scale = voltage_scale
        self.verify_chip_id = verify_chip_id
        self.config_word = config_word
        self._fd = None

    @property
    def connected(self) -> bool:
        return self._fd is not None

    def open(self) -> None:
        """Open /dev/i2c-<bus>, bind the slave address, verify ID, and configure averaging."""
        self.close()
        dev_path = f"/dev/i2c-{self.bus}"
        fd = os.open(dev_path, os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, self.address)
            self._fd = fd
            if self.verify_chip_id:
                mfg = self._read_u16(REG_MANUFACTURER_ID)
                die = self._read_u16(REG_DIE_ID)
                if not is_valid_chip_id(mfg, die):
                    raise OSError(
                        f"unexpected I2C ID at {dev_path} 0x{self.address:02x}: "
                        f"mfg=0x{mfg:04x}, die=0x{die:04x} (expected TI INA226)"
                    )
            self._write_u16(REG_CONFIG, self.config_word)
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None

    def read_bus_voltage(self) -> float:
        """Read and return the scaled bus voltage in volts."""
        if self._fd is None:
            self.open()
        try:
            raw = self._read_reg_bytes(REG_BUS_VOLTAGE, 2)
        except OSError:
            self.close()
            raise
        return decode_bus_voltage(raw, self.voltage_scale)

    def _write_u16(self, reg: int, value: int) -> None:
        payload = struct.pack(">BH", reg, value & 0xFFFF)
        os.write(self._fd, payload)

    def _read_reg_bytes(self, reg: int, length: int) -> bytes:
        os.write(self._fd, bytes([reg & 0xFF]))
        data = os.read(self._fd, length)
        if len(data) != length:
            raise OSError(
                f"short I2C read from reg 0x{reg:02x}: {len(data)}/{length} bytes"
            )
        return data

    def _read_u16(self, reg: int) -> int:
        return struct.unpack(">H", self._read_reg_bytes(reg, 2))[0]

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
