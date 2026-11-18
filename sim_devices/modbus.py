# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Modbus RTU complete-frame endpoint, implementing FC03 only."""


def crc16(data):
    """CRC-16/Modbus: polynomial 0xa001, initial value 0xffff."""
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def _adu(payload):
    crc = crc16(payload)
    payload.append(crc & 255)
    payload.append(crc >> 8)
    return bytes(payload)


class ModbusRTUSlave:
    """One complete ADU per respond(); no baud/timing/fragment simulation.

    read_holding(start, count) must return exactly count uint16 registers.
    IndexError maps to illegal address; other callback errors are programming
    errors and propagate. fault(start, count) is normal, timeout or crc.
    """
    def __init__(self, unit_id, read_holding, fault=None):
        if type(unit_id) is not int or not 1 <= unit_id <= 247:
            raise ValueError("unit_id must be 1..247")
        if not callable(read_holding) or (fault is not None and not callable(fault)):
            raise TypeError("callbacks must be callable")
        self.unit_id = unit_id
        self.read_holding = read_holding
        self.fault = fault
        self.requests = self.responses = self.crc_errors = self.exceptions = 0
        self.last_request = None

    def _exception(self, function, code):
        self.exceptions += 1
        self.responses += 1
        return _adu(bytearray((self.unit_id, function | 0x80, code)))

    def respond(self, frame):
        # Inspect length before any copy; do not retain caller buffers/history.
        frame = memoryview(frame)
        self.requests += 1
        self.last_request = {"length": len(frame), "unit": None,
                             "function": None, "start": None, "count": None}
        if not 4 <= len(frame) <= 256:
            return None
        unit, function = frame[0], frame[1]
        self.last_request["unit"] = unit
        self.last_request["function"] = function
        if crc16(frame[:-2]) != frame[-2] | (frame[-1] << 8):
            self.crc_errors += 1
            return None
        if unit != self.unit_id:
            return None  # Includes broadcast reads.
        if function != 3:
            return self._exception(function, 1)
        if len(frame) != 8:
            return self._exception(function, 3)
        start = (frame[2] << 8) | frame[3]
        count = (frame[4] << 8) | frame[5]
        self.last_request["start"] = start
        self.last_request["count"] = count
        if not 1 <= count <= 125:
            return self._exception(function, 3)
        if start + count > 65536:
            return self._exception(function, 2)
        fault = "normal" if self.fault is None else self.fault(start, count)
        if fault not in ("normal", "timeout", "crc"):
            raise ValueError("unknown response fault")
        if fault == "timeout":
            return None
        try:
            words = self.read_holding(start, count)
        except IndexError:
            return self._exception(function, 2)
        if len(words) != count:
            raise ValueError("read_holding must return exactly count registers")
        response = bytearray((unit, function, count * 2))
        for word in words:
            if type(word) is not int or not 0 <= word <= 65535:
                raise ValueError("registers must be uint16 integers")
            response.append(word >> 8)
            response.append(word & 255)
        response = _adu(response)
        if fault == "crc":
            # Flip a CRC bit, never alter decoded register data.
            response = response[:-1] + bytes((response[-1] ^ 1,))
            self.crc_errors += 1
        self.responses += 1
        return response

    def snapshot(self):
        return {"unit_id": self.unit_id, "requests": self.requests,
                "responses": self.responses, "crc_errors": self.crc_errors,
                "exceptions": self.exceptions,
                "last_request": None if self.last_request is None else dict(self.last_request)}
