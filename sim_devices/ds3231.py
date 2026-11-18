# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Independent DS3231 digital calendar model (Maxim DS3231 Rev. 10).

Supports time registers 00..06 (BCD, 12/24-hour), control 0e and status 0f,
plus the normal address pointer. Calendar range is 2000..2099; century-bit
writes and overflow beyond that range fail explicitly. Alarm, aging and
thermal registers/conversion are deliberately unsupported, not fake sensors.
Control is stored metadata (except unsupported CONV); no battery, oscillator
failure, SQW or alarm physics. OSF is environmental, not inferred from time.
Native ticks_ms/ticks_diff are used without modifying global time. Injected
clock_ms must be a monotonic integer-millisecond callable (no wraparound).
Snapshot counters saturate at 0x7fffffff: reads count successful read
transfers; writes count data-bearing writes, not address-pointer selection.
"""
import errno
import time


def _month_days(year, month):
    if month == 2:
        return 29 if year % 4 == 0 and (year % 100 != 0 or year % 400 == 0) else 28
    return 30 if month in (4, 6, 9, 11) else 31


def _validate(value):
    if not isinstance(value, (tuple, list)) or len(value) != 7:
        raise ValueError("datetime must contain year, month, day, hour, minute, second, weekday")
    if any(type(part) is not int for part in value):
        raise ValueError("datetime fields must be integers")
    y, m, d, h, minute, s, weekday = value
    if not (2000 <= y <= 2099 and 1 <= m <= 12 and
            1 <= d <= _month_days(y, m) and 0 <= h <= 23 and
            0 <= minute <= 59 and 0 <= s <= 59 and 1 <= weekday <= 7):
        raise ValueError("invalid DS3231 calendar datetime")
    return tuple(value)


def _bcd(value):
    return ((value // 10) << 4) | (value % 10)


def _decimal(value):
    if (value & 15) > 9 or (value >> 4) > 9:
        raise ValueError("invalid BCD digit")
    return (value >> 4) * 10 + (value & 15)


class DS3231I2C:
    def __init__(self, bus, addr=0x68, datetime=(2000, 1, 1, 0, 0, 0, 6),
                 osf=True, clock_ms=None):
        if type(addr) is not int or not 0x08 <= addr <= 0x77:
            raise ValueError("invalid I2C address")
        self.addr = addr
        self.online = True
        self.reads = 0
        self.writes = 0
        self._native_ticks = clock_ms is None and hasattr(time, "ticks_ms")
        if clock_ms is None:
            clock_ms = time.ticks_ms if self._native_ticks else self._monotonic_ms
        self._clock_ms = clock_ms
        self._status = 0x08  # EN32kHz power-on state; BSY and alarm flags clear.
        self._control = 0x1c
        self._pointer = 0
        self._hour12 = False
        self._timebuf = bytearray(7)
        self.set_datetime(datetime)
        self.set_osf(osf)
        bus.add_device(self)

    @staticmethod
    def _monotonic_ms():
        return int(time.monotonic() * 1000)

    def set_datetime(self, value):
        value = _validate(value)
        self._datetime = value
        self._last_ms = self._clock_ms()
        self._remainder_ms = 0
        self._encode()

    def set_osf(self, value):
        if type(value) is not bool:
            raise ValueError("OSF must be boolean")
        self._status = (self._status & 0x7f) | (0x80 if value else 0)

    def _check(self):
        if not self.online:
            raise OSError(errno.EIO)

    def _encode(self):
        y, m, d, h, minute, s, weekday = self._datetime
        buf = self._timebuf
        buf[0] = _bcd(s)
        buf[1] = _bcd(minute)
        buf[2] = (0x40 | (0x20 if h >= 12 else 0) | _bcd(h % 12 or 12)) if self._hour12 else _bcd(h)
        buf[3] = weekday
        buf[4] = _bcd(d)
        buf[5] = _bcd(m)
        buf[6] = _bcd(y - 2000)

    @staticmethod
    def _decode(buf):
        hour = buf[2]
        if hour & 0x80 or buf[5] & 0x80:
            raise ValueError("reserved hour/unsupported century bit")
        if hour & 0x40:
            h = _decimal(hour & 0x1f)
            if not 1 <= h <= 12:
                raise ValueError("invalid 12-hour clock value")
            h = h % 12 + (12 if hour & 0x20 else 0)
        else:
            h = _decimal(hour)
        return _validate((2000 + _decimal(buf[6]), _decimal(buf[5]),
                          _decimal(buf[4]), h, _decimal(buf[1]),
                          _decimal(buf[0]), _decimal(buf[3])))

    def _sync(self):
        now = self._clock_ms()
        delta = time.ticks_diff(now, self._last_ms) if self._native_ticks else now - self._last_ms
        if delta < 0:
            raise ValueError("clock_ms must be monotonic")
        elapsed = self._remainder_ms + delta
        seconds, remainder = divmod(elapsed, 1000)
        if seconds:
            y, m, d, h, minute, s, weekday = self._datetime
            days, seconds_in_day = divmod(h * 3600 + minute * 60 + s + seconds, 86400)
            weekday = (weekday - 1 + days) % 7 + 1
            if days > 36525:
                raise OverflowError("DS3231 model calendar exceeds 2099")
            while days:
                left = _month_days(y, m) - d
                if days <= left:
                    d += days
                    break
                days -= left + 1
                d = 1
                m += 1
                if m == 13:
                    y += 1
                    m = 1
                    if y > 2099:
                        raise OverflowError("DS3231 model calendar exceeds 2099")
            h, seconds_in_hour = divmod(seconds_in_day, 3600)
            minute, s = divmod(seconds_in_hour, 60)
            self._datetime = (y, m, d, h, minute, s, weekday)
            self._encode()
        self._last_ms = now
        self._remainder_ms = remainder

    @staticmethod
    def _register(register):
        if type(register) is not int or not 0 <= register <= 0x12:
            raise ValueError("DS3231 register must be 0x00..0x12")
        if not (register <= 6 or register in (0x0e, 0x0f)):
            raise NotImplementedError("DS3231 alarm, aging and temperature registers unsupported")

    def _range(self, register, count):
        self._register(register)
        if type(count) is not int or not 0 <= count <= 19:
            raise ValueError("DS3231 transfer length must be 0..19")
        for index in range(count):
            self._register((register + index) % 19)

    def readfrom(self, nbytes, stop=True):
        self._check()
        self._range(self._pointer, nbytes)
        buf = bytearray(nbytes)
        self.readfrom_into(buf, stop)
        return bytes(buf)

    def readfrom_into(self, buf, stop=True):
        self._check()
        self._range(self._pointer, len(buf))
        self._sync()
        for index in range(len(buf)):
            reg = self._pointer
            buf[index] = self._timebuf[reg] if reg <= 6 else (self._control if reg == 0x0e else self._status)
            self._pointer = (reg + 1) % 19
        self.reads = min(self.reads + 1, 0x7fffffff)

    def _write(self, register, buf, offset=0):
        count = len(buf) - offset
        self._range(register, count)
        self._sync()
        if count and register <= 6:
            candidate = bytearray(self._timebuf)
            for index in range(count):
                candidate[register + index] = buf[offset + index]
            value = self._decode(candidate)
            self._hour12 = bool(candidate[2] & 0x40)
            self._datetime = value
            self._encode()
            # Only a write to seconds resets the countdown chain.
            if register == 0:
                self._last_ms = self._clock_ms()
                self._remainder_ms = 0
        else:
            # Validate before changing either register in a combined transfer.
            if count and register == 0x0e and buf[offset] & 0x20:
                raise NotImplementedError("DS3231 forced temperature conversion unsupported")
            for index in range(count):
                reg = register + index
                value = buf[offset + index]
                if reg == 0x0e:
                    self._control = value
                else:
                    # OSF/A2F/A1F clear on writing zero; writing one retains.
                    # BSY is read-only and reserved bits read zero.
                    self._status = (self._status & value & 0x83) | (value & 0x08)
        self._pointer = (register + count) % 19
        if count:
            self.writes = min(self.writes + 1, 0x7fffffff)

    def writeto(self, buf, stop=True):
        self._check()
        if not len(buf):
            return 0
        self._register(buf[0])
        if len(buf) == 1:
            self._pointer = buf[0]
        else:
            self._write(buf[0], buf, 1)
        return len(buf)

    def readfrom_mem(self, memaddr, nbytes):
        self._check()
        self._range(memaddr, nbytes)
        self._pointer = memaddr
        return self.readfrom(nbytes)

    def readfrom_mem_into(self, memaddr, buf):
        self._check()
        self._range(memaddr, len(buf))
        self._pointer = memaddr
        self.readfrom_into(buf)

    def writeto_mem(self, memaddr, buf):
        self._check()
        self._write(memaddr, buf)

    def snapshot(self):
        self._sync()
        y, m, d, h, minute, s, weekday = self._datetime
        return {"online": self.online,
                "reads": self.reads, "writes": self.writes,
                "datetime": "%04d-%02d-%02dT%02d:%02d:%02d" % (y, m, d, h, minute, s),
                "weekday": weekday, "osf": bool(self._status & 0x80),
                "status": self._status, "control": self._control}
