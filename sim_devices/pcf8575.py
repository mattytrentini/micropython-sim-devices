# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Independent PCF8575 digital model (NXP PCF8575 Rev. 5, port transfers).

Only complete two-byte port transactions are supported; other lengths are
explicitly rejected. No register interface, interrupt pin, or analog physics.
A released (latched-high) pin samples its externally supplied digital level.
Snapshot reads/writes count successful transfers, saturating at 0x7fffffff.
"""
import errno


class PCF8575I2C:
    def __init__(self, bus, addr=0x20):
        if type(addr) is not int or not 0x20 <= addr <= 0x27:
            raise ValueError("PCF8575 address must be 0x20..0x27")
        self.addr = addr
        self.online = True
        self.reads = 0
        self.writes = 0
        self._latch = 0xffff
        self.raw_levels = 0xffff
        bus.add_device(self)

    @property
    def latch(self):
        return self._latch

    def inject(self, raw_u16):
        if type(raw_u16) is not int or not 0 <= raw_u16 <= 0xffff:
            raise ValueError("levels must be an unsigned 16-bit integer")
        self.raw_levels = raw_u16

    def _check(self):
        if not self.online:
            raise OSError(errno.EIO)

    def readfrom(self, nbytes, stop=True):
        self._check()
        if type(nbytes) is not int or nbytes != 2:
            raise ValueError("PCF8575 model requires a two-byte port read")
        value = self._latch & self.raw_levels
        self.reads = min(self.reads + 1, 0x7fffffff)
        return bytes((value & 255, value >> 8))

    def readfrom_into(self, buf, stop=True):
        self._check()
        if len(buf) != 2:
            raise ValueError("PCF8575 model requires a two-byte port read")
        value = self._latch & self.raw_levels
        buf[0] = value & 255
        buf[1] = value >> 8
        self.reads = min(self.reads + 1, 0x7fffffff)

    def writeto(self, buf, stop=True):
        self._check()
        if len(buf) != 2:
            raise ValueError("PCF8575 model requires a two-byte port write")
        self._latch = buf[0] | (buf[1] << 8)
        self.writes = min(self.writes + 1, 0x7fffffff)
        return 2

    def readfrom_mem(self, memaddr, nbytes):
        self._check()
        raise NotImplementedError("PCF8575 has no register interface")

    def readfrom_mem_into(self, memaddr, buf):
        self._check()
        raise NotImplementedError("PCF8575 has no register interface")

    def writeto_mem(self, memaddr, buf):
        self._check()
        raise NotImplementedError("PCF8575 has no register interface")

    def snapshot(self):
        return {"online": self.online, "latch": self._latch,
                "reads": self.reads, "writes": self.writes,
                "raw_levels": self.raw_levels,
                "sample": self._latch & self.raw_levels}
