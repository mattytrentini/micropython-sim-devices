# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Independent XL9535 digital model, Xinluda XL9535/XL9555 Rev. 2.4.

Registers 0..7 and pair-wise pointer toggling are implemented. Output pins
sample their push-pull latch; injected levels apply to input pins only.
Polarity inverts the input register, never the output latch. No INT/analog model.
Reference: https://wiki.lilygo.cc/datasheet/xl9555.pdf (sections 9.1..9.3).
Snapshot counters saturate at 0x7fffffff: reads count successful read
transfers; writes count data-bearing writes, not address-pointer selection.
"""
import errno


def _u16(value):
    if type(value) is not int or not 0 <= value <= 0xffff:
        raise ValueError("value must be an unsigned 16-bit integer")
    return value


class XL9535I2C:
    def __init__(self, bus, addr=0x20):
        if type(addr) is not int or not 0x20 <= addr <= 0x27:
            raise ValueError("XL9535 address must be 0x20..0x27")
        self.addr = addr
        self.online = True
        self.reads = 0
        self.writes = 0
        self.raw_levels = 0xffff
        self._config = 0xffff
        self._polarity = 0
        self._output = 0xffff
        self._pointer = 0
        bus.add_device(self)

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = _u16(value)

    @property
    def polarity(self):
        return self._polarity

    @polarity.setter
    def polarity(self, value):
        self._polarity = _u16(value)

    @property
    def output(self):
        return self._output

    def inject(self, raw_u16):
        self.raw_levels = _u16(raw_u16)

    def _check(self):
        if not self.online:
            raise OSError(errno.EIO)

    def _select(self, register):
        if type(register) is not int or not 0 <= register <= 7:
            raise ValueError("XL9535 register must be 0..7")
        self._pointer = register

    def _read(self, register):
        pair = register >> 1
        if pair == 0:
            value = ((self.raw_levels & self._config) |
                     (self._output & (~self._config & 0xffff))) ^ self._polarity
        elif pair == 1:
            value = self._output
        elif pair == 2:
            value = self._polarity
        else:
            value = self._config
        return (value >> ((register & 1) * 8)) & 255

    def _write(self, register, value):
        pair = register >> 1
        if pair == 0:
            raise ValueError("XL9535 input registers are read-only")
        shift = (register & 1) * 8
        mask = 255 << shift
        if pair == 1:
            self._output = (self._output & ~mask) | (value << shift)
        elif pair == 2:
            self._polarity = (self._polarity & ~mask) | (value << shift)
        else:
            self._config = (self._config & ~mask) | (value << shift)

    def readfrom(self, nbytes, stop=True):
        self._check()
        if type(nbytes) is not int or nbytes < 0:
            raise ValueError("read length must be nonnegative")
        buf = bytearray(nbytes)
        self.readfrom_into(buf, stop)
        return bytes(buf)

    def readfrom_into(self, buf, stop=True):
        self._check()
        for index in range(len(buf)):
            buf[index] = self._read(self._pointer)
            self._pointer ^= 1
        self.reads = min(self.reads + 1, 0x7fffffff)

    def writeto(self, buf, stop=True):
        self._check()
        if not len(buf):
            return 0
        self._select(buf[0])
        if len(buf) > 1 and self._pointer < 2:
            raise ValueError("XL9535 input registers are read-only")
        for index in range(1, len(buf)):
            self._write(self._pointer, buf[index])
            self._pointer ^= 1
        if len(buf) > 1:
            self.writes = min(self.writes + 1, 0x7fffffff)
        return len(buf)

    def readfrom_mem(self, memaddr, nbytes):
        self._check()
        self._select(memaddr)
        return self.readfrom(nbytes)

    def readfrom_mem_into(self, memaddr, buf):
        self._check()
        self._select(memaddr)
        self.readfrom_into(buf)

    def writeto_mem(self, memaddr, buf):
        self._check()
        self._select(memaddr)
        if len(buf) and self._pointer < 2:
            raise ValueError("XL9535 input registers are read-only")
        for value in buf:
            self._write(self._pointer, value)
            self._pointer ^= 1
        if len(buf):
            self.writes = min(self.writes + 1, 0x7fffffff)

    def snapshot(self):
        return {"online": self.online, "raw_levels": self.raw_levels,
                "reads": self.reads, "writes": self.writes,
                "config": self._config, "polarity": self._polarity,
                "output": self._output}
