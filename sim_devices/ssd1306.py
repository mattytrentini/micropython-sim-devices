# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""SSD1306 I2C digital command/RAM model with logical MONO_VLSB output."""


class SSD1306I2C:
    # Parameter counts from the SSD1306 command table. Scrolling is deliberately
    # unsupported rather than acknowledged without an implemented effect.
    _ARGS = {0x20: 1, 0x21: 2, 0x22: 2, 0x81: 1, 0xa8: 1,
             0xad: 1, 0x8d: 1, 0xd3: 1, 0xda: 1, 0xd5: 1,
             0xd9: 1, 0xdb: 1}

    def __init__(self, bus, width=128, height=64, addr=0x3c, on_frame=None):
        if type(width) is not int or not 1 <= width <= 128:
            raise ValueError("width must be 1..128")
        if type(height) is not int or not 16 <= height <= 64 or height % 8:
            raise ValueError("height must be a multiple of 8 in 16..64")
        if type(addr) is not int or not 0 <= addr <= 127:
            raise ValueError("addr must be a 7-bit address")
        if on_frame is not None and not callable(on_frame):
            raise TypeError("on_frame must be callable")
        self.addr = addr
        self.width, self.height = width, height
        self.on_frame = on_frame
        self.ram = bytearray(128 * 8)
        self.buffer = bytearray(width * (height // 8))
        self._offset = (128 - width) // 2
        self.frames = 0
        self.powered = False
        self.contrast = 0x7f
        self.inverted = False
        self.entire_on = False
        self.segment_remap = False
        self.com_reverse = False
        self.start_line = self.display_offset = 0
        self.multiplex = height - 1
        self.addressing = 2
        self.column_start, self.column_end = 0, 127
        self.page_start, self.page_end = 0, 7
        self.column = self.page = 0
        self._pending = None
        self._parameters = bytearray()
        self.settings = {}
        bus.add_device(self)

    def _command(self, value):
        if self._pending is not None:
            self._parameters.append(value)
            if len(self._parameters) == self._ARGS[self._pending]:
                command = self._pending
                self._pending = None
                parameters = tuple(self._parameters)
                self._parameters[:] = b""
                self._apply(command, parameters)
            return
        if value in self._ARGS:
            self._pending = value
            return
        if 0x00 <= value <= 0x0f:
            self.column = (self.column & 0x70) | value
        elif 0x10 <= value <= 0x17:
            self.column = (self.column & 15) | ((value & 7) << 4)
        elif 0x40 <= value <= 0x7f:
            self.start_line = value & 63
        elif 0xb0 <= value <= 0xb7:
            self.page = value & 7
        elif value in (0xa0, 0xa1):
            self.segment_remap = bool(value & 1)
        elif value in (0xc0, 0xc8):
            self.com_reverse = bool(value & 8)
        elif value in (0xa4, 0xa5):
            self.entire_on = bool(value & 1)
        elif value in (0xa6, 0xa7):
            self.inverted = bool(value & 1)
        elif value in (0xae, 0xaf):
            self.powered = bool(value & 1)
        elif value != 0xe3:  # Documented NOP.
            raise NotImplementedError("unsupported SSD1306 command: %02x" % value)

    def _apply(self, command, args):
        value = args[0]
        if command == 0x20:
            if value not in (0, 1, 2):
                raise ValueError("invalid memory addressing mode")
            self.addressing = value
        elif command in (0x21, 0x22):
            limit = 127 if command == 0x21 else 7
            if not 0 <= value <= args[1] <= limit:
                raise ValueError("invalid SSD1306 address window")
            if command == 0x21:
                self.column_start, self.column_end = value, args[1]
                self.column = value
            else:
                self.page_start, self.page_end = value, args[1]
                self.page = value
        elif command == 0x81:
            self.contrast = value
        elif command == 0xa8:
            if not 15 <= value <= 63:
                raise ValueError("invalid multiplex ratio")
            if value != self.height - 1:
                raise NotImplementedError("multiplex must match modeled panel height")
            self.multiplex = value
        elif command == 0xd3:
            if value > 63:
                raise ValueError("invalid display offset")
            self.display_offset = value
        elif command == 0xad:
            if value not in (0x00, 0x10, 0x20, 0x30):
                raise ValueError("invalid IREF selection")
        elif command == 0x8d and value not in (0x10, 0x14):
            raise ValueError("invalid charge pump setting")
        elif command == 0xda and value not in (0x02, 0x12, 0x22, 0x32):
            raise ValueError("invalid COM pin configuration")
        elif command == 0xdb and value not in (0, 0x10, 0x20, 0x30):
            raise ValueError("invalid VCOM deselect level")
        self.settings[command] = tuple(args)

    def _data(self, value):
        if self._pending is not None:
            raise ValueError("data interrupts incomplete SSD1306 command")
        self.ram[self.page * 128 + self.column] = value
        if self.addressing == 1:
            self.page += 1
            if self.page > self.page_end:
                self.page = self.page_start
                self.column += 1
                if self.column > self.column_end:
                    self.column = self.column_start
        else:
            self.column += 1
            if self.column > (127 if self.addressing == 2 else self.column_end):
                self.column = 0 if self.addressing == 2 else self.column_start
                if self.addressing == 0:
                    self.page += 1
                    if self.page > self.page_end:
                        self.page = self.page_start

    def _publish(self, data_written=False):
        changed = False
        # SEG/COM orientation is metadata: output is driver-logical coordinates,
        # not a physical panel photograph. Start-line and offset shift RAM rows.
        shift = (self.start_line - self.display_offset) % 64
        for page in range(self.height // 8):
            for x in range(self.width):
                if not self.powered:
                    value = 0
                elif self.entire_on:
                    value = 255
                elif shift == 0:
                    value = self.ram[page * 128 + x + self._offset]
                else:
                    value = 0
                    for bit in range(8):
                        row = (page * 8 + bit + shift) % 64
                        if self.ram[(row // 8) * 128 + x + self._offset] & (1 << (row % 8)):
                            value |= 1 << bit
                if self.powered and self.inverted:
                    value ^= 255
                index = page * self.width + x
                if self.buffer[index] != value:
                    self.buffer[index] = value
                    changed = True
        if changed or data_written:
            self.frames += 1
            if self.on_frame is not None:
                self.on_frame()

    def writeto(self, buf, stop=True):
        """Parse one I2C transaction; publish only after all bytes are consumed."""
        buf = memoryview(buf)
        index = 0
        data_written = False
        while index < len(buf):
            control = buf[index]
            index += 1
            if control & 0x3f:
                raise ValueError("reserved SSD1306 control bits are nonzero")
            if index == len(buf):
                raise ValueError("SSD1306 control byte has no payload")
            end = index + 1 if control & 0x80 else len(buf)
            while index < end:
                if control & 0x40:
                    self._data(buf[index])
                    data_written = True
                else:
                    self._command(buf[index])
                index += 1
        self._publish(data_written)
        return len(buf)

    def readfrom(self, nbytes, stop=True):
        raise NotImplementedError("SSD1306 I2C interface is write-only")

    def readfrom_into(self, buf, stop=True):
        raise NotImplementedError("SSD1306 I2C interface is write-only")

    def readfrom_mem(self, memaddr, nbytes):
        raise NotImplementedError("SSD1306 has no I2C register-read protocol")

    def readfrom_mem_into(self, memaddr, buf):
        raise NotImplementedError("SSD1306 has no I2C register-read protocol")

    def writeto_mem(self, memaddr, buf):
        raise NotImplementedError("use SSD1306 control-byte transactions")

    def snapshot(self):
        return {"width": self.width, "height": self.height, "frames": self.frames,
                "powered": self.powered, "contrast": self.contrast,
                "inverted": self.inverted, "entire_on": self.entire_on,
                "segment_remap": self.segment_remap, "com_reverse": self.com_reverse,
                "start_line": self.start_line, "display_offset": self.display_offset,
                "checksum": sum(self.buffer)}
