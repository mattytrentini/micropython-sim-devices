# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""ST7789 digital SPI model, with coalesced logical little-endian RGB565 output.

Coordinates are the configured logical view, not a physical panel photograph.
MADCTL axes are metadata; BGR and inversion affect visible colors. Scroll and
non-RGB565 writes are unsupported. Electrical/gamma settings are metadata, not
analog simulation. Reset preserves GRAM, resets controller state and blanks the
view. Call flush() once per drawing burst, never publish on each SPI write.
"""
import errno


class ST7789SPI:
    _ARGS = {0x2a: 4, 0x2b: 4, 0x36: 1, 0x3a: 1,
             0xb2: 5, 0xb6: 2, 0xb7: 1, 0xbb: 1, 0xc0: 1,
             0xc2: 2, 0xc3: 1, 0xc4: 1, 0xc6: 1, 0xd0: 2,
             0xe0: 14, 0xe1: 14}
    _ZERO = (0x00, 0x01, 0x10, 0x11, 0x13, 0x20, 0x21, 0x28, 0x29)

    def __init__(self, spi, cs, dc, reset=None, backlight=None,
                 width=240, height=320, panel_color_order="RGB",
                 native_inversion=False, on_frame=None):
        if type(width) is not int or not 1 <= width <= 320:
            raise ValueError("width must be 1..320")
        if type(height) is not int or not 1 <= height <= 320:
            raise ValueError("height must be 1..320")
        if panel_color_order not in ("RGB", "BGR"):
            raise ValueError("panel_color_order must be RGB or BGR")
        if type(native_inversion) is not bool:
            raise ValueError("native_inversion must be bool")
        self.spi, self.cs, self.dc = spi, cs, dc
        self.reset, self.backlight = reset, backlight
        self.width, self.height = width, height
        self.panel_color_order = panel_color_order
        self.native_inversion = native_inversion
        self.on_frame = on_frame
        self.online = True
        self.ram = bytearray(width * height * 2)
        self.buffer = bytearray(len(self.ram))
        self.frames = 0
        self._checksum = 0
        self._reset_controller()
        if reset is not None:
            reset.add_observer(self._reset_changed)
        if backlight is not None:
            backlight.add_observer(self._backlight_changed)
        spi.add_device(self)

    def _reset_controller(self):
        # ST7789 reset leaves display memory intact. Its power-on content is
        # unspecified on hardware; this model initializes it to zero once only.
        self.powered = False
        self.sleeping = True
        self.inverted = False
        self.madctl = 0
        self.colmod = 0x06
        self.normal_mode = True
        self.settings = {}
        self.column_start, self.column_end = 0, self.width - 1
        self.row_start, self.row_end = 0, self.height - 1
        self._x = self._y = 0
        self._command = None
        self._params = bytearray()
        self._remaining = 0
        self._pixel_high = None
        self._dummy_allowed = False
        self.dirty = True

    def _reset_changed(self, pin):
        if pin.value() == 0:
            self._reset_controller()
        else:
            self.dirty = True

    def _backlight_changed(self, pin):
        self.dirty = True

    def _start_command(self, command):
        if command not in self._ARGS and command not in self._ZERO and command != 0x2c:
            raise NotImplementedError("unsupported ST7789 command: %02x" % command)
        # Reset can abort an incomplete transfer, just as the reset pin can.
        if command == 0x01:
            self._reset_controller()
        elif self._remaining or self._pixel_high is not None:
            raise ValueError("incomplete ST7789 parameters or pixel")
        self._command = command
        self._params = bytearray()
        self._remaining = self._ARGS.get(command, 0)
        self._dummy_allowed = command in self._ZERO
        if command == 0x2c:
            if self.colmod != 0x55:
                raise NotImplementedError("ST7789 RAM writes require RGB565 COLMOD 55")
            self._x, self._y = self.column_start, self.row_start
        elif command in (0x10, 0x11):
            self.sleeping = command == 0x10
            self.dirty = True
        elif command in (0x20, 0x21):
            self.inverted = command == 0x21
            self.dirty = True
        elif command in (0x28, 0x29):
            self.powered = command == 0x29
            self.dirty = True
        elif command == 0x13:
            self.normal_mode = True

    def _apply_params(self):
        command, args = self._command, self._params
        if command in (0x2a, 0x2b):
            start = args[0] * 256 + args[1]
            end = args[2] * 256 + args[3]
            size = self.width if command == 0x2a else self.height
            if not 0 <= start <= end < size:
                raise ValueError("ST7789 window outside configured logical view")
            if command == 0x2a:
                self.column_start, self.column_end = start, end
            else:
                self.row_start, self.row_end = start, end
        elif command == 0x36:
            self.madctl = args[0]
            self.dirty = True
        elif command == 0x3a:
            if args[0] != 0x55:
                raise NotImplementedError("only ST7789 RGB565 COLMOD 55 is supported")
            self.colmod = args[0]
        else:
            self.settings[command] = tuple(args)

    def _data(self, value):
        if self._remaining:
            self._params.append(value)
            self._remaining -= 1
            if not self._remaining:
                self._apply_params()
        elif self._command == 0x2c:
            if self._pixel_high is None:
                self._pixel_high = value
                return
            index = (self._y * self.width + self._x) * 2
            if self.ram[index] != value or self.ram[index + 1] != self._pixel_high:
                self.ram[index] = value
                self.ram[index + 1] = self._pixel_high
                self.dirty = True
            self._pixel_high = None
            self._x += 1
            if self._x > self.column_end:
                self._x = self.column_start
                self._y += 1
                if self._y > self.row_end:
                    self._y = self.row_start
        elif self._dummy_allowed and value == 0:
            # The real vendor-compatible initialization sends one zero byte
            # after nominally argument-free commands. It is not a pixel.
            self._dummy_allowed = False
        else:
            raise ValueError("unexpected ST7789 data")

    def write(self, buf):
        """Consume selected SPI TX synchronously, without retaining its buffer."""
        if self.cs is not None and self.cs.value() != 0:
            return
        if not self.online:
            raise OSError(errno.EIO)
        if self.reset is not None and self.reset.value() != 1:
            return
        level = self.dc.value()
        if level not in (0, 1):
            raise ValueError("ST7789 DC level is unknown")
        if level == 0:
            for value in buf:
                self._start_command(value)
        else:
            for value in buf:
                self._data(value)

    def flush(self):
        """Project at most once per dirty burst; return whether a frame published.

        Both bytearrays remain stable. No full-frame temporary or per-write
        projection is made. The callback is no-argument and runs only here.
        """
        if not self.dirty:
            return False
        lit = (self.powered and not self.sleeping and
               (self.backlight is None or self.backlight.value() == 1) and
               (self.reset is None or self.reset.value() == 1))
        swap = bool(self.madctl & 8) != (self.panel_color_order == "BGR")
        invert = self.inverted != self.native_inversion
        if lit and not swap and not invert:
            # The normal configured panel path is one native in-place copy,
            # not a Python pixel loop or a temporary full-frame allocation.
            self.buffer[:] = self.ram
            self._checksum = sum(self.buffer)
        else:
            checksum = 0
            for index in range(0, len(self.ram), 2):
                color = self.ram[index] | (self.ram[index + 1] << 8)
                if not lit:
                    color = 0
                else:
                    if swap:
                        color = (color & 0x07e0) | ((color & 0xf800) >> 11) | ((color & 0x001f) << 11)
                    if invert:
                        color ^= 0xffff
                low, high = color & 255, color >> 8
                self.buffer[index], self.buffer[index + 1] = low, high
                checksum += low + high
            self._checksum = checksum
        self.dirty = False
        self.frames += 1
        if self.on_frame is not None:
            self.on_frame()
        return True

    def snapshot(self):
        """Return metadata without flushing or copying the framebuffer."""
        return {"width": self.width, "height": self.height, "frames": self.frames,
                "checksum": self._checksum, "powered": self.powered,
                "sleeping": self.sleeping,
                "backlight": self.backlight is None or self.backlight.value() == 1,
                "reset": self.reset is not None and self.reset.value() == 0,
                "inverted": self.inverted, "madctl": self.madctl,
                "colmod": self.colmod, "normal_mode": self.normal_mode,
                "panel_color_order": self.panel_color_order,
                "native_inversion": self.native_inversion,
                "settings": {"%02x" % command: list(values)
                             for command, values in self.settings.items()},
                "column_start": self.column_start, "column_end": self.column_end,
                "row_start": self.row_start, "row_end": self.row_end,
                "online": self.online, "dirty": self.dirty}
