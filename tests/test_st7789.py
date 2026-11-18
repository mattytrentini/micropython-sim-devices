# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Deterministic ST7789 protocol contracts, runnable without native hardware."""
import errno
import unittest

from sim_devices.st7789 import ST7789SPI


class Pin:
    def __init__(self, value):
        self.level = value
        self.observers = []

    def value(self, value=None):
        if value is None:
            return self.level
        if self.level != value:
            self.level = value
            for observer in self.observers:
                observer(self)

    def add_observer(self, observer):
        self.observers.append(observer)


class Bus:
    def __init__(self):
        self.devices = []

    def add_device(self, device):
        self.devices.append(device)

    def write(self, buf):
        for device in self.devices:
            device.write(buf)


class ST7789Tests(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.cs, self.dc, self.reset, self.backlight = Pin(1), Pin(1), Pin(1), Pin(1)
        self.model = ST7789SPI(self.bus, self.cs, self.dc, self.reset, self.backlight,
                               width=4, height=3)

    def command(self, command, data=None):
        self.cs.value(0)
        self.dc.value(0)
        self.bus.write(bytes((command,)))
        if data is not None:
            self.data(data)
        self.cs.value(1)

    def data(self, buf):
        self.cs.value(0)
        self.dc.value(1)
        self.bus.write(buf)
        self.cs.value(1)

    def initialize(self):
        self.command(0x11, b"\x00")
        self.command(0x13, b"\x00")
        self.command(0x3a, b"\x55")
        self.command(0x29, b"\x00")

    def test_fragmented_parameters_pixels_cursor_wrap_and_selection(self):
        self.initialize()
        self.command(0x2a, b"\x00")
        self.data(b"\x01\x00")
        self.data(b"\x02")
        self.command(0x2b, b"\x00\x01\x00\x02")
        self.command(0x2c)
        self.data(b"\xf8")
        self.assertEqual(sum(self.model.ram), 0)
        self.data(b"\x00\x07\xe0\x00")
        self.data(b"\x1f\xff\xff")
        # A fifth pixel wraps to the start of the 2x2 window, across CS toggles.
        self.data(b"\x12\x34")
        self.assertEqual(self.model.ram[10:14], b"\x34\x12\xe0\x07")
        self.assertEqual(self.model.ram[18:22], b"\x1f\x00\xff\xff")
        before = bytes(self.model.ram)
        self.cs.value(1)
        self.bus.write(b"\xde\xad")
        self.assertEqual(self.model.ram, before)
        self.assertEqual(self.model.frames, 0)
        self.model.flush()
        self.assertEqual(self.model.buffer, self.model.ram)

    def test_flush_is_once_per_burst_with_stable_buffers_and_snapshot(self):
        self.initialize()
        model = self.model
        ram, visible = model.ram, model.buffer
        events = []
        model.on_frame = lambda: events.append(model.frames)
        self.command(0x2c)
        for index in range(1000):
            self.data(b"\xf8\x00")
            model.snapshot()  # Observing metadata must not project or publish.
        self.assertEqual(events, [])
        self.assertEqual(model.frames, 0)
        self.assertEqual(sum(visible), 0)
        self.assertTrue(model.flush())
        self.assertEqual(events, [1])
        self.assertFalse(model.flush())
        self.assertEqual(events, [1])
        self.assertIs(model.ram, ram)
        self.assertIs(model.buffer, visible)
        self.assertEqual(model.snapshot()["checksum"], sum(visible))

    def test_display_sleep_backlight_inversion_bgr_and_axis_metadata(self):
        self.initialize()
        self.command(0x2c, b"\xf8\x00")
        self.model.flush()
        self.assertEqual(self.model.buffer[:2], b"\x00\xf8")
        self.command(0x36, b"\xe8")
        self.model.flush()
        self.assertEqual(self.model.madctl, 0xe8)
        self.assertEqual(self.model.buffer[:2], b"\x1f\x00")
        self.command(0x21, b"\x00")
        self.model.flush()
        self.assertEqual(self.model.buffer[:2], b"\xe0\xff")
        original = bytes(self.model.ram)
        for off, on in ((0x28, 0x29), (0x10, 0x11)):
            self.command(off)
            self.model.flush()
            self.assertEqual(sum(self.model.buffer), 0)
            self.assertEqual(self.model.ram, original)
            self.command(on)
            self.model.flush()
            self.assertEqual(self.model.buffer[:2], b"\xe0\xff")
        self.backlight.value(0)
        self.assertTrue(self.model.dirty)
        self.model.flush()
        self.assertEqual(sum(self.model.buffer), 0)
        self.backlight.value(1)
        self.model.flush()
        self.assertEqual(self.model.buffer[:2], b"\xe0\xff")
        self.assertEqual(self.model.ram, original)

    def test_reset_preserves_gram_but_resets_controller_and_aborts_fragments(self):
        self.initialize()
        self.command(0x36, b"\x68")
        self.command(0xb7, b"\x35")
        self.command(0x2c, b"\xab\xcd\x12")
        before = bytes(self.model.ram)
        self.reset.value(0)
        self.model.flush()
        self.assertEqual(sum(self.model.buffer), 0)
        self.assertEqual(self.model.ram, before)
        self.assertTrue(self.model.sleeping)
        self.assertFalse(self.model.powered)
        self.assertEqual(self.model.settings, {})
        self.assertEqual(self.model.madctl, 0)
        self.command(0x29)  # Held-low reset ignores selected TX.
        self.assertFalse(self.model.powered)
        self.reset.value(1)
        self.initialize()
        self.model.flush()
        self.assertEqual(self.model.buffer[:2], b"\xcd\xab")
        self.command(0x2a, b"\x00")
        self.command(0x01)
        self.model.flush()
        self.assertEqual(self.model.ram, before)
        self.assertEqual(sum(self.model.buffer), 0)

    def test_offline_selected_writes_fail_without_blank_or_auto_recovery(self):
        self.initialize()
        self.command(0x2c, b"\xf8\x00")
        self.model.flush()
        before = bytes(self.model.buffer)
        self.model.online = False
        self.cs.value(1)
        self.bus.write(b"ignored")
        self.cs.value(0)
        with self.assertRaises(OSError) as raised:
            self.bus.write(b"\x00")
        self.assertEqual(raised.exception.args[0], errno.EIO)
        self.assertEqual(self.model.buffer, before)
        self.assertFalse(self.model.dirty)
        self.model.online = True
        self.assertFalse(self.model.flush())
        self.command(0x28)
        self.model.flush()
        self.assertEqual(sum(self.model.buffer), 0)

    def test_protocol_errors_are_explicit_and_bounded(self):
        self.initialize()
        with self.assertRaises(NotImplementedError):
            self.command(0xff)
        with self.assertRaises(NotImplementedError):
            self.command(0x33)  # No fake scroll support.
        with self.assertRaises(ValueError):
            self.command(0x2a, b"\x00\x00\x00\x04")
        with self.assertRaises(ValueError):
            self.command(0x2b, b"\x00\x02\x00\x01")
        with self.assertRaises(NotImplementedError):
            self.command(0x3a, b"\x66")
        self.command(0x2a, b"\x00")
        with self.assertRaises(ValueError):
            self.command(0x2c)
        self.command(0x01)
        self.initialize()
        self.command(0x2c, b"\xab")
        with self.assertRaises(ValueError):
            self.command(0x29)
        self.command(0x01)
        self.command(0x11, b"\x00")
        with self.assertRaises(ValueError):
            self.data(b"\x00")  # Only one vendor dummy byte is accepted.
        self.command(0x29)
        with self.assertRaises(ValueError):
            self.data(b"\x01")

    def test_panel_native_color_and_inversion_configuration(self):
        model = ST7789SPI(self.bus, self.cs, self.dc, width=4, height=3,
                          panel_color_order="BGR", native_inversion=True)
        self.initialize()
        self.command(0x36, b"\x68")
        self.command(0x21, b"\x00")
        self.command(0x2c, b"\xf8\x00\x07\xe0\x00\x1f")
        model.flush()
        self.assertEqual(model.buffer[:6], b"\x00\xf8\xe0\x07\x1f\x00")
        self.assertEqual(model.snapshot()["panel_color_order"], "BGR")
        self.assertTrue(model.snapshot()["native_inversion"])


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
