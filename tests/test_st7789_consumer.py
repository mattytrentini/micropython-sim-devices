# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Native MicroPython: unchanged pinned ST7789 driver, CO16 config and ROM font.

Install st7789py, co16_tft_config and vga1_8x16 from the pinned upstream source;
no substitute framebuf, time, driver, font or firmware is used by these tests.
"""
import errno
import unittest
import mock_machine as machine

machine.register_as_machine()

import co16_tft_config
import st7789py as st7789
import vga1_8x16 as font
from sim_devices.st7789 import ST7789SPI


class ST7789Consumer(unittest.TestCase):
    def setUp(self):
        machine.reset_simulation()
        self.spi = machine.SPI(2)
        self.cs = machine.Pin(4, machine.Pin.OUT, value=1)
        self.dc = machine.Pin(0, machine.Pin.OUT, value=1)
        self.reset = machine.Pin(5, machine.Pin.OUT, value=1)
        self.backlight = machine.Pin(40, machine.Pin.OUT, value=0)
        self.model = ST7789SPI(self.spi, self.cs, self.dc, self.reset,
                               self.backlight, width=320, height=240,
                               panel_color_order="BGR", native_inversion=True)
        self.driver = co16_tft_config.config()

    def tearDown(self):
        machine.reset_simulation()

    def pixel(self, x, y):
        index = (y * self.model.width + x) * 2
        return self.model.buffer[index] | (self.model.buffer[index + 1] << 8)

    def test_real_configuration_double_init_color_and_font(self):
        model, driver = self.model, self.driver
        self.assertIs(driver.spi, self.spi)
        self.assertEqual(self.spi.baudrate, 20000000)
        self.assertEqual((driver.width, driver.height), (320, 240))
        self.assertEqual(model.madctl, 0x68)
        self.assertEqual(model.colmod, 0x55)
        self.assertTrue(model.powered)
        self.assertFalse(model.sleeping)
        self.assertTrue(model.inverted)
        self.assertEqual(self.backlight.value(), 1)
        self.assertEqual(model.settings[0xb2], (12, 12, 0, 51, 51))
        self.assertEqual(model.settings[0xb6], (10, 130))
        self.assertEqual(model.settings[0xc2], (1, 255))
        self.assertEqual(len(model.settings[0xe0]), 14)
        self.assertEqual(len(model.settings[0xe1]), 14)
        for number in (9, 14, 13):
            self.assertEqual(machine.Pin(number).value(), 1)
        self.assertEqual(model.frames, 0)
        model.flush()
        self.assertEqual(sum(model.buffer), 0)
        events = []
        model.on_frame = lambda: events.append(model.frames)
        driver.fill_rect(0, 0, 4, 2, st7789.RED)
        driver.hline(4, 0, 4, st7789.GREEN)
        driver.vline(319, 237, 3, st7789.BLUE)
        driver.pixel(318, 239, st7789.WHITE)
        driver.text(font, "A0!", 8, 16, st7789.YELLOW, st7789.BLUE)
        self.assertEqual(events, [])
        self.assertEqual(model.frames, 1)
        self.assertTrue(model.flush())
        self.assertEqual(events, [2])
        self.assertFalse(model.flush())
        self.assertEqual(self.pixel(0, 0), st7789.RED)
        self.assertEqual(self.pixel(3, 1), st7789.RED)
        self.assertEqual(self.pixel(7, 0), st7789.GREEN)
        self.assertEqual(self.pixel(319, 239), st7789.BLUE)
        self.assertEqual(self.pixel(318, 239), st7789.WHITE)
        self.assertEqual(self.pixel(10, 10), st7789.BLACK)
        lit = 0
        for char_index, char in enumerate("A0!"):
            for row in range(font.HEIGHT):
                glyph = font.FONT[(ord(char) - font.FIRST) * font.HEIGHT + row]
                for column in range(font.WIDTH):
                    foreground = bool(glyph & (128 >> column))
                    expected = st7789.YELLOW if foreground else st7789.BLUE
                    self.assertEqual(self.pixel(8 + char_index * 8 + column, 16 + row),
                                     expected)
                    lit += int(foreground)
        self.assertTrue(lit > 0)
        self.assertEqual(model.buffer, model.ram)
        self.assertEqual(len(self.spi.write_buf) <= self.spi.capture_limit, True)

    def test_real_power_inversion_reset_and_backlight(self):
        model, driver = self.model, self.driver
        driver.fill(st7789.RED)
        model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED)
        driver.inversion_mode(False)
        model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED ^ 0xffff)
        driver.inversion_mode(True)
        driver.sleep_mode(True)
        model.flush()
        self.assertEqual(sum(model.buffer), 0)
        self.assertEqual(model.ram[:2], b"\x00\xf8")
        driver.sleep_mode(False)
        model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED)
        driver._write(b"\x28")
        model.flush()
        self.assertEqual(sum(model.buffer), 0)
        driver._write(b"\x29")
        self.backlight.off()
        model.flush()
        self.assertEqual(sum(model.buffer), 0)
        self.backlight.on()
        model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED)
        driver.soft_reset()
        model.flush()
        self.assertTrue(model.sleeping)
        self.assertFalse(model.powered)
        self.assertEqual(sum(model.buffer), 0)
        self.assertEqual(model.ram[:2], b"\x00\xf8")
        driver.init(driver.init_cmds)
        driver.rotation(1)
        model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED)
        driver.hard_reset()
        model.flush()
        self.assertEqual(sum(model.buffer), 0)
        self.assertEqual(model.ram[:2], b"\x00\xf8")

    def test_real_transport_fault_and_unselected_bus(self):
        self.model.flush()
        self.model.online = False
        # Another peripheral's traffic on the same SPI controller is ignored.
        self.cs.on()
        self.spi.write(b"\xff\xff")
        self.assertFalse(self.model.dirty)
        with self.assertRaises(OSError) as raised:
            self.driver.pixel(10, 10, st7789.RED)
        self.assertEqual(raised.exception.args[0], errno.EIO)
        self.assertEqual(sum(self.model.ram), 0)
        self.model.online = True
        # Restoring connectivity performs no draw, publish or driver restart.
        self.assertFalse(self.model.flush())
        self.driver.pixel(10, 10, st7789.RED)
        self.model.flush()
        self.assertEqual(self.pixel(10, 10), st7789.RED)


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
