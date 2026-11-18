# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Pure-Python model contracts; also runnable with native MicroPython unittest."""
import unittest

from sim_devices.modbus import ModbusRTUSlave, crc16
from sim_devices.ssd1306 import SSD1306I2C


def frame(payload):
    value = crc16(payload)
    return bytes(payload) + bytes((value & 255, value >> 8))


class Bus:
    def __init__(self):
        self.devices = {}

    def add_device(self, device):
        self.devices[device.addr] = device


class ModbusTests(unittest.TestCase):
    def test_crc_known_vector_and_read(self):
        self.assertEqual(crc16(b"123456789"), 0x4b37)
        model = ModbusRTUSlave(1, lambda start, count: [0x1234, 0xabcd])
        # Published canonical FC03 request CRC, independent of local builder.
        response = model.respond(b"\x01\x03\x00\x00\x00\x02\xc4\x0b")
        self.assertEqual(response[:7], b"\x01\x03\x04\x12\x34\xab\xcd")
        self.assertEqual(crc16(response), 0)
        self.assertEqual(model.snapshot()["responses"], 1)

    def test_rejections_exceptions_and_callback_contract(self):
        def missing(start, count):
            raise IndexError(start)
        model = ModbusRTUSlave(7, missing)
        for request in (b"", b"a" * 257, frame((0, 3, 0, 0, 0, 1)),
                        frame((8, 3, 0, 0, 0, 1)), b"\x07\x03\x00\x00\x00\x01\x00\x00"):
            self.assertIsNone(model.respond(request))
        for payload, code in (((7, 6, 0, 0, 0, 1), 1),
                              ((7, 3, 0, 0, 0, 0), 3),
                              ((7, 3, 0, 0, 0, 126), 3),
                              ((7, 3, 0), 3),
                              ((7, 3, 255, 255, 0, 2), 2),
                              ((7, 3, 0, 0, 0, 1), 2)):
            reply = model.respond(frame(payload))
            self.assertEqual(reply[2], code)
            self.assertEqual(crc16(reply), 0)
        for words in ([], [65536], [-1], [True]):
            invalid = ModbusRTUSlave(1, lambda start, count: words)
            with self.assertRaises(ValueError):
                invalid.respond(frame((1, 3, 0, 0, 0, 1)))

    def test_faults_and_bounded_independent_snapshots(self):
        selected = ["timeout"]
        model = ModbusRTUSlave(1, lambda start, count: [42],
                               lambda start, count: selected[0])
        request = frame((1, 3, 0, 0, 0, 1))
        self.assertIsNone(model.respond(request))
        selected[0] = "crc"
        self.assertFalse(crc16(model.respond(request)) == 0)
        selected[0] = "normal"
        self.assertEqual(crc16(model.respond(request)), 0)
        before = model.snapshot()
        before["last_request"]["start"] = 123
        self.assertEqual(model.snapshot()["last_request"]["start"], 0)
        self.assertEqual(model.snapshot()["responses"], 2)
        self.assertEqual(ModbusRTUSlave(1, lambda s, n: [0]).snapshot()["requests"], 0)
        selected[0] = "unsupported"
        with self.assertRaises(ValueError):
            model.respond(request)


class OLEDTests(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.model = SSD1306I2C(self.bus)

    def command(self, *values):
        for value in values:
            self.model.writeto(bytes((0x80, value)))

    def test_split_commands_data_and_coherent_publish(self):
        model = self.model
        visible = model.buffer
        seen = []
        model.on_frame = lambda: seen.append((model.buffer[0], model.buffer[1]))
        self.command(0xad, 0x30, 0x20, 0, 0x21, 0, 1, 0x22, 0, 0, 0xaf)
        self.assertEqual(seen, [])
        model.writeto(b"\x40\x01\x80")
        self.assertEqual(seen, [(1, 128)])
        self.assertIs(model.buffer, visible)
        self.command(0xa7)
        self.assertEqual(model.buffer[0], 254)
        self.command(0xae)
        self.assertEqual(sum(model.buffer), 0)
        self.command(0xaf, 0xa6, 0xa5)
        self.assertEqual(sum(model.buffer), 255 * 1024)
        self.command(0xa4, 0x81, 96)
        self.assertEqual(model.buffer[0], 1)
        self.assertEqual(model.snapshot()["contrast"], 96)

    def test_address_modes_windows_and_centered_panels(self):
        model = self.model
        self.command(0xaf, 0x20, 1, 0x21, 2, 3, 0x22, 1, 2)
        model.writeto(b"\x40\x11\x22\x33\x44")
        self.assertEqual(model.ram[130], 0x11)
        self.assertEqual(model.ram[258], 0x22)
        self.assertEqual(model.ram[131], 0x33)
        self.assertEqual(model.ram[259], 0x44)
        self.command(0x20, 2, 0xb3, 0x04, 0x10)
        model.writeto(b"\x40\x55")
        self.assertEqual(model.ram[388], 0x55)
        narrow = SSD1306I2C(Bus(), width=64, height=32)
        narrow.writeto(b"\x00\xaf\x20\x00\x21\x20\x5f\x22\x00\x03")
        narrow.writeto(b"\x40\x01")
        self.assertEqual(narrow.buffer[0], 1)
        self.assertEqual(sum(self.model.buffer), 0x11 + 0x22 + 0x33 + 0x44 + 0x55)

    def test_control_chaining_and_errors_are_not_fake_success(self):
        self.model.writeto(b"\x80\xaf\xc0\x01\x40\x02")
        self.assertEqual(self.model.buffer[:2], b"\x01\x02")
        for packet in (b"\x41\x00", b"\x40", b"\x00\x21\x7f\x00"):
            with self.assertRaises(ValueError):
                SSD1306I2C(Bus()).writeto(packet)
        for command in (0x26, 0x2f, 0xff):
            with self.assertRaises(NotImplementedError):
                self.command(command)
        with self.assertRaises(NotImplementedError):
            self.model.readfrom(1)
        self.command(0x81)
        with self.assertRaises(ValueError):
            self.model.writeto(b"\x40\x00")

    def test_invalid_parameters_recover_without_growing_pending_state(self):
        for _ in range(20):
            with self.assertRaises(ValueError):
                self.command(0x21, 127, 0)
            self.command(0x81, 96)
            self.assertEqual(self.model.contrast, 96)
            self.assertEqual(len(self.model._parameters), 0)
        self.command(0xaf, 0x20, 0, 0x21, 0, 0, 0x22, 0, 0)
        self.model.writeto(b"\x40\x01")
        self.command(0x41)
        self.assertEqual(self.model.buffer[0], 0)
        self.command(0x40, 0xd3, 0)
        self.assertEqual(self.model.buffer[0], 1)


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
