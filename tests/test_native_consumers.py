# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Requires native framebuf, mock-machine 1.2, unchanged ssd1306 and umodbus."""
import unittest
import mock_machine as machine

machine.register_as_machine()

from ssd1306 import SSD1306_I2C
from umodbus.serial import Serial
from sim_devices.ssd1306 import SSD1306I2C
from sim_devices.modbus import ModbusRTUSlave


class NativeConsumers(unittest.TestCase):
    def setUp(self):
        machine.reset_simulation()

    def tearDown(self):
        machine.reset_simulation()

    def test_real_framebuf_driver_init_logo_show_and_display_commands(self):
        bus = machine.I2C("oled-test")
        model = SSD1306I2C(bus)
        driver = SSD1306_I2C(128, 64, bus)
        self.assertEqual(model.settings[0xad], (0x30,))
        self.assertEqual(model.buffer, driver.buffer)
        self.assertTrue(model.powered)
        coherent = []
        model.on_frame = lambda: coherent.append(bytes(model.buffer))
        driver.fill(0)
        driver.rect(0, 0, 128, 64, 1)
        driver.text("MicroPython", 8, 8, 1)
        driver.line(8, 30, 110, 54, 1)
        driver.pixel(127, 63, 1)
        driver.show()
        self.assertEqual(len(coherent), 1)
        self.assertEqual(coherent[0], bytes(driver.buffer))
        self.assertTrue(sum(model.buffer) > 0)
        self.assertTrue(model.buffer[1023] & 128)
        driver.contrast(96)
        self.assertEqual(model.contrast, 96)
        driver.invert(1)
        for index in range(1024):
            self.assertEqual(model.buffer[index], driver.buffer[index] ^ 255)
        driver.poweroff()
        self.assertEqual(sum(model.buffer), 0)
        driver.poweron()
        driver.invert(0)
        self.assertEqual(model.buffer, driver.buffer)
        driver.rotate(0)
        self.assertFalse(model.segment_remap)
        self.assertFalse(model.com_reverse)
        driver.rotate(1)
        self.assertTrue(model.segment_remap)
        self.assertTrue(model.com_reverse)

    def test_real_client_reads_and_rejects_missing_and_bad_crc(self):
        fault = ["normal"]
        model = ModbusRTUSlave(9, lambda start, count: [start + n for n in range(count)],
                               lambda start, count: fault[0])
        machine.UART("meter-test").set_responder(model.respond)
        client = Serial(uart_id="meter-test", baudrate=115200, pins=(2, 3))
        self.assertEqual(tuple(client.read_holding_registers(9, 20, 3, signed=False)), (20, 21, 22))
        for selected, message in (("timeout", "no data received from slave"),
                                  ("crc", "invalid response CRC")):
            fault[0] = selected
            try:
                client.read_holding_registers(9, 20, 3, signed=False)
            except OSError as error:
                self.assertEqual(str(error), message)
            else:
                self.fail("real Modbus client accepted " + selected)
        fault[0] = "normal"
        self.assertEqual(tuple(client.read_holding_registers(9, 20, 1, signed=False)), (20,))
        self.assertEqual(model.snapshot()["responses"], 3)


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
