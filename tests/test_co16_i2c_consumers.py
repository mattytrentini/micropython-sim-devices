# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Native mock-machine bus with the unchanged MIT production drivers.

Requires pcf8575 8d4c54e, xl9535 4d44bb4 and ds3231 44654c9; no copied
consumer implementation or host machine/time replacement is used.
"""
import errno
import unittest
import mock_machine as machine
from pcf8575 import PCF8575
from xl9535 import XL9535
from ds3231 import DS3231
from sim_devices.pcf8575 import PCF8575I2C
from sim_devices.xl9535 import XL9535I2C
from sim_devices.ds3231 import DS3231I2C


def state(model):
    value = model.snapshot()
    del value["reads"]
    return value


class ObservedBus:
    """Observe actual bus writes, forwarding all operations to the real HAL."""
    def __init__(self, bus):
        self.bus = bus
        self.writes = 0

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def writeto(self, addr, buf, stop=True):
        self.writes += 1
        return self.bus.writeto(addr, buf, stop)

    def writeto_mem(self, addr, register, buf):
        self.writes += 1
        return self.bus.writeto_mem(addr, register, buf)


class Consumers(unittest.TestCase):
    def setUp(self):
        machine.reset_simulation()
        self.ms = 0
        self.bus = ObservedBus(machine.I2C("expander-rtc-consumers"))
        self.relays = PCF8575I2C(self.bus, addr=0x20)
        self.inputs = XL9535I2C(self.bus, addr=0x21)
        self.rtc = DS3231I2C(self.bus, datetime=(2024, 2, 28, 23, 59, 59, 3),
                            osf=False, clock_ms=self.clock_ms)
        self.pcf = PCF8575(self.bus, address=0x20)
        self.xl = XL9535(self.bus, address=0x21)
        self.ds = DS3231(self.bus)

    def clock_ms(self):
        return self.ms

    def tearDown(self):
        machine.reset_simulation()

    def test_constructors_and_repeated_reads_never_control_devices(self):
        self.assertEqual(self.bus.writes, 0)
        before = (state(self.relays), state(self.inputs), state(self.rtc))
        for _ in range(3):
            self.assertEqual(self.pcf.port, 0xffff)
            self.assertEqual(self.xl.input, 0xffff)
            self.assertEqual(self.xl.config, 0xffff)
            self.assertEqual(self.ds.datetime(), (2024, 2, 28, 3, 23, 59, 59, 0))
            self.assertEqual(self.ds.OSF(), False)
        self.assertEqual(self.bus.writes, 0)
        self.assertEqual((state(self.relays), state(self.inputs), state(self.rtc)), before)
        for model in (self.relays, self.inputs, self.rtc):
            self.assertEqual(model.snapshot()["writes"], 0)
            self.assertTrue(model.snapshot()["reads"] > 0)
        self.rtc.set_osf(True)
        before = state(self.rtc)
        self.ds.datetime()  # Actual driver's warning is intentional; GET is not a setter.
        self.assertTrue(self.ds.OSF())
        self.assertEqual(state(self.rtc), before)
        self.assertEqual(self.bus.writes, 0)

    def test_all_sixteen_active_low_output_writes_preserve_other_bits(self):
        for channel in range(16):
            mask = 1 << channel
            before = self.pcf.port
            self.pcf.port = before & ~mask
            self.assertEqual(self.relays.latch, before & ~mask)
            self.assertEqual(self.pcf.port, before & ~mask)
        self.assertEqual(self.relays.latch, 0)
        for channel in range(16):
            before = self.pcf.port
            self.pcf.port = before | (1 << channel)
            self.assertEqual(self.relays.latch, before | (1 << channel))
        self.assertEqual(self.relays.latch, 0xffff)
        self.relays.inject(0xfffe)
        self.assertEqual(self.pcf.port, 0xfffe)
        self.assertEqual(self.relays.latch, 0xffff)

    def test_real_input_polarity_direction_and_individual_output_write(self):
        for index in range(16):
            raw = 0xffff ^ (1 << index)
            self.inputs.inject(raw)
            self.assertEqual(self.xl.input, raw)
        self.inputs.inject(0xa55a)
        self.xl.polarity = 0x8421
        self.assertEqual(self.xl.polarity, 0x8421)
        self.assertEqual(self.xl.input, 0xa55a ^ 0x8421)
        self.xl.output = 0x1234
        self.xl.config = 0xff00
        self.assertEqual(self.xl.config, 0xff00)
        self.assertEqual(self.xl.input, 0xa534 ^ 0x8421)
        self.xl.pin(0, 1)
        self.assertEqual(self.xl.output, 0x1235)
        self.assertEqual(self.inputs.output, 0x1235)
        self.assertEqual(self.xl.config, 0xff00)  # No silent repair to all-input.

    def test_real_clock_set_bcd_tick_leap_and_status_preservation(self):
        self.bus.writeto_mem(0x68, 14, b"\x04")
        self.rtc.set_osf(True)
        self.ds.datetime((2024, 2, 28, 23, 59, 59, 3))
        self.assertFalse(self.ds.OSF())
        self.assertEqual(self.rtc.snapshot()["status"], 8)
        self.assertEqual(self.rtc.snapshot()["control"], 4)
        self.assertEqual(self.bus.readfrom_mem(0x68, 0, 7), b"\x59\x59\x23\x03\x28\x02\x24")
        self.ms = 1000
        self.assertEqual(self.ds.datetime(), (2024, 2, 29, 4, 0, 0, 0, 0))
        self.ms += 86400000
        self.assertEqual(self.ds.datetime(), (2024, 3, 1, 5, 0, 0, 0, 0))
        self.ds.datetime((2026, 12, 31, 23, 59, 59, 4))
        self.ms += 1000
        self.assertEqual(self.ds.datetime(), (2027, 1, 1, 5, 0, 0, 0, 0))
        self.assertFalse(self.ds.OSF())

    def test_real_drivers_observe_per_device_fault_and_recovery(self):
        cases = ((self.relays, lambda: self.pcf.port, lambda: setattr(self.pcf, "port", 0x1234)),
                 (self.inputs, lambda: self.xl.input, lambda: setattr(self.xl, "config", 0)),
                 (self.rtc, lambda: self.ds.datetime(),
                  lambda: self.ds.datetime((2026, 1, 1, 0, 0, 0, 4))))
        for model, read, write in cases:
            before = state(model)
            expected = read()
            model.online = False
            for operation in (read, write):
                try:
                    operation()
                except OSError as error:
                    self.assertEqual(error.args[0], errno.EIO)
                else:
                    self.fail("real driver ignored I2C fault")
            # Other attached devices remain readable; no bus-wide fake outage.
            for other, other_read, _ in cases:
                if other is not model:
                    other_read()
            model.online = True
            self.assertEqual(read(), expected)
            self.assertEqual(state(model), before)


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
