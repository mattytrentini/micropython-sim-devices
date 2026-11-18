# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Matt Trentini
"""Pure digital-model contracts; no hardware or global clock replacement."""
import errno
import unittest
from sim_devices.pcf8575 import PCF8575I2C
from sim_devices.xl9535 import XL9535I2C
from sim_devices.ds3231 import DS3231I2C


def state(model):
    value = model.snapshot()
    del value["reads"]  # Observations may read; they must never write.
    return value


class Bus:
    def __init__(self):
        self.devices = {}

    def add_device(self, device):
        self.devices[device.addr] = device


class Clock:
    def __init__(self):
        self.ms = 0

    def __call__(self):
        return self.ms


class ExpanderTests(unittest.TestCase):
    def test_quasi_bidirectional_latch_and_external_low(self):
        model = PCF8575I2C(Bus())
        self.assertEqual(model.latch, 0xffff)
        self.assertEqual(model.readfrom(2), b"\xff\xff")
        model.inject(0xfffe)
        self.assertEqual(model.readfrom(2), b"\xfe\xff")
        self.assertEqual(model.latch, 0xffff)
        self.assertEqual(model.writeto(b"\xff\x7f"), 2)
        self.assertEqual(model.readfrom(2), b"\xfe\x7f")
        model.inject(0xffff)
        buf = bytearray(2)
        self.assertEqual(model.readfrom_into(buf), None)
        self.assertEqual(buf, b"\xff\x7f")
        self.assertEqual(model.snapshot()["latch"], 0x7fff)
        for operation in (lambda: model.readfrom(1), lambda: model.writeto(b"\x00"),
                          lambda: model.readfrom_into(bytearray(3))):
            with self.assertRaises(ValueError):
                operation()
        with self.assertRaises(NotImplementedError):
            model.writeto_mem(0, b"\x00\x00")
        self.assertEqual(model.latch, 0x7fff)

    def test_xl_register_direction_polarity_and_pair_pointer(self):
        model = XL9535I2C(Bus())
        self.assertEqual(model.config, 0xffff)
        self.assertEqual(model.polarity, 0)
        self.assertEqual(model.output, 0xffff)
        model.inject(0xa55a)
        self.assertEqual(model.readfrom_mem(0, 2), b"\x5a\xa5")
        model.writeto_mem(2, b"\x34\x12")
        model.config = 0xff00
        self.assertEqual(model.readfrom_mem(0, 2), b"\x34\xa5")
        model.polarity = 0x8001
        self.assertEqual(model.readfrom_mem(0, 2), b"\x35\x25")
        self.assertEqual(model.readfrom_mem(2, 2), b"\x34\x12")
        model.writeto(b"\x03\xab\xcd")  # high output, then low output
        self.assertEqual(model.output, 0xabcd)
        model.writeto(b"\x03", stop=False)
        self.assertEqual(model.readfrom(4), b"\xab\xcd\xab\xcd")
        model.writeto_mem(7, b"\x0f")
        self.assertEqual(model.config, 0x0f00)
        with self.assertRaises(ValueError):
            model.writeto_mem(0, b"\xff")
        with self.assertRaises(ValueError):
            model.readfrom_mem(8, 1)
        self.assertEqual(model.output, 0xabcd)

    def test_strict_environment_parameters_and_independent_instances(self):
        for factory in (PCF8575I2C, XL9535I2C):
            first, second = factory(Bus()), factory(Bus())
            first.inject(0)
            self.assertEqual(second.snapshot()["raw_levels"], 0xffff)
            for bad in (True, -1, 65536, 2.5, "0"):
                with self.assertRaises(ValueError):
                    first.inject(bad)
        model = XL9535I2C(Bus())
        for bad in (False, -1, 65536):
            with self.assertRaises(ValueError):
                model.config = bad
            with self.assertRaises(ValueError):
                model.polarity = bad

    def test_offline_transports_fail_without_losing_state(self):
        for factory in (PCF8575I2C, XL9535I2C, DS3231I2C):
            model = factory(Bus(), clock_ms=Clock()) if factory is DS3231I2C else factory(Bus())
            before = model.snapshot()
            model.online = False
            operations = (lambda: model.readfrom(2),
                          lambda: model.readfrom_into(bytearray(2)),
                          lambda: model.writeto(b"\x00\x00"),
                          lambda: model.readfrom_mem(0, 2),
                          lambda: model.readfrom_mem_into(0, bytearray(2)),
                          lambda: model.writeto_mem(0, b"\x00\x00"))
            for operation in operations:
                try:
                    operation()
                except OSError as error:
                    self.assertEqual(error.args[0], errno.EIO)
                else:
                    self.fail("offline transfer succeeded")
            self.assertEqual(model.snapshot()["online"], False)
            model.online = True
            self.assertEqual(model.snapshot(), before)
            self.assertEqual(len(model.readfrom(2)), 2)

    def test_counters_are_bounded_and_pointer_selection_is_not_a_write(self):
        models = (PCF8575I2C(Bus()), XL9535I2C(Bus()),
                  DS3231I2C(Bus(), clock_ms=Clock()))
        for model in models:
            self.assertEqual(model.snapshot()["reads"], 0)
            self.assertEqual(model.snapshot()["writes"], 0)
            model.reads = 0x7ffffffe
            model.writes = 0x7ffffffe
            for _ in range(2):
                if isinstance(model, PCF8575I2C):
                    model.readfrom(2)
                    model.writeto(b"\xff\xff")
                else:
                    before = model.writes
                    model.writeto(b"\x00")
                    self.assertEqual(model.writes, before)
                    model.readfrom(2)
                    if isinstance(model, XL9535I2C):
                        model.writeto_mem(2, b"\xff\xff")
                    else:
                        model.writeto_mem(15, b"\x88")
            self.assertEqual(model.snapshot()["reads"], 0x7fffffff)
            self.assertEqual(model.snapshot()["writes"], 0x7fffffff)


class CalendarTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.model = DS3231I2C(Bus(), datetime=(2024, 2, 28, 23, 59, 59, 3),
                               clock_ms=self.clock)

    def test_bcd_leap_midnight_weekday_and_fractional_ticks(self):
        model = self.model
        self.assertEqual(model.readfrom_mem(0, 7), b"\x59\x59\x23\x03\x28\x02\x24")
        self.clock.ms = 999
        self.assertEqual(model.snapshot()["datetime"], "2024-02-28T23:59:59")
        self.clock.ms = 1000
        self.assertEqual(model.snapshot()["datetime"], "2024-02-29T00:00:00")
        self.assertEqual(model.snapshot()["weekday"], 4)
        self.clock.ms += 86400000
        self.assertEqual(model.snapshot()["datetime"], "2024-03-01T00:00:00")
        self.assertEqual(model.snapshot()["weekday"], 5)
        self.assertTrue(model.snapshot()["osf"])
        model.set_datetime((2023, 2, 28, 23, 59, 59, 2))
        self.clock.ms += 1000
        self.assertEqual(model.snapshot()["datetime"], "2023-03-01T00:00:00")
        model.set_datetime((2026, 12, 31, 23, 59, 59, 4))
        self.clock.ms += 1000
        self.assertEqual(model.readfrom_mem(0, 7), b"\x00\x00\x00\x05\x01\x01\x27")

    def test_status_readonly_queries_and_clock_writes_preserve_osf(self):
        model = self.model
        self.assertEqual(model.readfrom_mem(15, 1), b"\x88")
        model.writeto_mem(14, b"\x04")
        before = state(model)
        for _ in range(4):
            model.readfrom_mem(0, 7)
            model.readfrom_mem(15, 1)
            model.snapshot()
        self.assertEqual(state(model), before)
        model.writeto_mem(0, b"\x58\x59\x23\x07\x31\x12\x26")
        self.assertTrue(model.snapshot()["osf"])
        model.writeto_mem(15, b"\x08")
        self.assertFalse(model.snapshot()["osf"])
        self.assertEqual(model.snapshot()["status"], 8)
        self.assertEqual(model.snapshot()["control"], 4)
        model.writeto_mem(15, b"\x88")  # writing 1 cannot raise OSF
        self.assertFalse(model.snapshot()["osf"])
        model.set_osf(True)
        model.set_datetime((2025, 1, 1, 0, 0, 0, 3))
        self.assertTrue(model.snapshot()["osf"])
        self.clock.ms += 2000
        self.assertEqual(model.snapshot()["datetime"], "2025-01-01T00:00:02")

    def test_twelve_hour_mode_and_partial_pointer_writes(self):
        model = self.model
        model.writeto(b"\x02\x52")  # 12 AM
        self.assertEqual(model.snapshot()["datetime"], "2024-02-28T00:59:59")
        model.writeto_mem(2, b"\x72")  # 12 PM
        self.assertEqual(model.snapshot()["datetime"], "2024-02-28T12:59:59")
        self.clock.ms += 1000
        model.writeto(b"\x02", stop=False)
        self.assertEqual(model.readfrom(1), b"\x61")  # 1 PM, mode retained
        model.writeto_mem(0, b"\x03")
        self.assertEqual(model.readfrom_mem(0, 1), b"\x03")

    def test_only_seconds_writes_reset_subsecond_countdown(self):
        model = self.model
        self.clock.ms = 500
        model.writeto_mem(1, b"\x58")
        self.clock.ms = 1000
        self.assertEqual(model.snapshot()["datetime"], "2024-02-28T23:59:00")
        self.clock.ms = 1500
        model.writeto_mem(0, b"\x00")
        self.clock.ms = 2000
        self.assertEqual(model.readfrom_mem(0, 1), b"\x00")
        self.clock.ms = 2500
        self.assertEqual(model.readfrom_mem(0, 1), b"\x01")

    def test_invalid_dates_and_unsupported_registers_are_explicit(self):
        model = self.model
        before = model.snapshot()
        for bad in ((2023, 2, 29, 0, 0, 0, 3), (2024, 1, 1, 0, 0, 0, 0),
                    (2024, 1, 1, True, 0, 0, 1), (2100, 1, 1, 0, 0, 0, 1)):
            with self.assertRaises(ValueError):
                model.set_datetime(bad)
        for register, payload in ((0, b"\x6a"), (2, b"\x24"), (5, b"\x82")):
            with self.assertRaises(ValueError):
                model.writeto_mem(register, payload)
        self.assertEqual(model.snapshot(), before)
        for register in (7, 11, 16, 17):
            with self.assertRaises(NotImplementedError):
                model.readfrom_mem(register, 1)
            with self.assertRaises(NotImplementedError):
                model.writeto_mem(register, b"\x00")
        with self.assertRaises(NotImplementedError):
            model.writeto_mem(14, b"\x20")
        self.assertEqual(model.snapshot(), before)
        model.set_datetime((2099, 12, 31, 23, 59, 59, 4))
        self.clock.ms += 1000
        with self.assertRaises(OverflowError):
            model.snapshot()


if __name__ == "__main__":
    raise SystemExit(0 if unittest.main().wasSuccessful() else 1)
