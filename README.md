# micropython-sim-devices

Version 0.2.0. MIT-licensed, board-independent digital device models for stock
MicroPython Unix and pure Python. No pinouts, board register maps, hardware access,
image dependencies or replacements for native `framebuf`/fonts/time/GC.
Simulated results do not validate electrical behavior or physical hardware.

## Installation

The root `package.json` is a mip manifest with relative source URLs and the MIT
license. Install from the published repository using:

```python
import mip
mip.install("github:mattytrentini/micropython-sim-devices", version="v0.2.0")
```

Manual installation: copy `sim_devices/` to your library path and retain `LICENSE`.
The models themselves have no dependencies. For real-driver simulation, use
`micropython-mock-machine` 1.3.0, native MicroPython `framebuf`, and the actual
consumer drivers; the models neither bundle nor replace those drivers.

## Modbus RTU

```python
from sim_devices.modbus import ModbusRTUSlave

registers = [0x1234, 0xabcd]

def read_holding(start, count):
    if start + count > len(registers):
        raise IndexError(start)
    return registers[start:start + count]

slave = ModbusRTUSlave(1, read_holding)
uart.set_responder(slave.respond)
```

`respond(frame)` accepts exactly one complete RTU ADU and returns response bytes
or `None`. Supported: unit IDs 1..247, FC03, 1..125 uint16 registers, 16-bit
address space, CRC-16/Modbus. CRC is independently implemented from the protocol;
no external client implementation is included. Wrong-unit/broadcast reads,
invalid request CRCs and frames outside 4..256 bytes receive no response.
A valid addressed unsupported function gets exception 1; a callback `IndexError`
or address-range overflow gets exception 2; malformed FC03 length/quantity gets
exception 3. Callback length/type errors raise locally rather than inventing
registers or hiding programming errors.

Optional `fault(start, count)` returns exactly `normal`, `timeout` or `crc`.
Timeout returns no bytes. CRC faults flip a response CRC bit, leaving data intact,
so the actual client must reject the actual response. Unknown fault names raise.
Snapshots contain only counters and one bounded request metadata record; there
is no growing transaction log. `crc_errors` counts rejected request CRCs plus
injected response CRC faults; `responses` includes protocol exceptions and corrupt
responses, but excludes timeouts. `requests` counts every `respond` invocation.

Not modeled: fragmentation, RTU silent intervals, serial electrical timing,
multiple devices arbitrating a wire, writes, other functions, register semantics,
calibration or any particular meter.

## SSD1306 I2C

```python
from sim_devices.ssd1306 import SSD1306I2C
from ssd1306 import SSD1306_I2C

model = SSD1306I2C(i2c, width=128, height=64)
driver = SSD1306_I2C(128, 64, i2c)  # unchanged official driver
model.on_frame = publish  # no arguments; read model.buffer here
```

The bus must offer `add_device(device)` and dispatch normal `writeto` operations
by `device.addr`. The real driver's `writevto` needs HAL 1.2.0: it must concatenate
segments into **one transaction**, not issue separate control/data writes.
Widths 1..128 and heights 16..64 divisible by 8 are accepted. Narrow panels use
the official driver's centered controller columns. `ram` holds fixed 128x64
controller memory; `buffer` is a stable, in-place MONO_VLSB bytearray of visible
logical pixels. Never replace or resize either buffer.

Supported: Co/D-C control-byte chains; parameter accumulation across transactions;
horizontal, vertical and page addressing; column/page windows and wrap; start
line and display offset; power, inversion, entire-on and contrast; segment/COM
orientation metadata; matching multiplex; clock/precharge/VCOM/COM-pin/charge-pump
configuration metadata; IREF selection including the official driver's AD30;
and documented NOP. The real official driver performs all drawing, text and logo
operations using native `framebuf`, then supplies its raw display bytes.

Output is **driver-logical**, not a physical panel photograph: segment remap and
COM scan direction are tracked but do not flip the published logical image.
Start line/display offset shift RAM rows. Power-off blanks output without losing
RAM; power-on restores it. Inversion and entire-on affect visible bytes. Contrast
and electrical settings are metadata, not analog brightness or scan simulation.
Multiplex values that disagree with configured geometry fail explicitly.

`on_frame()` runs only after a complete I2C transaction, never per byte or command
argument. Data transactions publish once, including repeated unchanged frames;
command transactions publish only if visible bytes change. `frames` counts these
publications even without a callback. Snapshots include geometry, publication
count, power/contrast/inversion/orientation and sum-of-visible-bytes `checksum`.
No RGB conversion, encoded images, retained frame history or external canvas
library is used. Consumers can bind their own publisher after construction.

Unsupported commands (including scrolling), reads and register-memory APIs raise
`NotImplementedError`. Invalid control bytes, windows or parameters raise
`ValueError`. I2C writes are not database transactions: an invalid later byte can
leave earlier bytes applied to controller state, but no partial frame is
published. Command arguments remain bounded, including after invalid commands.
No SPI, reset pin, optics, physical orientation, timing or analog emulation.

## GPIO expanders and RTC

These independent I2C models attach through `bus.add_device(device)`. Board
wiring and active-low channel interpretations belong to the application, not
the library. All accept explicit addresses; there are no KinCony pin constants.

| Model | Implemented digital behavior | Explicit limits |
| --- | --- | --- |
| `sim_devices.pcf8575.PCF8575I2C` | Power-on latch `0xffff`; sampled port = latch AND externally injected levels; little-endian two-byte reads/writes | Exactly two-byte transfers; no register-memory protocol, interrupts or electrical timing |
| `sim_devices.xl9535.XL9535I2C` | Input/output/polarity/direction registers 0–7, paired pointer sequencing; injected input samples and real output-latch behavior | No interrupt or analog model |
| `sim_devices.ds3231.DS3231I2C` | BCD time, native monotonic ticking, leap dates/day rollover, 12/24-hour fields, control/status preservation and OSF | Years 2000–2099; no alarm, temperature, aging, battery or oscillator physics |

`PCF8575I2C.inject(raw_u16)` models externally held-low/released-high samples.
`XL9535I2C.inject(raw_u16)` supplies physical input levels; `.config` and
`.polarity` are explicit environmental register settings. App reads never
repair or reapply them. Relay control-pin samples are not relay-contact feedback.

The RTC constructor accepts
`datetime=(year, month, day, hour, minute, second, weekday)` (Monday=1),
`osf=True`, and optional `clock_ms` for deterministic monotonic-clock tests.
`set_datetime()` initializes environmental time without clearing OSF;
`set_osf(bool)` explicitly changes that flag. The actual driver's writes
still travel through the I2C register protocol. Reads never clear OSF or
set the clock; timestamps are timezone-naive, not browser/host wall time.

All three expose `.online`: false makes transport operations raise `EIO`,
not return fake zero readings. Restoring it preserves controller state.
Snapshots remain available offline and include bounded saturating `reads`
and `writes` counters. Address-pointer selection is not counted as a data write.
Unsupported operations fail explicitly rather than pretending full chip support.

## ST7789 SPI LCD

`sim_devices.st7789.ST7789SPI(spi, cs, dc, reset=None, backlight=None, ...)`
attaches through generic `SPI.add_device()`. Synchronous Pin observers capture
reset/backlight changes separately from scheduled IRQ delivery. No driver, font,
SPI object or Pin is replaced by a high-level fake display.

Configure logical `width`/`height`, `panel_color_order` (`RGB` or `BGR`), and
`native_inversion`. The last two describe panel requirements: matching MADCTL
color order and inversion yield the written RGB565 colors; mismatches swap
red/blue or invert bits. The fixed `ram` and visible `buffer` contain logical,
little-endian RGB565 pixels, suitable for an existing framebuffer publisher.
MADCTL axes are metadata, not a physical mount/rotation photograph.

The model parses real command/data SPI traffic, including fragmented parameters
and odd-byte pixel chunks, address windows/cursor wrapping, reset, sleep/display
power and backlight, and the unchanged driver's initialization/configuration.
Reset preserves GRAM but blanks the view and resets controller state.
Electrical/gamma settings are metadata; scrolling and non-RGB565 formats fail
explicitly. Unselected bus traffic is ignored. `.online=False` raises `EIO`
on selected writes without pretending the panel lost power.

**Coalesce publication:** SPI writes only update bounded RAM and mark `dirty`.
Call `flush()` once per drawing burst or from a bounded async pump; it projects
one visible frame, invokes optional no-argument `on_frame`, and returns whether
it published. Never snapshot a 153,600-byte frame for every tiny font/SPI write.
Bind `on_frame` to an existing publisher; `snapshot()` does not flush/copy pixels.
The original application's response to a fatal SPI error remains its own policy.

## Regression tests

The seven protocol/model tests and two real-driver consumer tests passed on the
stock published MicroPython Unix v1.29.0 image. No custom firmware or hardware
was used. The CPython command below is also provided, but was not run here.

```sh
PYTHONPATH=. python -m unittest discover -s tests -p test_models.py
```

For native MicroPython, supply this repository and `unittest` on `MICROPYPATH`,
then run `micropython tests/test_models.py`. Native consumer regression also
requires HAL 1.2.0 plus unchanged `ssd1306.py` and `umodbus/` on that path:

```sh
micropython tests/test_native_consumers.py
```

Consumer references: official micropython-lib SSD1306 at
`92ee97d94c4b558cf4e6d27a4534401f1e9b9f50` (MIT), and external
micropython-modbus serial driver at `e9ab05a2f1b37796446208a024368725953fc299`
(GPL-noticed source; not distributed by this MIT package). Native tests exercise
real driver initialization including AD30, native logo/text pixels, vectored
show, invert/power/contrast and real client rejection/recovery for timeout/CRC.

The CO16 additions passed another 25 native model/real-driver tests:

```sh
micropython tests/test_co16_i2c.py
micropython tests/test_co16_i2c_consumers.py
micropython tests/test_st7789.py
micropython tests/test_st7789_consumer.py
```

Supply unchanged PCF8575, XL9535, DS3231, `st7789py`, `co16_tft_config` and
`vga1_8x16` modules plus HAL 1.3.0 and `unittest` on `MICROPYPATH`.
The consumer drivers/font are MIT-licensed external dependencies, not bundled
copies. Tests exercise actual register/SPI traffic, all 16 channels, readonly
startup, faults/recovery, ticking leap midnight, ROM-font pixels and color/power
effects. CPython commands are provided where applicable, not claimed run.
