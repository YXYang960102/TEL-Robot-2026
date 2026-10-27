# Robot-side ELRS/CRSF bridge

Desk/build verification only (`python3 build.py`). Never connects to or
flashes hardware. `dist/verification.json` keeps `flashed`/`hardware_tested`
hardcoded `false` for this build, matching the convention of the
`stm32_elrs_bidirectional_test` bench (retired 2026-10-14, still in git history). (It has since been
flashed and passed the 2.4 GHz downlink test; see docs/codex-handoff.md.)
The other bench/ground firmwares named below, `stm32_elrs_bidirectional_test`
and `stm32_elrs_ground_bridge`, were both removed on 2026-10-14; mentions of
them here are history.

## Role

One half of the two-board ELRS deployment (the other half is
`tools/stm32_ground_control`). This board sits on the robot, between
the ES900RX radio module and the Mega:

```
ES900RX (CRSF, RF)  <--USART2 PA2/PA3-->  this board  <--USART1 PA9/PA10-->  Mega
```

- **Downlink** (operator's RC channels, arriving over RF at ES900RX): decoded
  from `CRSF_FRAME_RC_CHANNELS` frames on USART2, reformatted as the ASCII
  line `RC,c1,...,c10*HH\n` (`src/ch_line_format.c`; channel meaning in the
  shared table `src/IO/RcControlMap.h`), sent to the Mega on USART1 and
  parsed by the Mega's `src/IO/MechLink.cpp`. (Before 2026-10-11 this was a
  6-field `CH,...` line in `tools/stm32_elrs_common/src/tel_channel_map.h`
  order; that header now only serves the older ground bridge.)
- **Line check and baud** (2026-10-27): USART1 runs at **250000** baud
  (BRR 0x40; the Mega's U2X hits it exactly, where 115200 was +2.1% off on
  the Mega), and both lines end in `*HH`, the CRC-8/DVB-S2 of the text
  before the `*` (`rcmap_line_seal` / `rcmap_line_open` in
  `src/IO/RcControlMap.c`). A line whose check fails is dropped: the Mega
  counts its drops in TEL slot 39 (Dashboard 「丟棄 N 行」, log
  `UART_DROP`). Before this, a digit flipped on the wire still parsed
  (2026-10-26: bogus armed / owner flags and a jumping elevation readback;
  a flipped `0` -> `8` could also read as a fire / arm press). **Flash this
  board and the Mega together**: either one alone drops every line.
- **Uplink, status** (added 2026-10-11): the Mega sends `ST,flags,seq,code*HH\n`
  every 100 ms (`MechLink::sendStatus`). This board packs the newest one into
  a standard CRSF **Battery** frame (`rcmap_status_to_battery`: voltage =
  `0xA5` marker + flags, current = event code, capacity = event sequence)
  and sends it to ES900RX every `STATUS_SEND_PERIOD_MS` (100 ms), only while
  that line is younger than `STATUS_STALE_MS` (500 ms). Battery is a frame
  type ELRS already relays as flight-controller telemetry, so no custom
  frame type has to survive the radio. **Not yet verified on hardware.**
- **Uplink, other lines**: dropped (2026-10-17). This board used to relay
  any non-`ST,` line as `CRSF_TEL_CHUNK` frames every 20 ms, forever, so a
  single garbled line from a Mega reset could keep about 40 custom frames a
  second competing with the status frames for ELRS's telemetry slots. Only
  status goes up now.

USART2/PA2-PA3's role, baud, and bring-up code are unchanged from
`tools/stm32_elrs_bidirectional_test`, which is hardware-confirmed working
over real ELRS RF (both directions; see `docs/codex-handoff.md`). USART1 is
new: that bench rig used PA9 half-duplex to an ES900TX module this board
does not have; here PA9/PA10 are an ordinary full-duplex link to the Mega.

## Verification done so far

- `test/host_test_ch_line_format.c`: native host test for the one piece of
  new pure logic (channel values -> `CH,...` line). `tools/stm32_elrs_common`'s
  own host tests (CRSF framing, chunk split, reassembly) cover the rest of
  what this firmware calls.
- `build.py` cross-compiles, checks the vector table (SysTick/USART1/USART2
  wired, no collisions), checks image bounds, and checks for undefined
  symbols -- the same static checks `stm32_elrs_bidirectional_test` runs.

## Not done

No bench power-up, no ES900RX pairing, no Mega-side wiring test. The
register-level bring-up (clocks, GPIO AF, USART config) is adapted
line-for-line from the hardware-confirmed bench rig for the parts that
carry over (USART2/PA2-PA3), but the new USART1 full-duplex Mega link and
all of this firmware's own logic are unverified beyond the checks above.

## 2026-10-11 changes: fresh board, RF-loss behavior, Mega bench test

**Board / flash address.** The robot-side board is now a fresh WeAct
STM32F401CCU6 with **no WeAct HID bootloader**, so the image links at
`0x08000000` (was `0x08004000`) and is loaded through the chip's ROM USB
DFU bootloader — the same procedure already used successfully on this board
for `tools/stm32_mega_uart_test` (whose 60 s Mega↔STM32 UART run passed on
real hardware over these same PA9/PA10 ↔ D17/D16 wires):

1. STM32 USB-C to the Mac; hold BOOT0, tap NRST, release BOOT0.
2. `dfu-util -l` must show `0483:df11 ... @Internal Flash /0x08000000`.
3. One line, from the repo root:
   ```bash
   cd tools/stm32_elrs_robot_bridge/dist && dfu-util -a 0 -s 0x08000000:leave -D stm32f401cc_elrs_robot_bridge.bin
   ```

**RF loss.** `CH,...` lines are only forwarded to the Mega while a validated
`RC_CHANNELS` frame from ES900RX has arrived within `DOWNLINK_STALE_MS`
(500 ms, `ch_line_should_forward()`, host-tested). Before this change the
bridge sent a line every 20 ms no matter what, repeating stale values, so
the Mega's `MechLink` could never detect an outage (and the wired Dashboard
refused Enable forever with the bridge plugged in). Now the bridge goes
silent on RF loss and `MechLink`'s 200 ms line timeout reports
`signalLost` within ~0.7 s. This assumes ES900RX stops emitting
`RC_CHANNELS` frames when its link drops; that is exactly what the bench
test below checks.

**Bench test with the Mega (downlink only).** Ground side: the old board
(WeAct HID bootloader, 0x08004000) still carries
`tools/stm32_elrs_bidirectional_test`, which sends RC channels every 10 ms
with CH1 cycling 1000 → 1500 → 2000 every 2 s. Wire its ES900TX as in that
project's README (PA9 → JR pin 5 Signal, GND, external VCC — never the
STM32's 3.3 V). Robot side: this firmware + ES900RX on PA2/PA3. Mega:

```bash
pio run -e mega_elrs_link_test -t upload
```
```bash
pio device monitor -e mega_elrs_link_test
```

Pass criteria:
- `$ELRS_EVENT,LINK_OK` appears and `$ELRS,...,link=OK` lines show `fwd=`
  stepping through ~1000 / 1500 / 2000 every 2 s (`fwdChanges` climbing).
- Power the ground STM32 off: `$ELRS_EVENT,LINK_LOST` within ~0.7 s,
  channels return to neutral 1500.
- Power it back on: `$ELRS_EVENT,LINK_OK` again, `fwd=` cycling resumes.

Uplink (Mega `TEL,...` → TEL_CHUNK → ground) is not part of this step: this
Mega test build doesn't send `TEL` lines, and the ground board's old test
firmware doesn't reassemble TEL chunks.

## 2026-10-11 (later): wireless control round

Downlink is now the 9-channel `RC,...` line and the status uplink above was
added (`src/main.c`: the USART1 RX interrupt sorts incoming lines into the
`ST,` status buffer vs. TEL lines; `send_status_frame()`). Host tests in
`test/host_test_ch_line_format.c` cover the RC line, forwarding rule and
`ST` parsing; `build.py` also compiles `src/IO/RcControlMap.c` from the Mega
tree so both ends share one codec.

Full round-trip bench test (no motors): flash this build, flash the ground
board with `tools/stm32_ground_control` (see its README), and upload
`mega_elrs_link_test` to the Mega. It decodes every `RC` frame with the
production codec and sends status back, so the Dashboard connected to the
ground board should show `$EVENT,FEED_TRIGGER_SENT` for each F press — the
first hardware proof that Battery frames make it back over the air.
