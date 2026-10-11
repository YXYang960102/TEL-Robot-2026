# Robot-side ELRS/CRSF bridge

Desk/build verification only (`python3 build.py`). Never connects to or
flashes hardware. `dist/verification.json` keeps `flashed`/`hardware_tested`
hardcoded `false` for this build, matching `tools/stm32_elrs_bidirectional_test`'s
convention -- this firmware has not been loaded onto a board.

## Role

One half of the two-board ELRS deployment (the other half is
`tools/stm32_elrs_ground_bridge`). This board sits on the robot, between
the ES900RX radio module and the Mega:

```
ES900RX (CRSF, RF)  <--USART2 PA2/PA3-->  this board  <--USART1 PA9/PA10-->  Mega
```

- **Downlink** (operator's RC channels, arriving over RF at ES900RX): decoded
  from `CRSF_FRAME_RC_CHANNELS` frames on USART2, reformatted as the ASCII
  line `CH,fwd,turn,mech,aux,fire,mode\n` (`src/ch_line_format.c`, channel
  order from `tools/stm32_elrs_common/src/tel_channel_map.h`), sent to the
  Mega on USART1. This replaces the channel values the Mega used to read
  directly off SBUS -- see the Mega's `src/IO/MechLink.h/.cpp`.
- **Uplink** (the Mega's `TEL,...` dashboard telemetry line, read back on
  USART1): split into `CRSF_TEL_CHUNK_DATA_MAX`-byte pieces
  (`tools/stm32_elrs_common/src/tel_chunk_split.c`) and sent as
  `CRSF_FRAME_TEL_CHUNK` frames on USART2 for ES900RX to relay back over RF.
  The ground bridge reassembles them (`tel_reassembler.c`) back into the
  original line for the dashboard.

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
