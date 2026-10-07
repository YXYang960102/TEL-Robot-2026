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
