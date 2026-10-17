# Ground-station STM32: wireless robot control

The PC Dashboard (`tools/dashboard/index.html`) plugs into **this board's
USB-C** instead of the Mega's. The board turns the Dashboard's keyboard
commands into ELRS RC channels, and turns the robot's status coming back
over the air into lines the Dashboard shows.

```
PC Dashboard --USB CDC--> this STM32 --CRSF PA9 (half-duplex, 400000)--> ELRS TX
     ))) 2.4 GHz )))
ELRS RX --CRSF PA2/PA3--> robot STM32 bridge --"RC,..." PA9/PA10--> Mega Serial2
Mega --"ST,..." --> robot bridge --CRSF Battery frame--> ELRS RX ))) TX --> this STM32 --> PC
```

Hardware status: **builds and host tests only. It has never been flashed or run on a board.**

## Files

- `src/GroundCommandState.{h,cpp}`: pure logic with no Arduino calls, host-tested.
  - `GroundCommandState` parses the PC lines.
  - `RobotStatusTracker` turns status frames into PC lines.
- `src/main.cpp`: USB serial, the CRSF UART, 100 Hz RC frames, Battery and
  link-statistics frame parsing, and the PC13 LED.
- `src/shared_sources.c`: compiles `tools/stm32_elrs_common/src/crsf.c` and the
  Mega tree's `src/IO/RcControlMap.c` into this build. The ground and the
  robot share one channel codec.
- `test/host_test_ground_state.cpp`: the host test.

## Line protocol (PC ↔ this board)

The PC sends exactly what it sends to the Mega over a wired link:

| Line | Effect here |
|------|-------------|
| `E` | Arm channel (CH6) goes high, every axis goes neutral, and the target hole goes back to 0 (the AI's own pick). Prints `$GROUND_EVENT,ENABLE_REQUESTED`. The robot enables on the low→high edge. |
| `X` | Arm channel goes low and every axis goes neutral. Prints `$GROUND_EVENT,ENABLE_RELEASED`. |
| `K` | Heartbeat. |
| `T,<l>,<r>` | Becomes forward = (l+r)/2 on CH1 and turn = (r−l)/2 on CH2 (the robot mixes them back with the same polarity). |
| `ROT,<d>` / `ANG,<d>` | The nudge channel (CH7): left/right or up/down while non-zero, center at 0. If both are held, elevation wins. |
| `F` | Fire (CH5) goes high for 100 ms. The robot acts on the low→high edge, so one press is one shot. |
| `TGT,<0..9>` | Target hole on CH3 (1 left-bottom … 9 right-top, 0 = the AI's own pick). It is held after the key is released, until the next `E`. |
| `AUTO,<1..4>` | Auto path on CH8 for 200 ms, then back to none. A press during the 200 ms is ignored. The robot allows one path per Enable. The Dashboard sends it right after Enable from its path menu. |
| `MODE,<0..2>` | Operator mode on CH4, straight to that level (0 full auto, 1 semi auto, 2 full manual). The Dashboard sends it right after Enable from its mode menu. |
| `FLY,<0..1>` / `ANGT,<deg>` | Accepted, so the link stays alive, but sent nowhere: table v2 has no wireless flywheel channel or absolute elevation channel. |

All of these are accepted only while enabled; otherwise the board prints `REJECTED_NOT_ENABLED`. `R` and `L` were retired with table v2 and are now `UNKNOWN`.
| `M` | Cycles the operator mode on CH4: 2000 full manual → 1750 semi auto → 1500 full auto → back. Prints `$GROUND_EVENT,MODE_<name>`. `E`, `X` and a PC timeout put CH4 back to full manual (the robot refuses an Enable whose CH4 isn't manual). |

Safety rules:
- If no line arrives for 250 ms while enabled, the board releases the arm channel and prints `$GROUND_EVENT,PC_TIMEOUT`.
- A line too long for the buffer also releases (`COMMAND_TOO_LONG`).
- If this board loses power, or the TX loses the link, the robot's ELRS RX
  stops sending frames. The Mega reports `DISABLED_RC_LINK_LOST` within about 0.7 s.

The board sends to the PC:
- `$EVENT,<name>`: a robot event (the same names the Mega prints over USB), relayed through the status uplink.
- `$WIRELESS,<fresh>,<armed>,<owner 0/1/2>,<pins>,<feedBusy>,<rotateManual>,<rcLink>,<ageMs>,<mode>`: every 250 ms. `mode` is the robot's operator mode (0 full auto, 1 semi auto, 2 full manual, -1 not reported by older robot firmware), carried in the top bits of the status event code.
- `$RFLINK,<fresh>,<uplinkLQ>,<downlinkLQ>,<rssi>`: every 250 ms, from the TX module's link statistics.
- `$AIM,<fresh>,<distanceCm>,<elevationPulseUs>,<selectedId>,<targetId>,<valid>,<aimOffsetPx>`: every 250 ms, from the robot's aim frames (2026-10-17). The Mega sends status, status, aim in turn, one per 100 ms, through the same Battery frame, and the robot bridge forwards them unchanged. ELRS only sends a few of them back: the default telemetry ratio is about 1:64. This gives the Dashboard the camera distance, the elevation pulse and the lock state over the radio. `fresh` is 0, with zeros, when no aim frame arrived for 3 s, for example with older robot firmware. `$WIRELESS` also uses 3 s, because on the 2026-10-17 bench status frames arrived up to about 1 s apart. Raising the ELRS Telem Ratio (to 1:8 or 1:4) makes both faster.

Channel table: `src/IO/RcControlMap.h`.

CH4 starts at 2000 (FULL_MANUAL) and only `M` changes it. The old `stm32_elrs_bidirectional_test` firmware (removed from the repo 2026-10-14) sent 1000 on CH4, which production reads as FULL_AUTO. **If a ground board still has that old firmware on it, reflash it with this one before running the robot firmware.**

## Flashing (Jeremy runs these; Claude never flashes)

This is the **old board**, the one with the WeAct HID bootloader that carried the retired `stm32_elrs_bidirectional_test` bench.

Flashing through ROM DFU at 0x08000000 overwrites the HID bootloader. That is fine: from now on this board is always loaded through ROM DFU (BOOT0), the same way as the robot board.

1. Plug the STM32's USB-C into the Mac. Hold BOOT0, tap NRST, then release BOOT0.
2. Check that `dfu-util -l` shows `0483:df11 ... @Internal Flash /0x08000000`.
3. Flash. Run the command as a single line:
   ```bash
   cd /Users/jeremy/Desktop/FRC8169/TEL_Robot_2026/tools/stm32_ground_control && ~/.platformio/penv/bin/pio run -t upload
   ```
   or, using the built image:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D /Users/jeremy/Desktop/FRC8169/TEL_Robot_2026/tools/stm32_ground_control/.pio/build/blackpill_f401cc/firmware.bin
   ```
4. Tap NRST. The Mac should now show a new serial port (`/dev/cu.usbmodem...`). Select that port in the Dashboard.

## Wiring (same as the current bench rig)

| STM32 | ELRS TX module | Note |
|-------|----------------|------|
| PA9 | JR/CRSF Signal | Single-wire half-duplex. Put a 1 kΩ resistor in series. |
| GND | GND | Common ground. |
| (external supply) | VCC | **Never** power it from the STM32's 3.3 V. |

## Test order (wheels off the ground)

1. **Round trip with no motors.** Run Mega `mega_elrs_link_test` with this board.
   - Enable on the Dashboard should complete.
   - Each fire press (the gamepad's fire button) should log `FEED_TRIGGER_SENT`.
   - Each Q/W/E/R press should log `AUTO_PATH_STARTED`, and keys 1–9 should log `TARGET_SELECTED`.
   - `$RFLINK` should show an LQ near 100.
   - Turning this board off should make the Mega print `LINK_LOST`.

   This is the first hardware check that Battery-frame status actually comes back over the air. If no `$EVENT` appears and `$WIRELESS` stays at `0,...`, the uplink is the problem. Enable cannot complete without it: the Dashboard waits 3 s, then sends X.
2. **Production Mega firmware.** Run it only after the pin table is confirmed (`PIN_TABLE_CONFIRMED`); until then the robot refuses Enable with `ENABLE_REJECTED_PINS_UNCONFIRMED`.
   - Test at low speed first: chassis direction (stick), each stop key, fire, the arrows.
   - Then turn this board off while moving and close the Dashboard page while moving. The robot must stop each time.
