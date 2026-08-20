# Codex <-> Claude Handoff Log

Shared, append-only log for TEL_Robot_2026. Read the complete file before
project work and append new entries chronologically.

---

## 2026-08-20 - Codex: fixed-speed feed and beam-break counting

**User Request:** Jeremy assigned feeding and storage-tower behavior to the
`Dribbler` branch and asked for fixed-speed ball feeding with beam-break
detection like the legacy firmware.

**Discussion Result:** A requested shot runs the existing feed outputs at one
configured speed. A ball is counted only after the exit beam is stably blocked
and then stably clears. YOLO does not decrement or invent the physical count;
the Arduino event is the authoritative confirmation that one ball passed.

**Why:** The previous modular code read output pin 51 as if it were a sensor and
decremented on every loop while LOW. Separating output pin 51 from legacy exit
sensor pin 45 and using a complete blocked-to-clear event prevents repeated
counts from one ball or a held sensor.

**Changed:** `Dribbler` now stores `shootRemaining`, reports feeding/sensor/event
state, drives pin 2 with a fixed PWM while a request remains, controls the gate
on pin 51, debounces the exit sensor on pin 45, and stops automatically when the
requested count reaches zero.

**Added:** `DribblerConstants.h` contains feed PWM 180, pin polarity, 8 ms
debounce, and request bounds `0..12`. `PIN_DRIBBLE_EXIT_SENSOR=45` restores the
legacy top beam-break input without reusing an actuator output.

**Calculation:** One confirmed event is `stableBlocked: true -> false` after at
least 8 ms at each raw transition. Example: request 3, beam blocks, then clears:
remaining changes `3 -> 2` and completed count `0 -> 1`. A beam held LOW for
100 updates remains one incomplete event and does not decrement repeatedly.

**Impact:** Storage-tower multi-motor sequencing, ultrasonic behavior, YOLO
return transport, and automatic shot requests remain future work. PWM 180 and
sensor polarity are architecture defaults, not measured hardware calibration.

**Evidence:** `platformio run -e megaatmega2560` succeeded: RAM 1038/8192 bytes
(12.7%), flash 13050/253952 bytes (5.1%). No firmware upload, sensor fixture,
motor power, or real-ball test was performed.

**Next Test:** With motor power disabled, monitor pin 45 while manually blocking
and clearing the sensor and confirm exactly one count per passage. Then power
the feed mechanism with an immediate stop path and tune `FEED_PWM` from a low
safe value while confirming pin 51 polarity before moving a real ball.
