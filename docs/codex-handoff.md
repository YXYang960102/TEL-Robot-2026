# Codex <-> Claude Handoff Log

Shared, append-only log between Codex and Claude Code sessions working on this repo.
Read the whole file before starting new work. Never edit or delete an earlier entry.

---

## 2026-08-17 — Claude Code

**User Request:** Jeremy shared a CAD screenshot (Autodesk Inventor, part "shooter 06",
`/Users/jeremy/Downloads/IMG_0821.DNG`) of the shooter hood elevation mechanism and said:
the AS5600 currently being bench-tested (`src/Sensors/AS5600Encoder`, `src/Actuators/ContinuousServoMotor`,
`src/Bench/As5600ServoTestSubsystem`, PlatformIO env `uno_as5600_servo_test`) is expected to be the
elevation-angle encoder for the shooter hood. There will be **two identical MG996 continuous servos**,
one per side of the hood (large arc gear + pinion visible in the CAD), and mechanically one side's
rotation output may be reversed relative to the other due to mirrored mounting.

**Discussion Result:** Not yet decided. Presented three options for how the two AS5600+servo channels
relate mechanically/electrically:
1. Two independent closed loops, one side's `outputSign`/encoder-delta direction inverted, same target angle after each side's own zero.
2. Mechanically rigid same-axis coupling — only one AS5600 is the real feedback source, the other servo is open-loop/mirrored-follower with no independent closed loop.
3. Undecided — record context only, no code yet.
Jeremy chose option 3: discuss/record only, do not write dual-servo code this session.

**Why:** Mechanical design (whether the two sides are rigidly coupled on one shaft vs. independently
loaded) isn't finalized yet, and that choice determines the control architecture (one PID vs. two,
whether the second AS5600 is authoritative or redundant/diagnostic only).

**Changed / Added:** No code changes this session. This log file was created.

**Calculation:** N/A — no control code touched.

**Impact:** `src/Bench/As5600ServoTestSubsystem` remains single-axis, UNO-only
(`uno_as5600_servo_test` env excludes `Bench/`, `Sensors/`, `Actuators/` from the `megaatmega2560`
build, so this has no effect on the main robot firmware yet). Existing `src/Shooter` /
`ShooterConst` VER control (potentiometer-based, `PIN_POT`, `VER_MIN`/`VER_MAX`) is untouched.

**Evidence:** None run — no code changed.

**Next Test:** Once the mechanical coupling question above is resolved, extend the bench harness to
two AS5600 + two servo channels on the UNO test rig (still staged before touching the Mega
`src/Shooter` build) before any real hood integration. Confirm zero-point calibration strategy
(mechanical hardstop vs. boot-relative zero) at the same time, since `AS5600Encoder::zero()`
currently zeros at whatever position it powers on in, which is fine for the bench PID test but not
for a real elevation reference.

---

## 2026-08-18 — Claude Code

**User Request:** Jeremy asked whether Claude knew this repo (`TEL_Robot_2026`, branch
`ShooterMG996`) and `/Users/jeremy/Desktop/RobotAI/YOLO_Detect_single` (branch `main`) are part of
the same system: an Orin Nano running YOLO detection, not yet pulling hole/goal depth, packaging
final output into a packet sent to the Arduino Mega — currently being bench-tested on a UNO first.

**Discussion Result:** Confirmed by reading both repos' actual code (not from memory). The two
projects are already wired together end to end:
- `YOLO_Detect_single/vision_main.py` computes `tx, ty, distance, target_id, valid`; `distance`
  comes from `depth_distance.py`'s `DepthEstimator` only when `camera_frame.depth_mm is not None`
  (a real RGBD depth stream), otherwise `distance = 0`. `test_rgbd_camera.py` in that repo looks
  like the in-progress work to wire up that depth stream — matches Jeremy's "haven't pulled the
  hole depth down yet."
- `serial_tx.py` sends `f"{tx},{ty},{distance},{target_id},{valid}\n"` at `SERIAL_BAUD = 115200`.
- `TEL_Robot_2026/src/Vision/Vision.cpp` reads `Serial1` at `115200` baud and parses the exact same
  5-field CSV format into `tx/ty/distance/targetId/valid`. This module is already included in the
  `megaatmega2560` build (not excluded like `Bench/`/`Sensors/`/`Actuators/`).

**Why:** Confirms the vision packet pipeline (Jetson -> Mega) is a separate, already-integrated path
from the AS5600 dual-servo elevation work above — they are two independent staged efforts that will
eventually both feed the shooter subsystem (vision aiming + AS5600 elevation feedback), not one
blocking the other.

**Changed / Added:** No code changes. Documented the cross-repo data flow here so future sessions
don't need to re-derive it by reading both repos again.

**Calculation:** N/A.

**Impact:** No files changed in either repo.

**Evidence:** Read `src/Vision/Vision.h/.cpp`, `src/Constants/VisionConstants.h` in this repo, and
`vision_main.py`, `serial_tx.py`, `depth_distance.py`, `config.py` in `YOLO_Detect_single` (git
status clean there, `main` branch, latest commit `ea3db01d Add hole-depth fallback for distance
estimation`).

**Next Test:** Once the RGBD depth stream is actually validated in `YOLO_Detect_single`
(`test_rgbd_camera.py`), do an end-to-end serial bench test: run `vision_main.py` with `--serial`
enabled against the Mega running `src/Vision`, confirm `Vision::isValid()` tracks real
target-lost/found transitions (it depends on `PACKET_TIMEOUT_MS = 300` in `VisionConstants.h`) and
that `distance` reflects real depth instead of the `0` fallback.

---

## 2026-08-18 — Claude Code

**User Request:** Jeremy shared the team's briefing deck (`東京威力2026-簡報.pdf`) and said the
`Chassis` branch owns chassis control + vision integration. Current hardware per the deck: front 2
wheels are Mecanum (795 planetary-gearbox brushed motors, 3" wheels), rear 2 are passive/unpowered
casters ("果凍輪"), driven with 火鳳凰 "New Rain" 480A brushed ESCs, 11.1V 5200mAh LiPo, ELRS 2.4G
receiver — and the deck's "機器人移動控制" slide explicitly labels the drive logic as
"坦克車式駕駛邏輯" (tank/differential drive: 2 independent sides, forward/back only, no strafe).
Jeremy confirmed the 5V-Mega-TX to 3.3V-Jetson-RX voltage risk from the prior entry is understood and
accepted, then asked to update the chassis code to match the deck and to do it on its own branch.

**Discussion Result:** Reading `src/Chassis/Chassis.cpp` (as it stood in `ShooterMG996` at commit
`90f93e6`) showed it still implemented full 4-wheel mecanum kinematics (`vx,vy,w -> FR/FL/BR/BL`,
4 independent Servo/ESC outputs on `PIN_FR`/`PIN_FL`/`PIN_BR`/`PIN_BL`), which no longer matches the
finalized 2-motor tank-drive hardware — mixing math assumed holonomic strafing the rear casters can't
provide. Separately, `src/IO/SBUS.cpp`'s yaw channel (`ch3`) was mapped to `500..-500` (centered at
0) while every other channel (`ch0`,`ch1`,`ch2`,`ch8`) is centered at `1500`; `Chassis.cpp` uniformly
did `(SBUS::chN - 1500) / 500.0` for all of them, so `ch3` got double-offset — at stick-neutral,
`w` computed to `-3.0` instead of `0`, producing a strong rotation bias even when the transmitter's
yaw stick was centered. Jeremy agreed to fix both. First attempt put the fix on a brand-new branch
(`chassis-tank-drive-fix`, cut from `ShooterMG996`); Jeremy then asked to use the existing `Chassis`
branch instead and keep the AS5600 bench work only on `ShooterMG996` — too many branches otherwise.
Moved the edit over with `git stash push -- <the 3 files>` (path-scoped, left the AS5600/docs
untracked files untouched), checked out `ShooterMG996`, deleted the now-empty
`chassis-tank-drive-fix` branch, checked out `Chassis`, and popped the stash there. Confirmed first
that `Chassis` (`231485d`) and `ShooterMG996` (`90f93e6`) have identical committed content for every
touched file (`Chassis.h/.cpp`, `SBUS.cpp`, plus `.vscode/arduino.json`/`platformio.ini`/`.DS_Store`
that were already dirty in the working tree), so no merge/conflict risk in moving branches.

**Why:** The mechanical design in the briefing deck (rear passive casters, explicit tank-drive
slide) supersedes the older 4-wheel mecanum-mix code, which predates that decision. The `ch3`
double-offset was a plain calculation bug, not a design choice. The branch choice is Jeremy's
preference to avoid branch sprawl — `Chassis` already exists and owns chassis work; a fresh
per-fix branch wasn't warranted here.

**Changed / Added:** Edits are on branch `Chassis` (`231485d`, currently checked out), still
uncommitted:
- `src/Chassis/Chassis.h` / `.cpp`: dropped the `br`/`bl` `Servo` members and their pin attach calls
  (no motor on the rear casters); dropped the `vy` (strafe) term entirely; mixing is now
  `FR = vx - w`, `FL = vx + w` (2 outputs only, on `PIN_FR`/`PIN_FL`).
- `src/IO/SBUS.cpp`: `ch3` remapped from `map(data.ch[1],170,1820,500,-500)` to
  `map(data.ch[1],170,1820,2000,1000)` — same reversed polarity, now centered at `1500` like every
  other channel, so `Chassis.cpp`'s existing `(SBUS::ch3 - 1500) / 500.0` is correct without
  further changes.
- `src/Constants/Pins.h` was left untouched — `PIN_BR`/`PIN_BL` still defined but now unused by any
  code.

**Calculation:** `vx = (ch0 - 1500) / 500` (range ~-1..1, ch0 spans 1000-2000). `w = (ch3 - 1500) /
500` (now range ~-1..1, ch3 spans 1000-2000 after the fix). `FR = vx - w`, `FL = vx + w`, then both
divided by `max(|FR|,|FL|)` if that exceeds 1, then `1500 + value*500` written as the servo/ESC pulse
in microseconds (1500 = stop, matching the 795-motor ESC convention used elsewhere in this repo).

**Impact:** Only `Chassis` module behavior and the meaning of `SBUS::ch3` changed. Checked every
other consumer first: `ch3` is only read in `Chassis.cpp` (now fixed) and printed raw for debug in
`src/Telemetry/Telemetry.cpp` (no arithmetic there, unaffected by the centering change). `ch1`
(former strafe input) is no longer read anywhere.

**Evidence:** Code review and `git diff` only. `pio` is not installed on this machine, so the
`megaatmega2560` build was not compiled or flashed. No bench or field test run — do not treat this as
verified beyond static review.

**Next Test:** Compile-check `pio run -e megaatmega2560` first. Then bench-verify with the chassis
ESCs powered but wheels off the ground / robot on blocks: (1) stick neutral -> both `FL`/`FR` pulses
sit at 1500us (stopped), (2) forward-only input -> both wheels spin the same direction at matching
speed, (3) yaw-only input -> wheels spin opposite directions at equal speed (in-place rotation).
Only move to a free-rolling test after those three hold.
