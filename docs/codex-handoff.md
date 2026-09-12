# Codex <-> Claude Handoff Log

Shared, append-only log for TEL_Robot_2026. Read the complete file before
project work and append new entries chronologically.

---

## 2026-08-20 - Codex: Chassis and YOLO transport foundation

**User Request:** Jeremy assigned chassis and Vision work to the `Chassis`
branch and asked Codex to combine the current two-wheel tank control with the
actual `YOLO_Detect_single` serial transport before moving on to Dribbler and
Shooter.

**Discussion Result:** Keep Jetson as a fact producer and Mega as the motor
controller. Vision accepts only the exact five-field packet
`tx,ty,distance_mm,target_id,valid`; chassis manual input remains SBUS, but now
uses a command API that a future autonomous controller can also call. Invalid,
malformed, stale Vision data is neutralized. SBUS failsafe, lost-frame, or
100 ms silence commands a chassis stop.

**Why:** YOLO must not directly own motors, stale input must not leave the last
motor command active, and future autonomous control needs one bounded chassis
command interface instead of reaching into Servo PWM values.

**Changed:** `Vision` now validates exactly five numeric fields, range checks
them, tracks packet freshness separately from target validity, and zeroes target
facts after invalid/malformed/stale input. `SBUS` now exposes health, frame age,
and normalized forward/turn commands and neutralizes channels on failure.
`Chassis` now consumes bounded `[-1,1]` commands and provides `stop()`.
`main.cpp` routes healthy SBUS commands into Chassis and stops otherwise.

**Added:** `ChassisConstants.h` stores ESC pulses and command deadband;
`SBUSConstants.h` stores receiver ranges and the 100 ms timeout.

**Calculation:** `forward=(ch0-1500)/500`, `turn=(ch3-1500)/500`.
Tank mixing is `right=forward-turn`, `left=forward+turn`; if either magnitude
exceeds 1, both are divided by the largest magnitude. Output is
`1500 + command*500 us`, clamped to `1000..2000 us`. Example: forward `0.6`,
turn `0.2` gives right `0.4 -> 1700 us` and left `0.8 -> 1900 us`.

**Impact:** Dashboard telemetry schema is unchanged. Shooter, Dribbler,
ShooterMG996, YOLO Python code, and autonomous movement behavior are unchanged.

**Evidence:** `platformio run -e megaatmega2560` succeeded: RAM 1076/8192 bytes
(13.1%), flash 16788/253952 bytes (6.6%). No firmware upload, serial loopback,
receiver bench test, or energized chassis test was performed.

**Next Test:** With wheels raised and an immediate power-disable path, verify
neutral SBUS produces 1500 us on both drive outputs, forward commands equal
wheel direction, turn commands opposite wheel direction, and disconnecting the
receiver stops both outputs within 100 ms. Separately send valid, invalid,
malformed, and paused YOLO packets with actuators disabled and inspect telemetry.

---

## 2026-08-22 — Claude Code

**User Request:** Jeremy approved Codex's Orin/Mega power-gated startup
proposal (shared inbox) and said implement it; Claude owns the Mega side.
Then, after Codex's read-only mutual review (`進行審核`), Jeremy authorized
fixing every finding.

**Discussion Result / Changed, in commit order:**
1. `4920882`: added the Mega->Orin heartbeat and Orin lifecycle-line parsing
   to `Vision`, against a self-proposed wire format that turned out to be
   wrong.
2. `b440f24`: corrected that format to match Codex's already-implemented and
   unit-tested Orin side exactly - `MEGA_READY,1` once then
   `MEGA_HEARTBEAT,1` every 100ms; `VISION_STANDBY/STARTING/READY/ERROR,1`
   parsed via a `VISION_` prefix check; added `Vision::isVisionReady()`.
3. `fcc6ca5`: Codex's review found `parseOrinControl()` checked only the
   prefix and never validated the version field, so `VISION_READY,999` or
   even `VISION_READY,` (empty) would both be accepted as a fresh READY.
   Fixed to parse and require the version field equal `PROTOCOL_VERSION`,
   still tolerating extra trailing fields after a valid version (matching
   Orin's own tolerance).
4. `b8c39d9`: Codex's review found `Shooter::update()`'s vertical PID ran
   unconditionally every loop regardless of any readiness state - `setV` is
   hardcoded to `2000`, unreachable by the `analogRead(PIN_POT)` 0-1023
   range, `Kp2=0.45` (`src/Constants/PIDConfig.h`), and no
   `SetOutputLimits()` call exists anywhere in this repo (grepped to
   confirm), so PID_v1's default 0-255 output ceiling saturates
   immediately: `0.45 * (2000-1023) = 439.65 > 255`, so `escV` gets
   `1500 + 255 = 1755us` from the very first `loop()` iteration after boot,
   independent of SBUS health or vision validity. Gated the whole function
   on `Vision::isVisionReady()`: neutral (1500us) on `escH`/`escV`/`falcon`
   and `readyH`/`readyV=false` until a fresh `VISION_READY` is seen.

**Why:** The heartbeat/lifecycle work is the Claude-owned half of the
approved Orin power-gate proposal (Codex: Orin/YOLO_Detect_single side;
Claude: TEL/Mega side). The two fixes in items 3-4 came directly out of
Codex's independent read-only review of the actual branch objects, not
self-review - re-verified both findings against source (`PIDConfig.h`,
`grep -rn SetOutputLimits`) before fixing rather than taking the review at
face value.

**Calculation:** See item 4 above for the exact saturation math. Heartbeat
timing: Orin's incoming-Mega timeout is 1000ms and its own status/heartbeat
cadence is 100ms; Mega's `HEARTBEAT_INTERVAL_MS=100` matches that cadence,
giving roughly 10 heartbeat slots inside Orin's timeout window as margin.
Vision-state staleness reuses the existing `PACKET_TIMEOUT_MS=300` (already
used for numeric-target staleness) for lifecycle-line staleness too.

**Impact:** `Vision` and `Shooter` changed. Numeric target parsing (5-field,
range-checked, `PACKET_TIMEOUT_MS`), `Chassis`/`SBUS` failsafe behavior, and
`Dribbler` are unaffected by this entry's changes (Dribbler's own port of
this same work is a separate commit sequence on the `Dribbler` branch, see
that branch's own handoff entry).

**Evidence:** `avr-g++ -fsyntax-only` against every `.cpp` under `src/`
(excluding `Bench/Sensors/Actuators`), using the exact flags/include paths
from this machine's cached `.pio/build/megaatmega2560/idedata.json` - all
compile clean after every commit in this sequence, re-run after each fix, not
just once at the end. No `pio` link, no firmware upload, no hardware test of
any kind - the Shooter fix in particular has never been bench-verified with
a multimeter or servo tester, only reasoned about from source.

**Next Test:** Before connecting the real shooter mechanism, bench-verify
with the ESC/servo signal wires disconnected from any load: power the Mega
alone and confirm (with a servo tester or oscilloscope on `escH`/`escV`/
`falcon`) that all three sit at exactly 1500us while no `VISION_READY` line
is being sent, then feed a simulated `VISION_READY,1` over `Serial1` (e.g.
from a USB-serial adapter) and confirm the vertical PID only then starts
moving `escV` away from neutral. Only after that holds, proceed to the
existing Chassis/SBUS bench test sequence above and eventually a real
Orin<->Mega link test.

---

## 2026-08-24 - Codex: Chassis, Vision, and IO readability refactor

**User Request:** After synchronizing formal branches, refactor one owning
branch at a time using mechanism-owned Constants and visible subsystem APIs.
This entry covers only the formal `Chassis` branch and preserves the existing
two powered front wheels plus passive rear caster architecture.

**Constants:** `ChassisConstants.h` now groups right/left signal pins and
inversion, output PWM, and manual command bounds/deadband. The old
`SBUSConstants.h` became `IOConstants.h`, which owns receiver channel indices,
raw ranges, mapped pulse ranges, auxiliary range, and frame timeout.
`VisionConstants.h` now groups `Transport`, `Validation`, and visibly isolated
`Legacy` prediction values. Duplicate Chassis pin macros were removed from
`Pins.h` after migration.

**Chassis API:** `Chassis.h` visibly separates lifecycle, manual open-loop, and
telemetry. `setDriveCommand()` was renamed `setOpenLoop(forward, turn)` and
`main.cpp` was updated. Tank mixing remains `right=forward-turn` and
`left=forward+turn`, followed by common normalization. New getters expose
bounded commands and actual PWM values without exposing Servo objects.

**IO API:** SBUS channel storage is private. Callers now use named command and
telemetry getters instead of public `ch0/ch1/ch2/ch3/ch8` globals. The current
firmware mapping remains exact: receiver channel 0 drives forward, channel 1
is inverted for turn, channel 2 is the mechanism input, channel 3 is the
auxiliary input, and channel 8 is mode. The `TEL` telemetry packet retains the
same five values in the same order, so the Dashboard wire schema is unchanged.
Failsafe, lost-frame, and 100 ms stale-frame neutralization are unchanged.

**Vision API:** Public methods are grouped as lifecycle, YOLO target facts,
and transport/Orin status. `getXPred()` remains as a documented compatibility
alias for `tx`. The exact five-field packet, numeric/range validation,
`MEGA_READY,1`, 100 ms `MEGA_HEARTBEAT,1`, strict lifecycle version checking,
300 ms staleness, and invalid-target clearing are unchanged.

**Evidence:** `git diff --check` passes. PlatformIO `megaatmega2560` release
build succeeds: RAM `1205/8192` bytes (14.7%), Flash `17960/253952` bytes
(7.1%). No firmware upload, UART loopback, receiver test, PWM measurement, or
powered chassis test was performed.

**Next:** Commit and push this refactor separately on `Chassis`, request Claude
review, then switch the normal working tree to `Dribbler`. Do not merge into
`dev` before subsystem verification.

---

## 2026-08-25 - Codex: Chassis keyboard bench control

**User Request:** Add an isolated Mac keyboard test for the two-track chassis.
Mode 2 is the default arcade mapping (`W/S` forward/reverse, `A/D` left/right
turn). Mode 1 is direct tank control (`Q/A` left forward/reverse, `E/D` right
forward/reverse). The page must show all key hints and remain separate from the
existing robot dashboard.

**Architecture:** `Chassis` now exposes an output-enable gate, direct left/right
open-loop control, and left/right command telemetry. Arcade mixing is isolated
in `DifferentialDriveMixer.h`, which normalizes `left=forward+turn` and
`right=forward-turn`. The formal `main.cpp` explicitly enables chassis outputs
only while SBUS is healthy and disables them on receiver failure, preserving the
existing formal behavior while making the safety state visible.

**Bench Build:** Added PlatformIO environment `mega_chassis_keyboard_test`. It
builds only `Bench/ChassisKeyboardTest.cpp` and `Chassis/`; the normal
`megaatmega2560` environment excludes all `Bench/` sources. Bench constants cap
manual output at 10%, send telemetry every 100 ms, and disable outputs if no
valid command or heartbeat arrives for 250 ms. Boot, malformed/unknown/oversize
commands, explicit stop, and timeout all leave outputs disabled at 1500 us.

**Test Page:** `tools/chassis_keyboard_test/index.html` uses Web Serial at
115200 baud. It provides connect, enable, stop, mode buttons, bilingual key
hints, command/PWM telemetry, and serial events. `Enter` disables while keeping
the connection; Space and Escape disable then disconnect. It sends neutral on
focus loss and page hide.
The existing dashboard layout and files were not changed.

**Evidence:** `git diff --check`, native differential mixer tests, and dashboard
JavaScript syntax validation pass. PlatformIO release builds pass for both
`mega_chassis_keyboard_test` (RAM 567/8192, Flash 9312/253952) and formal
`megaatmega2560` (RAM 1206/8192, Flash 17968/253952). Browser DOM/screenshot
inspection at desktop size showed all controls and telemetry with no console
warnings/errors. No firmware upload, PWM measurement, track-off-ground test, or
powered chassis test was performed.

**Next Test:** Upload `mega_chassis_keyboard_test` over USB first. Keep tracks
off the floor, disconnect drivetrain power while confirming 1500 us neutral,
then power the motor controllers and test one direction at a time at the fixed
10% command. Wireless control can reuse the line protocol only when the adapter
is exposed to the Mac/browser as a serial port and is wired to the UART used by
the bench firmware; wireless firmware upload additionally requires a compatible
bootloader reset/DTR path. USB remains the recommended upload and first-test
transport.

### 2026-08-25 keyboard safety convention update

Jeremy standardized the keyboard safety behavior for current and future bench
pages. `Enter` now sends neutral plus disable while preserving the serial
connection. Space sends the same fail-safe disable before closing the serial
connection; Escape mirrors that emergency disconnect as a backup. Re-enabling
is intentionally available only through the visible `Enable / 啟用` button.
Window blur and page hiding continue to disable outputs without disconnecting.
This update changes only the Chassis test page and documentation; the firmware
protocol and Chassis control code are unchanged.

---

## 2026-09-11 - Codex: ELRS pairing setup intake

**User Request:** Jeremy asks to prioritize ELRS configuration and keep records here because another chat cannot receive his messages. Computer can find ExpressLRSRX and ExpressLRSTX, but TX/RX do not pair. Jeremy reports both ELRS units are on STM32, connected using USB, with default IP 10.0.0.1.

**Discussion / Evidence:** Read shared protocol and project handoff; checked official ExpressLRS binding and WebUI documentation. Device hardware targets, firmware versions, radio bands, binding UID, Model Match, and actual USB/UART topology are still unknown. Seeing configuration APs does not establish an RF control link. User-reported STM32 arrangement has not been independently verified.

**Approach / Why:** First collect TX and RX WebUI information separately and clarify whether USB connects to STM32 carrier boards or directly to ELRS modules. Then propose exact binding changes based on verified hardware/version; avoid selecting firmware targets from MCU assumptions. Official references: https://www.expresslrs.org/quick-start/binding/ and https://www.expresslrs.org/quick-start/webui/ .

**Changed / Impact:** Appended this record per explicit user request. No robot code, device configuration, firmware, or Git history changed. Existing .vscode/extensions.json modification was present before this work and was left untouched.

**Next Test:** Read each device information page, compare firmware major versions and binding configuration, then verify RF link outside WiFi configuration mode with actuator outputs disabled. No hardware test or successful pairing has been observed yet.

## 2026-09-11 - Codex: Corrected ELRS bench architecture and restart plan

**User Request / Correction:** Jeremy clarifies that ELRS modules and STM32 are mounted on breadboards; STM32F401CCU6 generates simulated radio-control signals for ELRS TX. The far end receives packets and returns data. PC connects by USB to the STM32 board for setup/flashing. This supersedes the earlier ambiguous phrase that ELRS itself is on STM32. Exact receiver-side MCU count/model and wiring are unconfirmed. Goal is a fresh start through configuration, flashing, binding and verified bidirectional packets.

**Evidence:** Read the supplied shared ChatGPT conversation in the browser: https://chatgpt.com/share/6aa3faf8-6054-83e8-92ac-a504ec5e64f8 . Visible prior assistant text mentions ES900TX, an elrs_f401ccu6_platformio project, CRSF, and ST-Link/DFU alternatives. These are historical claims, not verified current hardware or source. Focused Desktop/Downloads filename search did not locate the referenced ELRS project/archive. USB system_profiler command returned no text, so enumeration status is inconclusive.

**Proposed Architecture / Why:** Prefer CRSF RC uplink plus supported telemetry downlink to preserve Jeremy's simulated handset goal. Receiver-side MCU must generate identifiable return telemetry; local transmitter link statistics alone do not prove end-to-end return. Arbitrary byte-for-byte packet echo instead requires a deliberate transport choice; AirPort provides transparent bidirectional serial but replaces ordinary RC operation and has no built-in ACK/retry. References: https://www.expresslrs.org/software/airport/ , https://www.expresslrs.org/quick-start/receivers/wiring-up/ , https://docs.platformio.org/en/stable/boards/ststm32/blackpill_f401cc.html .

**Plan / Risks:** Confirm both module targets/versions, both MCU boards, wiring/power and intended payload first. Verify USB DFU enumeration and a minimal observable MCU firmware before ELRS integration. Then establish normal binding, validate changing RC channels at remote MCU, and validate remote-generated telemetry returning to originating MCU/PC. TX-side single-wire/inversion requirements depend on exact module connector; do not assume ordinary crossed UART wiring. Keep firmware work isolated from existing Mega Chassis code. Exact files and build target to be proposed after hardware confirmation, before implementation approval.

**Changed / Next:** Only appended this authorized record. No firmware/configuration changed or flashed; no physical link verified. Await module identities/versions, MCU count and current connections to select an actionable build and wiring plan.

## 2026-09-11 - Codex: Read-only review of supplied ELRS projects; direct configuration

**Request:** Treat both supplied Downloads projects as reference only. Jeremy says wiring/pins remain as supplied and asks whether ELRS can be configured without first programming STM32, and what alternatives exist.

**Verified files:** Compared elrs_f401ccu6_platformio and elrs_f401ccu6_platformio_usbdfu recursively. Source/header and base README are identical; differences are platformio.ini upload environments and added README_USB_DFU.md. USB variant changes the upload method to DFU; main.c has no USB CDC initialization or USB-to-UART bridge.

**Architecture correction:** Supplied code uses ONE F401 simultaneously connected to both ES900TX and ES900RX, rather than the two-MCU tentative diagram in the preceding entry. USART1 PA9 is configured single-wire half-duplex at 400000 baud to TX JR Signal. USART2 PA2 TX goes to RX module RX, PA3 RX goes to RX module TX, at 420000 baud. README names ES900TX/ES900RX; physical identities still need WebUI confirmation. JR pin numbering/orientation has not been physically verified.

**Conclusion:** WiFi-capable ELRS modules can be configured directly using their own WebUI without custom STM32 firmware. Existing ExpressLRS TX/RX hotspots make this the first route to inspect. ELRS 3.0+ supports setting binding phrase in WebUI. Configure each separately at 10.0.0.1, verify target/version/domain/UID, then leave WiFi mode for RF testing. Receiver disconnects RF when entering configuration WiFi mode. Setting matching binding configuration does not itself prove CRSF traffic or telemetry success. USB DFU on STM32 does not flash/configure the separate ELRS modules.

**Alternatives / limits:** Compatible handset plus ELRS Lua can supply a known CRSF source and configuration controls. Direct module firmware update via WiFi is possible with a verified matching firmware target if needed; USB/UART module flashing depends on the module's own interface and switch routing. AirPort is an optional transparent-data architecture that replaces regular RC; not required for current configuration-first task. No firmware, source, pins or device settings changed. Only this authorized handoff was appended.

**Next:** Inspect TX WebUI hardware target, firmware version and Options/Binding page first, followed by RX. Official references: https://www.expresslrs.org/quick-start/webui/ ; https://www.expresslrs.org/quick-start/binding/ ; https://www.expresslrs.org/quick-start/transmitters/es900tx/ .

## 2026-09-11 - Codex: TX WebUI screenshot evidence

Jeremy supplied five TX WebUI screenshots. Visible identity: HappyModel ES900 TX; Firmware Rev. 3.2.0 (8d3388), FCC915; update target HappyModel_TX_ES900TX. Options show UID Flashed 180,148,31,165,224,154; Binding Phrase field displays placeholder (does not establish absence of stored binding UID); WiFi auto-on 60 seconds; TLM report interval 240 ms; UART inverted unchecked; fan runtime 30 seconds; Unlock higher power checked. WiFi page reports Access Point mode. These are screenshot observations, not a live readback or evidence of pairing. No settings changed. Next: obtain RX identity/version/domain/UID before deciding whether binding settings need changes. Do not confuse TLM report interval with over-air telemetry ratio, or higher-power unlock with currently selected RF power. Keep STM32 firmware and pin mapping unchanged during this configuration comparison.

## 2026-09-11 - Codex: RX screenshot comparison

RX screenshots identify HappyModel ES900 RX, firmware 3.2.0 (8d3388), FCC915. Displayed UID Flashed 180,148,31,165,224,154 exactly matches TX screenshot. RX UART baud 420000 matches supplied STM32 USART2 configuration. Invert TX pin unchecked; Lock on first connection checked; Model Match unchecked; Force telemetry OFF unchecked; auto WiFi 60 seconds; WiFi AP mode shown. No obvious version/domain/displayed binding UID mismatch or RX-side forced telemetry disable is present. These screenshots do not prove an active RF link, TX telemetry ratio, or valid CRSF input. No reason established to reset binding or update firmware. Next discriminating question: whether supplied STM32 firmware has ever been successfully uploaded and is running, and current LED behavior after normal startup outside WiFi mode. No device or program modification performed; record only.

## 2026-09-11 - Codex: STM32 upload status unknown; DFU enumeration gate

Jeremy cannot confirm whether teammate successfully flashed STM32; teammate suspects an external programmer is required. Jeremy confirms TX WebUI has no Model Match control. Absence of that TX WebUI control is not evidence of a fault; RX screenshot has Model Match disabled. Proposed next action is read-only USB DFU enumeration after user holds BOOT0 while pressing/releasing NRST, then releases BOOT0. Standard BlackPill F401CC supports ROM DFU per Zephyr board documentation; actual board/cable/USB connectivity must be verified. ioreg USB read during this turn showed controller nodes but no STM32 device. This does not establish empty flash or inability to program: supplied application has no USB CDC implementation. No flashing or settings changes performed. External ST-Link remains a fallback rather than a demonstrated requirement. Reference: https://docs.zephyrproject.org/latest/boards/weact/blackpill_f401cc/doc/index.html .

## 2026-09-11 - Codex: First user-triggered DFU enumeration attempt

Jeremy reports completing BOOT0 + RESET. Read-only ioreg -p IOUSB -w 0 shows only two host controllers, no USB peripheral children; focused IOUSBHostDevice read yields no matching device properties. STM32 DFU enumeration is therefore not observed through the current tool view. This is not proof that firmware cannot be flashed, that flash is empty, or that ST-Link is required. Next: cross-check macOS System Information USB view and verify a data-capable cable/direct connection; inspect board photo and power indicator if still absent. No upload, erase, firmware/configuration change, or wiring change performed by Codex.

## 2026-09-11 - Codex: Photos and failed DFU with known data cable

Jeremy confirms BOOT0/NRST sequence repeated with a known data-capable cable; macOS System Information still shows only USB buses. Photos IMG_0962.JPG and IMG_0963.JPG show a BlackPill-style board with BOOT0/NRST/KEY markings, red PWR LED, USB plugged in, and ELRS peripherals still wired/powered. Exact PCB revision and every wire endpoint cannot be confirmed from obstructed photos. Power LED does not prove bootloader execution or successful USB data communication.

Next proposed diagnostic is an isolated STM32 test: remove all power first, label/preserve wire mapping, temporarily isolate STM32 from breadboard/peripheral signal and power connections, power only by USB, then retry BOOT0/NRST once. This isolates external electrical loading and boot-interface interactions without changing firmware or intended pin assignments. If still absent, investigate board USB path/boot straps/clock or use ST-Link SWD for read-only device identification before any erase/write. No diagnosis of damaged MCU or mandatory external programmer has been established. No firmware/configuration changed. ST bootloader reference: AN2606.

## 2026-09-11 - Codex: USB enumerates as WeAct HID, not ROM DFU

Jeremy reports cutting main power then performing BOOT0/NRST; supplied screenshot shows WeAct Studio HID Bootloader. Live ioreg independently confirms product name and VID 0x0483 / PID 0x572a. USB communication and execution of an installed HID bootloader are now demonstrated; teammate ELRS application presence remains unknown. Do not infer precise prior interference cause or that peripheral wires were removed. Earlier blanket DFU assumption is superseded for current device mode.

WeAct official HID README states default application forwarding address 0x08004000 with 16KB bootloader at 0x08000000; a compatible HID uploader and correctly linked application are required. Existing reference project's upload_protocol=dfu cannot communicate with current HID mode. Official V1.1+ procedure from HID mode: long-press KEY until C13 LED flashes, release to enter embedded bootloader (ISP); installed version not yet confirmed, so treat as next bounded test and verify resulting USB identity before uploading. No erase, upload or program edits performed. Reference: https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1/blob/master/Soft/WeAct_HID_FW_Bootloader/README.md .

## 2026-09-11 - Codex: HID KEY transition disconnects without observed DFU

Jeremy reports C13 flashing then going off and USB disconnecting after KEY operation. Live USB tree now shows only host controllers, no HID or DFU peripheral. Thus HID disappearance is observed but ROM DFU enumeration is not; do not claim successful DFU or damaged board. Main power was cut per prior user report, but physical isolation of all peripheral wires has not been confirmed. Next bounded diagnostic: with all power removed, isolate STM32 signal and external power wiring completely, keep original mapping documented, then USB-only repeat HID entry and documented KEY transition if HID is present. No reset/erase/upload performed by Codex. Preserve installed HID bootloader; HID-compatible application/uploader remains alternative if isolated DFU fails.

## 2026-09-11 - Codex: HID recovery attempt not observed by user

Jeremy replies that HID did not appear after the proposed recovery procedure. Completion of every isolation step and present PWR LED state are not independently confirmed. Stop repeating mode-switch sequences; next discriminate whether USB-only board has a lit PWR LED and whether a cold normal boot enumerates. Previously verified HID enumeration proves USB data worked at least once, but does not establish present boot state or successful ELRS application flashing. If power is present and cold boot remains unenumerated, SWD identification using ST-Link is the practical next diagnostic; do not erase or alter option bytes before reading identity/protection status and preserving readable existing flash. No firmware writes or configuration changes performed.

### Live enumeration correction immediately following previous entry

The actual ioreg result during this turn DOES show WeAct Studio HID Bootloader@00100000, registered and active. This supersedes the preceding tentative absent-device status: user UI did not show it, but live OS enumeration confirms HID has recovered. Keep current USB/power state and do not repeat KEY/BOOT0 switching. Next investigate compatible HID uploader and application link/vector offset before proposing any code change or upload; external ST-Link is not currently established as necessary. No flash operation performed.

## 2026-09-11 - Codex: HID upload feasibility checkpoint

Jeremy requests next step. Reference USB project uses DFU, STM32Cube, no custom linker/VTOR configuration. Installed PlatformIO package directory contains AVR toolchain/framework only, not STM32 build dependencies; pio and dfu-util are not on shell PATH. Official WeAct HID documentation lists Windows and Linux upload tools, states uploader/bootloader are not open source, and specifies application base 0x08004000 (16KB reserved bootloader). A supported macOS HID uploader has not been verified; do not claim ready-to-upload Mac path or substitute generic STM32 HID tooling without protocol validation.

Proposed next route: if Windows machine available, inspect/use official WeAct HID uploader with an application built and vector table relocated to 0x08004000. First application should be isolated PC13 blink with no ELRS UART traffic, preserving existing bootloader; F401CC 256KB flash leaves 240KB application space after 16KB reservation. Files would live in an isolated bench project (build config, linker/vector configuration, minimal main), preserving the supplied reference folders and robot source. Hardware upload and code changes still require concrete plan approval under standing user instructions. If Windows unavailable, confirm alternative host/tool compatibility or pursue SWD rather than inventing a macOS protocol implementation. No code edited or firmware uploaded.

## 2026-09-11 - Codex: Parallels Windows HID passthrough confirmed

User screenshots show Parallels USB menu checked for WeAct Studio HID Bootloader and Windows Device Manager vendor-defined HID hardware IDs HID\VID_0483&PID_572A&REV_0200 and HID\VID_0483&PID_572A. Thus guest Windows enumeration is confirmed; official WeAct uploader operation and flash programming remain untested. REV_0200 is a USB device revision descriptor, not independently verified bootloader software version. Next use official Windows WeAct HID Flash GUI only to inspect device recognition, without selecting an application or starting erase/download. Retain standard HID driver. Reference source folder: https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1/tree/master/Soft/WeAct_HID_FW_Bootloader . No firmware or project code modified.

## 2026-09-11 - Codex: WeAct MCU Info command succeeds; identity discrepancy

User screenshot of official HID tool reports device [0483:572A] found, read FW version command, 'FW version: V1.2 0x433 ROM: 384KB', Finish. This verifies bidirectional HID command/response through Parallels Windows, not flash write success. Tool-reported ROM 384KB is inconsistent with assumed STM32F401CC 256KB; raw 0x433 interpretation and custom bootloader ROM reporting require verification. Do not adopt 384KB as actual flash capacity or automatically change board target. Next confirm physical MCU marking from sharp close-up before choosing linker memory size/application target. No flash writes, erase, or source edits performed.

## 2026-09-12 - Codex: Approved HID LED test built, not flashed

**User Request / Approval:** Jeremy explicitly approved creating the LED test program. Chip top marking supplied as STM32F401CCU6 GH24SVQ CHN 518. Continue using conservative F401CC limits despite WeAct tool's unexplained 0x433/384KB report.

**Changed / Added:** Created isolated tools/stm32_hid_led_test with src/config.h, src/main.c, src/startup.S, linker.ld, build.py, README.md and dist artifacts; packaged tools/stm32_hid_led_test.zip. Root Mega platformio.ini, robot source and both Downloads reference projects unchanged. Installed PlatformIO ARM toolchain package 1.120301.0 with tool permission, GCC 12.3.1. Standalone bare-metal build avoids HAL startup overwriting VTOR and does not contain an upload command.

**Flow / Why:** HID jumps to application vector at 0x08004000; startup disables interrupts, establishes MSP and initializes data/BSS. Main resets inherited NVIC/SysTick state, switches to HSI 16MHz, resets USB and GPIO A/B/C states, then configures only active-low PC13. Test requires USB-only isolated board, external main power off. No CRSF/UART/USB CDC application traffic. Normal APP operation may remove HID enumeration; use documented KEY+reset to return to HID.

**Calculation:** Flash 256KiB minus reserved 16KiB = 240KiB application region, 0x08004000..0x0803FFFF. RAM 64KiB, initial MSP 0x20010000. SysTick reload 16000000/1000-1=15999, polling at nominal 1ms. Pattern: three 150ms on pulses, two 150ms inter-pulse off gaps, then 1200ms off: 3*150+2*150+1200=1950ms/group. HSI tolerance and polling overhead apply.

**Evidence:** ARM build succeeds with -Wall -Wextra -Werror. BIN 936 bytes; SHA256 bec72c5922c70303aaaa56fad960f319b60146d7cb877e74b4a4dcd0e1531340. Automated check verifies vector VMA/LMA 0x08004000, initial stack, all 101 Thumb handler pointers in image, size limit, no undefined symbols. Reset vector 0x08004199. Inspected linker sections and startup disassembly; git diff --check passes. ELF/map/disassembly/verification.json retained. No device commands, flash write, erase, or physical LED test performed.

**Risk / Next Test:** GUI exposes no verified APP readback; no old APP backup exists. Documentation explicitly says Download replaces old application and that layout avoidance of bootloader is not proof of closed-source uploader erase behavior. Deliver BIN for user selection and review, then authorized hardware test should verify at least three triple-flash groups, cold restart and HID recovery before ELRS integration. Build success is not hardware success. Existing .vscode/extensions.json change left untouched.

## 2026-09-12 - Codex: First HID download stalls; bootloader recovery confirmed

Jeremy selected the 936-byte stm32f401cc_hid_led.bin in Windows WeAct HID GUI and initiated Download. Screenshot shows 25%, 'Sending <reset pages> command ...' for over one minute; Parallels USB device remained checked. No write completion or LED execution observed. Advised closing stalled host tool before USB power cycle, recovering HID with KEY at power-on, then MCU Info only. Jeremy now supplies fresh successful log: HID [0483:572A] found, read FW version, V1.2 0x433 ROM:384KB, Finish. Bootloader communication has recovered; application erase/write extent is unknown. Do not claim old APP preserved or LED test flashed. Host Documents files are not targets of board flash commands.

Next investigation is GUI/bootloader flash command compatibility and board flash identity discrepancy, not RC pairing or repeated erase. Use read-only diagnostics and official tool documentation/source artifacts where available. A successful MCU Info response does not prove flash hardware or USB transport under erase is healthy. No further erase/download authorized by this status message.

## 2026-09-12 - Codex: Stalled uploader filename confirmed

User properties screenshot shows WeAct_HID_Flash_NETFramework4+ V11 FluentWPF 230502 AnyCPU.exe, 634368 bytes, in Windows Downloads. Filename matches previously recommended official AnyCPU artifact; this is not a hash/authenticity verification. Do not blame user selection or infer Mark-of-the-Web checkbox caused reset-pages timeout: program launches and MCU Info succeeds. Proposed bounded comparison: official sibling X64 GUI, close AnyCPU first, MCU Info only. Windows ARM compatibility and whether a changed process architecture affects the stalled flash operation remain unverified; no promise that X64 solves it, and no further erase/download performed.

## 2026-09-12 - Codex: X64-only MCU Info succeeds

After closing both uploader instances and reopening only the newly downloaded official X64 GUI, Jeremy supplies successful MCU Info output: [0483:572A] found, V1.2 0x433 ROM:384KB, Finish. Concurrent-tool contention is a possible explanation for the preceding X64 read stall, not a proven cause of the original AnyCPU reset-pages write stall. Next bounded comparison is a single Download attempt with only X64 open, same verified 936-byte LED BIN, isolated USB-powered board and external main power off. Existing APP state remains unknown. If reset-pages stalls again for over a minute, stop repeated HID writes and investigate outside the VM / via SWD identification rather than assuming bad BIN or applying APP Erase. No new device write performed by Codex in this turn.

## 2026-09-12 - Codex: Review of external AI advice and native Mac terminal log

Jeremy supplied external AI conversation and terminal transcript. Transcript confirms Homebrew installed dfu-util 0.11/libusb; dfu-util found no DFU device; global pip was blocked by PEP668, then user correctly created ~/my_env and pip reported no matching distribution for weact-hid-flash. No successful native flash shown. Current live ioreg shows no external USB devices; do not infer disconnect cause.

External advice contains unsupported claims: that 25% proves Parallels packet loss, guaranteed 100% native success, a special double-NRST deep-DFU sequence, and an official open-source pip weact-hid-flash uploader. WeAct official README instead lists Windows/Linux binaries and explicitly says bootloader/uploader are not open source; compatible macOS pip tool not verified. Do not install guessed packages or bypass PEP668 to solve package-not-found.

Critical correction: external advice tells user to write stm32f401cc_hid_led.bin at 0x08000000. This project's linker origin is 0x08004000 and the former address contains installed HID bootloader. Writing this APP at 0x08000000 would overwrite bootloader area and put linked code at wrong location. Do not execute that command. Earlier malformed command lacks -D and failed to find DFU, so supplied log does not show such overwrite. HID 0483:572A is not DFU 0483:DF11; installing dfu-util cannot change device mode. Next remains genuine native Windows HID comparison if available, or ST-Link SWD read-only identity/protection/backup inspection, rather than repeated destructive writes or invented tooling. No firmware or Python environment modified by Codex.

## 2026-09-12 - Codex: Approved native HID open diagnostics

Jeremy approved adding diagnostic messages and compiling/testing read. Inspected current Cli source in /Users/jeremy/Documents/GitHub/WeAct_HID_Bootloader_F4x1-master. Added only four stderr messages to Cli/hid-mac.c: selected path, registry entry, IOHIDDeviceCreate outcome, and IOHIDDeviceOpen SeizeDevice return code. Original preserved as hid-mac.c.before-diagnostics-20260912. Open mode/protocol/firmware unchanged. Compiled /tmp/WeAct_HID_Flash-CLI-diagnostic successfully with GCC/Apple clang, one existing kIOMasterPortDefault deprecation warning.

Executed read inside and outside sandbox (approved USB access); both returned device 0483:572A not found, exit 1. Live ioreg USB inventory contains only host controllers. Consequently no open-stage diagnostic was reached, no version read succeeded, and no erase/download command executed. Earlier ChatGPT transcript's HID enumeration evidence is historical, not current. Next: reconnect/recover HID with USB assigned to Mac, rerun diagnostic read; use stage/error output before changing exclusive-open behavior. Existing CLI executable was preserved; diagnostic executable is in /tmp.
