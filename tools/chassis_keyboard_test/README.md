# Chassis keyboard bench test

This standalone page controls only the Chassis through Web Serial. It does not
modify or use the robot Dashboard and requires the
`mega_chassis_keyboard_test` PlatformIO environment.

## Controls

- `2` selects the default Arcade mode: `W/S` forward/reverse and `A/D`
  left/right in-place turn.
- `1` selects Tank mode: `Q/A` left track forward/reverse and `E/D` right
  track forward/reverse. Compatible keys may be held together.
- `Enter` sends neutral and disables outputs while keeping the serial connection.
- Space is the emergency disconnect: it disables outputs before closing serial.
- Escape mirrors the emergency disconnect as a backup.
- Outputs are enabled only with the on-screen `Enable / 啟用` button.

The test command is limited to `10%` in `ChassisBenchConstants.h`. Firmware
boots disabled and independently disables outputs if no command or heartbeat
arrives for more than `250 ms`. Releasing a movement key updates both track
commands immediately. Changing modes first sends neutral.

## Build and run

1. In PlatformIO, select `env:mega_chassis_keyboard_test`.
2. Build and upload it to the Mega. This selects an environment, not a different
   project folder.
3. Serve this directory from localhost and open it in Chrome.
4. Connect the serial port with drive power off. Confirm both PWM values remain
   at `1500 us` while disabled.
5. Raise the chassis so the tracks cannot move the robot during the first
   powered direction test. Use an external motor supply and shared ground.

USB Serial is the recommended first connection. A wireless serial adapter can
use the same 115200-baud protocol if macOS exposes it as a serial port accepted
by the browser. Wireless firmware upload additionally requires a compatible
adapter and automatic reset wiring; otherwise upload by USB and use wireless
only for control/telemetry.

Current formal outputs are Mega D38 for the left drive and D39 for the right
drive. The architecture currently assumes one command signal per side.
