# Bare Mega UART diagnostic

Use only a bare Mega connected to the Mac by USB. Disconnect all power before
removing the shield, Orin connection, motor, and D18/D19 loopback jumper.
This firmware enables D19/RX1's internal pull-up to Mega 5V; do not attach Orin
or any external equipment during this test. It never initializes Servo or motors.

Build: `pio run -e mega_uart_bare_test`.
Upload manually using PlatformIO's Upload under **mega_uart_bare_test**, not
the default robot environment. Close all browser serial connections first.
Open the USB serial monitor at115200 baud (8N1).

The test automatically performs:
1. QUIET for10000ms: Serial1 receiver enabled with pull-up, no TX frames.
2. TX for10000ms: same RX setting, send `MEGA_HEARTBEAT,1\r\n` every200ms
   (5Hz) on D18. The first send is after200ms; approximately49 frames per phase.
3. DONE: no more transmissions. Send R to repeat; R during a test is ignored.

It prints STAT once per second and RESULT at each phase end. Counts are
cumulative within each phase; only the first16 received bytes are shown in hex.
`rx_pin` is one instant logic sample, not an oscilloscope measurement.
The receiver is intentionally not flushed between phases; boundary bytes may
carry over. Keep both RESULT lines or send R if the boot output was missed.

Expected with an isolated healthy board: rx_bytes=0 in both phases.
Both zero supports floating/noise as a cause of the earlier unconnected-input
result but does not validate the shield or the Orin link. Nonzero only during
TX suggests TX-correlated interference/leakage/return path; it is not proof of
a hard short. Nonzero in QUIET requires further board/pin/power investigation.
Arduino RX counts are bytes delivered by its driver, not frame-error counters
or an electrical measurement. Do not declare hardware faulty from this alone.

After diagnosis this replaces, not runs alongside, vision firmware. Restore the
intended bench firmware with all external equipment disconnected before wiring
the system back up. No autonomous following should be enabled for this test.
