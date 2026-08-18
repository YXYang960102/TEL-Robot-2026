# AS5600 Servo UNO Test

This bench sketch uses the AS5600 as a boot-zeroed relative multi-turn encoder and drives a continuous-rotation servo toward relative count `1024`.

## Wiring

AS5600 to Arduino UNO:

- `VCC` to `5V`
- `GND` to `GND`
- `SDA` to `A4`
- `SCL` to `A5`

Servo:

- Signal wire to `D9`
- Servo power to an external supply
- External supply ground to Arduino `GND`

Do not power a high-torque servo from the UNO 5V pin or from the Mac USB port.

## PlatformIO

```sh
pio run -e uno_as5600_servo_test
pio run -e uno_as5600_servo_test -t upload
```

## Source

The test is split into small modules:

- `src/Constants/As5600ServoTestConstants.h`
- `src/Sensors/AS5600Encoder.*`
- `src/Actuators/ContinuousServoMotor.*`
- `src/Bench/As5600ServoTestSubsystem.*`
- `src/Bench/As5600ServoUnoTest.cpp`

## Serial Output

The sketch starts in manual open-loop mode. Send commands from the Serial Monitor:

- `f`: forward open-loop
- `r`: reverse open-loop
- `s`: stop
- `+`: increase manual speed offset
- `-`: decrease manual speed offset
- `z`: zero encoder at the current position
- `p`: switch to PID closed-loop mode
- `m`: switch back to manual open-loop mode
- `h`: print command help

Use PID only after motor motion changes the AS5600 reading through the real mechanism.

- `raw`: AS5600 raw count, `0` to `4095`
- `zero_raw`: the raw count captured at boot as relative zero
- `delta`: shortest signed movement since the previous sample
- `rejected_delta`: ignored movement when a sample jumps too far to trust
- `rejected_count`: total number of rejected jump samples
- `relative`: boot-zeroed multi-turn position; this can pass `4095` or go negative
- `target_relative`: target relative count, currently `1024`
- `error`: shortest signed error to the target
- `abs_error`: absolute error in AS5600 counts
- `error_deg`: absolute error in degrees, where one count is about `0.0879` degrees
- `pid_us`: PID output added to the servo stop pulse
- `pulse_us`: final servo command, where `1500` is stop
- `stopped`: `1` when the error is inside the tolerance band
- `magnet`: `1` when the AS5600 detects the magnet

If the servo moves the wrong way, change `config.outputSign` in `src/Bench/As5600ServoUnoTest.cpp` from `1` to `-1`.

The continuous servo may not move near its stop pulse. This test uses at least `80us`
of offset so commands move outside the common `1450us` to `1500us` stop band.

The current tolerance is `stopToleranceCounts = 1`, so `relative=1023`, `1024`,
or `1025` stops. Use `0` if only `1024` should stop.

`maxEncoderDeltaCounts = 180` filters unlikely one-sample jumps before they affect
the multi-turn relative position. If `rejected_count` rises while the mechanism is
not moving quickly, check magnet alignment, AS5600 wiring, servo power noise, and
shared ground.
