# Autonomous Zone Strategy

Field coordinate proposal:

- `x = 0..400 cm`
- `y = 0..1700 cm`
- `y = 0..300`: prep zone
- `y = 300..600`: human zone
- `y = 600..1100`: robot operating zone
- `y = 1100..1700`: scoring zone side

The robot should only move inside the robot operating zone. With a safety margin, the robot center should stay inside:

```text
x = 40..360 cm
y = 640..1060 cm
```

## Method 1: IMU and Encoders

This is the recommended autonomous mode.

Use:

- wheel encoders for chassis translation distance
- IMU yaw for heading
- fixed start pose from the starting box

Flow:

```text
choose left or right start box
set initial x/y/heading
read encoder delta and IMU yaw
update estimated robot pose
predict next x/y before applying chassis command
block vx/vy if next pose leaves the safe robot zone
use YOLO tx/distance only for aiming and distance decisions
```

Advantages:

- works even when the target is temporarily not visible
- can enforce field boundaries
- good enough for a conservative auto routine

Limits:

- mecanum/omni wheels can slip
- encoder odometry drifts over time
- needs calibration for wheel diameter, chassis geometry, and heading sign

## Method 2: No IMU and No Encoders

This can be tested, but it should not be trusted as the only boundary protection.

Camera-only options:

1. Use target geometry to estimate relative angle and distance.
2. Use field markers such as AprilTag, ArUco, or known visual landmarks.
3. Use optical flow or image motion to estimate movement.

For this robot, plain YOLO detection alone is not enough to know field position. YOLO can tell:

```text
target visible or not
target left/right offset
approximate target distance
target id
```

But YOLO alone cannot reliably tell:

```text
absolute robot x/y in the field
whether the robot is about to cross the robot-zone boundary
how far the chassis moved if the target disappears
```

Safe camera-only auto should therefore be conservative:

```text
do not freely drive around the field
only rotate in place for aiming
allow short timed forward movement at low speed
stop when valid == 0
stop when distance is too close
stop after a strict time limit
require driver override at all times
```

Recommended first camera-only routine:

```text
start in known box
wait for valid target
rotate until abs(tx) is small
if distance is too far, move forward slowly for a short fixed time
stop and aim again
shoot only when valid, tx, and distance are all acceptable
never continue forward if target is lost
```

## Practical Recommendation

Build the auto system in stages:

1. Vision-only dashboard monitoring.
2. Stationary auto aim.
3. Timed low-speed forward movement with hard stop.
4. Add IMU and encoders.
5. Enable true field-zone safety using estimated robot pose.

The dashboard field view already supports optional `robot_x`, `robot_y`, and `heading` telemetry fields, so the UI is ready when odometry is added later.
