#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Constants/ShooterConstants.h"

// Shooter head: three motors, no sensors (see ShooterConstants.h).
//   turret    -- manual open loop, or vision-follow PIDF on the camera's tx
//   elevation -- manual position jog (or speed, for a continuous servo)
//   flywheel  -- open-loop PWM command, 0..1

enum class AngleControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP
};

enum class RotateControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    VISION_FOLLOW
};

class Shooter {
public:
    // Lifecycle and safety
    static void init();
    static void update();
    static void stopAll();
    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();

    // Manual control (-1..1; flywheel 0..1)
    static void setAngleOpenLoop(double command);
    // Elevation as an absolute target angle (positional servo only; degrees
    // per ShooterConstants::Angle::ANGLE_MAP, clamped to its range). Returns
    // false on a continuous-rotation elevation, which has no position.
    static bool setAngleTargetDegrees(double degrees);
    // Same, as the raw target pulse (us), clamped to Angle::MIN/MAX_PULSE_US.
    // Auto uses this with the shot table's measured pulses.
    static bool setAnglePulseUs(double pulseUs);
    static double getAngleTargetDegrees();
    static double getAngleMinDegrees();
    static double getAngleMaxDegrees();
    static void setRotateOpenLoop(double command);
    static void setFlywheelOpenLoop(double command);
    static void disableRotate();

    // Turret driven by the camera: VisionFollowController on Vision::getTx()
    // every update().
    static void setRotateVisionFollow();
    // Operator fine-adjust: vision-follow aims this many px right (+) of
    // the target's center. 0 = straight at it.
    static void setRotateAimOffsetPx(double offsetPx);

    // Status
    // Turret locked on target: |tx| within VISION_READY_TOLERANCE_PX for
    // READY_SETTLE_TIME_MS while vision-following.
    static bool isRotateReady();
    // Time-based proxy only: no RPM/tachometer feedback exists for the
    // Falcon 500 flywheel (PWM command path is one-way). True once the
    // commanded output has been non-zero and steady (within
    // FLYWHEEL_READY_BAND of where it settled) for Flywheel::SPIN_UP_MS.
    // Re-verify against real shots before trusting it.
    static bool isFlywheelReady();
    // Ready to fire: turret locked and flywheel spun up (the same two
    // conditions Auto checks before feeding).
    static bool isReady();
    static AngleControlMode getAngleControlMode();
    static RotateControlMode getRotateControlMode();

    // Telemetry
    static int getAnglePulseUs();
    static int getRotatePulseUs();
    static int getFlywheelPulseUs();
    // The open-loop command actually driving the flywheel, 0.0-1.0 -- a
    // command ratio, not an RPM (no speed feedback exists).
    static double getFlywheelCommand();

private:
    static void updateAngle();
    static void updateRotate();
    static void updateFlywheelReady();
    static void writeAngleCommand(double command);
    static void releaseAngle();
    static void writeRotateCommand(double command);
    static void writeFlywheelCommand(double command);

    static Servo angleServo;
    static Servo rotateServo;
    static Servo flywheelOutput;
    static AngleControlMode angleMode;
    static RotateControlMode rotateMode;
    static bool outputsEnabled;
    static bool rotateReady;
    static double rotateAimOffsetPx;
    static double angleManualCommand;
    static double rotateManualCommand;
    static double flywheelOpenLoopCommand;
    static int anglePulseUs;
    static int rotatePulseUs;
    static int flywheelPulseUs;
    static unsigned long flywheelSpinStartMs;
    static double flywheelSpinCommand; // the command the spin-up timer started at
    // What actually goes to the Falcon: flywheelOpenLoopCommand, reached
    // at Flywheel::RAMP_UP_PER_SECOND going up, at once going down.
    static double flywheelOutputCommand;
    static unsigned long flywheelRampLastMs;
    static void rampFlywheel();
    static bool flywheelReady;
};
