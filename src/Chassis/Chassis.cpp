#include "Chassis.h"

#include "../Constants/ChassisConstants.h"
#include "DifferentialDriveMixer.h"

#include <Arduino.h>

using namespace ChassisConstants;

Servo Chassis::rightDrive;
Servo Chassis::leftDrive;
double Chassis::forwardCommand = 0.0;
double Chassis::turnCommand = 0.0;
double Chassis::leftCommand = 0.0;
double Chassis::rightCommand = 0.0;
bool Chassis::outputsEnabled = false;
int Chassis::rightPulseUs = Output::NEUTRAL_US;
int Chassis::leftPulseUs = Output::NEUTRAL_US;
double Chassis::filteredLeftCommand = 0.0;
double Chassis::filteredRightCommand = 0.0;
unsigned long Chassis::lastUpdateMs = 0;
bool Chassis::hasLastUpdateMs = false;

void Chassis::init() {
    rightDrive.attach(RightDrive::SIGNAL_PIN);
    leftDrive.attach(LeftDrive::SIGNAL_PIN);
    outputsEnabled = false;
    hasLastUpdateMs = false;
    stop();
}

void Chassis::update() {
    if (!outputsEnabled) {
        rightPulseUs = Output::NEUTRAL_US;
        leftPulseUs = Output::NEUTRAL_US;
        filteredLeftCommand = 0.0;
        filteredRightCommand = 0.0;
        hasLastUpdateMs = false; // next enable starts the filter fresh, not a huge dt
    } else {
        const unsigned long now = millis();
        const double dtMs = hasLastUpdateMs
            ? static_cast<double>(now - lastUpdateMs)
            : 0.0; // first tick after an enable: snap filtered state to target
        lastUpdateMs = now;
        hasLastUpdateMs = true;

        rightPulseUs = commandToPulseUs(
            filterTowards(rightCommand, filteredRightCommand, dtMs),
            RightDrive::INVERTED);
        leftPulseUs = commandToPulseUs(
            filterTowards(leftCommand, filteredLeftCommand, dtMs),
            LeftDrive::INVERTED);
    }
    rightDrive.writeMicroseconds(rightPulseUs);
    leftDrive.writeMicroseconds(leftPulseUs);
}

void Chassis::setOutputsEnabled(bool enabled) {
    if (enabled && !outputsEnabled) {
        stop();
    }
    outputsEnabled = enabled;
    if (!outputsEnabled) {
        stop();
    }
}

bool Chassis::areOutputsEnabled() {
    return outputsEnabled;
}

void Chassis::setOpenLoop(double forward, double turn) {
    forwardCommand = applyDeadband(constrain(
        forward,
        Manual::MIN_COMMAND,
        Manual::MAX_COMMAND));
    turnCommand = applyDeadband(constrain(
        turn,
        Manual::MIN_COMMAND,
        Manual::MAX_COMMAND));

    const DifferentialDriveOutput mixed =
        DifferentialDriveMixer::mixArcade(forwardCommand, turnCommand);
    leftCommand = mixed.left;
    rightCommand = mixed.right;
}

void Chassis::setTankOpenLoop(double left, double right) {
    leftCommand = applyDeadband(constrain(
        left,
        Manual::MIN_COMMAND,
        Manual::MAX_COMMAND));
    rightCommand = applyDeadband(constrain(
        right,
        Manual::MIN_COMMAND,
        Manual::MAX_COMMAND));
    forwardCommand = (leftCommand + rightCommand) / 2.0;
    turnCommand = (leftCommand - rightCommand) / 2.0;
}

void Chassis::stop() {
    forwardCommand = 0.0;
    turnCommand = 0.0;
    leftCommand = 0.0;
    rightCommand = 0.0;
    // Immediate, not filtered: an explicit/safety stop must not ease out
    // over the filter's time constant.
    filteredLeftCommand = 0.0;
    filteredRightCommand = 0.0;
    hasLastUpdateMs = false;
    rightPulseUs = Output::NEUTRAL_US;
    leftPulseUs = Output::NEUTRAL_US;

    if (rightDrive.attached()) {
        rightDrive.writeMicroseconds(rightPulseUs);
    }
    if (leftDrive.attached()) {
        leftDrive.writeMicroseconds(leftPulseUs);
    }
}

double Chassis::getForwardCommand() {
    return forwardCommand;
}

double Chassis::getTurnCommand() {
    return turnCommand;
}

double Chassis::getLeftCommand() {
    return leftCommand;
}

double Chassis::getRightCommand() {
    return rightCommand;
}

int Chassis::getRightPulseUs() {
    return rightPulseUs;
}

int Chassis::getLeftPulseUs() {
    return leftPulseUs;
}

double Chassis::applyDeadband(double command) {
    // Rescaled, not a hard cutoff: output is 0 at |command| == DEADBAND and
    // ramps continuously to +-1 at the stick's full range. A hard cutoff
    // (0 below the threshold, unchanged above it) jumps straight from 0 to
    // DEADBAND*RANGE_US the instant the stick crosses the threshold -- a
    // real step on the PWM pin, not a ramp -- which can sit right in a
    // brushed ESC's weak low-throttle band and judder (Jeremy, 2026-10-17).
    const double magnitude = abs(command);
    if (magnitude < Manual::COMMAND_DEADBAND) {
        return 0.0;
    }
    const double scaled = (magnitude - Manual::COMMAND_DEADBAND) / (1.0 - Manual::COMMAND_DEADBAND);
    return command < 0.0 ? -scaled : scaled;
}

double Chassis::filterTowards(double target, double& state, double dtMs) {
    // Exponential filter: state moves a fraction `alpha` of the way to
    // `target` each call. alpha = dt / (dt + tau) makes the result
    // independent of how often update() actually runs (loop jitter,
    // delay(10) plus whatever else ran that tick), not just a fixed
    // per-call step. dtMs <= 0 (first tick after an enable) snaps straight
    // to the target instead of dividing by a non-positive time constant.
    if (dtMs <= 0.0 || Manual::COMMAND_FILTER_TIME_CONSTANT_MS <= 0.0) {
        state = target;
        return state;
    }
    const double alpha = dtMs / (dtMs + Manual::COMMAND_FILTER_TIME_CONSTANT_MS);
    state += (target - state) * alpha;
    return state;
}

int Chassis::commandToPulseUs(double command, bool inverted) {
    const double appliedCommand = inverted ? -command : command;
    return constrain(
        Output::NEUTRAL_US + static_cast<int>(appliedCommand * Output::RANGE_US),
        Output::MIN_PULSE_US,
        Output::MAX_PULSE_US);
}
