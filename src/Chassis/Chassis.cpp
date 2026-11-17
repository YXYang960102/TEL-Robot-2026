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
        hasLastUpdateMs = false;
    } else {
        const unsigned long now = millis();
        const double dtMs = hasLastUpdateMs
            ? static_cast<double>(now - lastUpdateMs)
            : 0.0;
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
    const double magnitude = abs(command);
    if (magnitude < Manual::COMMAND_DEADBAND) {
        return 0.0;
    }
    const double scaled = (magnitude - Manual::COMMAND_DEADBAND) / (1.0 - Manual::COMMAND_DEADBAND);
    return command < 0.0 ? -scaled : scaled;
}

double Chassis::filterTowards(double target, double& state, double dtMs) {
    if (dtMs <= 0.0 || Manual::COMMAND_FILTER_TIME_CONSTANT_MS <= 0.0) {
        state = target;
        return state;
    }
    if (target * state < 0.0) state = 0.0;
    if (fabs(target) <= fabs(state)) {
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
