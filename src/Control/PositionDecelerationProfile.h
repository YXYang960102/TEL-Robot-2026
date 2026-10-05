#pragma once

#include <math.h>

class PositionDecelerationProfile {
public:
    static double calculateMaximumCommand(
        double initialErrorMagnitude,
        double currentErrorMagnitude,
        double decelerationFraction,
        double cruiseCommand,
        double minimumApproachCommand) {
        initialErrorMagnitude = fabs(initialErrorMagnitude);
        currentErrorMagnitude = fabs(currentErrorMagnitude);
        cruiseCommand = fabs(cruiseCommand);
        minimumApproachCommand = fabs(minimumApproachCommand);

        if (initialErrorMagnitude <= 0.0 || cruiseCommand <= 0.0) {
            return 0.0;
        }
        if (decelerationFraction <= 0.0 ||
            currentErrorMagnitude / initialErrorMagnitude >=
                decelerationFraction) {
            return cruiseCommand;
        }

        const double scaledCommand =
            cruiseCommand *
            (currentErrorMagnitude / initialErrorMagnitude) /
            decelerationFraction;
        if (scaledCommand < minimumApproachCommand) {
            return minimumApproachCommand;
        }
        return scaledCommand > cruiseCommand
            ? cruiseCommand
            : scaledCommand;
    }

    static double calculateProfiledCommand(
        double error,
        double initialErrorMagnitude,
        double decelerationFraction,
        double cruiseCommand,
        double minimumApproachCommand) {
        const double maximumCommand = calculateMaximumCommand(
            initialErrorMagnitude,
            error,
            decelerationFraction,
            cruiseCommand,
            minimumApproachCommand);
        if (error > 0.0) {
            return maximumCommand;
        }
        if (error < 0.0) {
            return -maximumCommand;
        }
        return 0.0;
    }
};
