#pragma once

class DirectionalLimit {
public:
    static double apply(
        double command,
        bool reverseLimitTriggered,
        bool forwardLimitTriggered) {
        if ((command < 0.0 && reverseLimitTriggered) ||
            (command > 0.0 && forwardLimitTriggered)) {
            return 0.0;
        }
        return command;
    }
};
