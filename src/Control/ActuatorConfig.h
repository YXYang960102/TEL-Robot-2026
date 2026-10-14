#pragma once

struct ServoMotorConfig {
    constexpr ServoMotorConfig(
        bool invertedValue,
        int minimumPulseUs,
        int neutralPulseUs,
        int maximumPulseUs)
        : inverted(invertedValue),
          minimumUs(minimumPulseUs),
          neutralUs(neutralPulseUs),
          maximumUs(maximumPulseUs) {}

    bool inverted;
    int minimumUs;
    int neutralUs;
    int maximumUs;
};
