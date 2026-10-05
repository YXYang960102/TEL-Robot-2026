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

struct PositionLimitConfig {
    constexpr PositionLimitConfig(
        bool forwardEnabledValue,
        bool reverseEnabledValue,
        long forwardLimitValue,
        long reverseLimitValue)
        : forwardEnabled(forwardEnabledValue),
          reverseEnabled(reverseEnabledValue),
          forwardLimit(forwardLimitValue),
          reverseLimit(reverseLimitValue) {}

    bool forwardEnabled;
    bool reverseEnabled;
    long forwardLimit;
    long reverseLimit;
};

struct DigitalLimitSwitchConfig {
    constexpr DigitalLimitSwitchConfig(
        unsigned char signalPinValue,
        bool triggeredHighValue,
        bool useInternalPullupValue,
        unsigned long debounceMsValue)
        : signalPin(signalPinValue),
          triggeredHigh(triggeredHighValue),
          useInternalPullup(useInternalPullupValue),
          debounceMs(debounceMsValue) {}

    unsigned char signalPin;
    bool triggeredHigh;
    bool useInternalPullup;
    unsigned long debounceMs;
};
