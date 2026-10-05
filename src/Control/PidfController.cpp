#include "PidfController.h"

#include <math.h>

PidfController::PidfController(const PidfConfig& initialConfig)
    : config(initialConfig) {}

void PidfController::setConfig(const PidfConfig& newConfig) {
    config = newConfig;
    reset(previousMeasurement);
}

void PidfController::setOutputLimits(double minimum, double maximum) {
    if (minimum > maximum) {
        const double temporary = minimum;
        minimum = maximum;
        maximum = temporary;
    }

    outputMinimum = minimum;
    outputMaximum = maximum;
    output = clamp(output, outputMinimum, outputMaximum);
}

void PidfController::setIntegralOutputLimits(double minimum, double maximum) {
    if (minimum > maximum) {
        const double temporary = minimum;
        minimum = maximum;
        maximum = temporary;
    }

    integralOutputMinimum = minimum;
    integralOutputMaximum = maximum;
    integralTerm = clamp(
        integralTerm,
        integralOutputMinimum,
        integralOutputMaximum);
}

void PidfController::reset(double measurement) {
    integralAccumulator = 0.0;
    previousMeasurement = measurement;
    measurementInitialized = true;
    clearTerms();
}

double PidfController::calculate(
    double setpoint,
    double measurement,
    double dtSeconds,
    double feedforwardReference) {
    error = setpoint - measurement;
    proportionalTerm = config.kP * error;
    feedforwardTerm = config.kFF * feedforwardReference;

    const bool validDt = dtSeconds > 0.0 && dtSeconds < 1000000.0;
    if (measurementInitialized && validDt) {
        measurementRate = (measurement - previousMeasurement) / dtSeconds;
        derivativeTerm = -config.kD * measurementRate;
    } else {
        measurementRate = 0.0;
        derivativeTerm = 0.0;
    }

    const bool integralEnabled =
        config.kI != 0.0 &&
        config.kIZone > 0.0 &&
        fabs(error) <= config.kIZone &&
        validDt;

    if (!integralEnabled) {
        integralAccumulator = 0.0;
        integralTerm = 0.0;
    } else {
        double candidateAccumulator = integralAccumulator + error * dtSeconds;
        double minimumAccumulator = integralOutputMinimum / config.kI;
        double maximumAccumulator = integralOutputMaximum / config.kI;
        if (minimumAccumulator > maximumAccumulator) {
            const double temporary = minimumAccumulator;
            minimumAccumulator = maximumAccumulator;
            maximumAccumulator = temporary;
        }
        candidateAccumulator = clamp(
            candidateAccumulator,
            minimumAccumulator,
            maximumAccumulator);

        const double candidateIntegralTerm = clamp(
            config.kI * candidateAccumulator,
            integralOutputMinimum,
            integralOutputMaximum);
        const double candidateOutput =
            proportionalTerm +
            candidateIntegralTerm +
            derivativeTerm +
            feedforwardTerm;

        const bool candidateInsideOutputLimits =
            candidateOutput >= outputMinimum &&
            candidateOutput <= outputMaximum;
        const bool candidateRelievesHighSaturation =
            candidateOutput > outputMaximum &&
            candidateIntegralTerm < integralTerm;
        const bool candidateRelievesLowSaturation =
            candidateOutput < outputMinimum &&
            candidateIntegralTerm > integralTerm;

        if (candidateInsideOutputLimits ||
            candidateRelievesHighSaturation ||
            candidateRelievesLowSaturation) {
            integralAccumulator = candidateAccumulator;
            integralTerm = candidateIntegralTerm;
        }
    }

    const double rawOutput =
        proportionalTerm +
        integralTerm +
        derivativeTerm +
        feedforwardTerm;
    output = clamp(rawOutput, outputMinimum, outputMaximum);
    saturated = output != rawOutput;
    previousMeasurement = measurement;
    measurementInitialized = true;
    return output;
}

double PidfController::getError() const {
    return error;
}

double PidfController::getMeasurementRate() const {
    return measurementRate;
}

double PidfController::getProportionalTerm() const {
    return proportionalTerm;
}

double PidfController::getIntegralTerm() const {
    return integralTerm;
}

double PidfController::getDerivativeTerm() const {
    return derivativeTerm;
}

double PidfController::getFeedforwardTerm() const {
    return feedforwardTerm;
}

double PidfController::getOutput() const {
    return output;
}

bool PidfController::isSaturated() const {
    return saturated;
}

double PidfController::clamp(double value, double minimum, double maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void PidfController::clearTerms() {
    error = 0.0;
    measurementRate = 0.0;
    proportionalTerm = 0.0;
    integralTerm = 0.0;
    derivativeTerm = 0.0;
    feedforwardTerm = 0.0;
    output = 0.0;
    saturated = false;
}
