#pragma once

struct PidfConfig {
    constexpr PidfConfig(
        double proportional = 0.0,
        double integral = 0.0,
        double derivative = 0.0,
        double integralZone = 0.0,
        double feedforward = 0.0)
        : kP(proportional),
          kI(integral),
          kD(derivative),
          kIZone(integralZone),
          kFF(feedforward) {}

    double kP;
    double kI;
    double kD;
    double kIZone;
    double kFF;
};

class PidfController {
public:
    explicit PidfController(const PidfConfig& config = PidfConfig());

    void setConfig(const PidfConfig& config);
    void setOutputLimits(double minimum, double maximum);
    void setIntegralOutputLimits(double minimum, double maximum);
    void reset(double measurement = 0.0);

    double calculate(
        double setpoint,
        double measurement,
        double dtSeconds,
        double feedforwardReference = 0.0);

    double getError() const;
    double getMeasurementRate() const;
    double getProportionalTerm() const;
    double getIntegralTerm() const;
    double getDerivativeTerm() const;
    double getFeedforwardTerm() const;
    double getOutput() const;
    bool isSaturated() const;

private:
    static double clamp(double value, double minimum, double maximum);
    void clearTerms();

    PidfConfig config;
    double outputMinimum = -1.0;
    double outputMaximum = 1.0;
    double integralOutputMinimum = -1.0;
    double integralOutputMaximum = 1.0;
    double integralAccumulator = 0.0;
    double previousMeasurement = 0.0;
    double error = 0.0;
    double measurementRate = 0.0;
    double proportionalTerm = 0.0;
    double integralTerm = 0.0;
    double derivativeTerm = 0.0;
    double feedforwardTerm = 0.0;
    double output = 0.0;
    bool measurementInitialized = false;
    bool saturated = false;
};
