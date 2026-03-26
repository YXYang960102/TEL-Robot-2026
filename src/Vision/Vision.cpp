#include "Vision.h"
#include "../Constants/RobotConstants.h"

void Vision::begin(HardwareSerial &serial, long baud) {
    _serial = &serial;
    _serial->begin(baud);
}

void Vision::update() {
    if (_serial->available()) {
        String input = _serial->readStringUntil('\n');
        parseXY(input);
    }
}

void Vision::parseXY(String s) {
    int commaIndex = s.indexOf(',');
    if (commaIndex != -1) {
        x_error = s.substring(0, commaIndex).toFloat();
        y_range = s.substring(commaIndex + 1).toFloat();
        updatePrediction(x_error);
    }
}

void Vision::updatePrediction(double x_meas) {
    using namespace RobotConfig::Vision;
    unsigned long now = millis();
    if (firstSample) {
        xFilt = x_meas;
        lastTime = now;
        firstSample = false;
        return;
    }
    double dt = (now - lastTime) / 1000.0;
    if (dt <= 0) return;

    xFilt = FILTER_ALPHA * xFilt + PREDICTION_WEIGHT * x_meas; 
    vx = (xFilt - xFiltPrev) / dt;
    xPred = xFilt + vx * PREDICTION_TD;
    
    xFiltPrev = xFilt;
    lastTime = now;
}