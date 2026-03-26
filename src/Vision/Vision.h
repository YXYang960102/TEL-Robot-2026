#ifndef VISION_H
#define VISION_H

#include <Arduino.h>

class Vision {
public:
    void begin(HardwareSerial &serial, long baud);
    void update(); 
    
    float getXError() { return x_error; }
    float getYRange() { return y_range; }
    double getXPred() { return xPred; }

private:
    HardwareSerial* _serial;
    float x_error = 0, y_range = 0;
    
    double xPred = 0, xFilt = 0, xFiltPrev = 0, vx = 0;
    unsigned long lastTime = 0;
    bool firstSample = true;

    void parseXY(String s);
    void updatePrediction(double x_meas);
};

#endif