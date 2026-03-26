#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Servo.h>
#include <Arduino.h>

class Dribbler {
public:
    void begin();
    void update(bool manualIntake, bool manualOuttake); 
    void intake();  
    void outtake(); 
    void stop();    
    bool hasBall(); 

private:
    Servo motor;
    int sensorPin;
};

#endif