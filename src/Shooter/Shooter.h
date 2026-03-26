#ifndef SHOOTER_H
#define SHOOTER_H

#include <Servo.h>
#include <PID_v1.h>
#include <AS5600.h>

class Shooter {
public:
    void begin();
    void update(double xError, double yRange, bool autoMode);
    void fire(int speed); 
    void stop();

private:
    Servo horMotor;   // 水平轉向
    Servo verMotor;   // 垂直仰角
    Servo flyWheel;  // Falcon 主發射馬達
    Servo esc_l, esc_r; // 左右發射輔助輪
    
    AMS_5600 ams5600;
    
    // PID 控制相關
    double pidInput, pidOutput, pidSetpoint;
    PID* horPID;
    
    // 內部邏輯：根據距離自動調整仰角與轉速
    void adjustVertical(double yRange);
};

#endif