#include "Shooter.h"
#include "../Constants/RobotConstants.h"

void Shooter::begin() {
    using namespace RobotConfig::Shooter;
    using namespace RobotConfig::Chassis; 

    horMotor.attach(PIN_HOR_MOTOR); 
    verMotor.attach(PIN_VER_MOTOR);
    flyWheel.attach(PIN_FALCON);
    esc_l.attach(PIN_FLYWHEEL_L);
    esc_r.attach(PIN_FLYWHEEL_R);
    
    
    pidSetpoint = 0; 
    horPID = new PID(&pidInput, &pidOutput, &pidSetpoint, HOR_KP, HOR_KI, HOR_KD, DIRECT);
    horPID->SetMode(AUTOMATIC);
    horPID->SetOutputLimits(-500, 500); // 1500 +/- 500
    stop();
}

void Shooter::update(double xError, double yRange, bool autoMode) {
    if (autoMode) {
        pidInput = xError;
        horPID->Compute();
        horMotor.writeMicroseconds(RobotConfig::Chassis::PWM_STOP + (int)pidOutput);
        adjustVertical(yRange);
    } else {
        horMotor.writeMicroseconds(RobotConfig::Chassis::PWM_STOP);
    }
}

void Shooter::adjustVertical(double yRange) {
    using namespace RobotConfig::Auto;
    using namespace RobotConfig::Shooter;

    if (yRange <= 0) return; 

    int targetIndex = 0;
    double minDiff = 999.0;

    for (int i = 0; i < TABLE_SIZE; i++) {
        double diff = abs(yRange - Y_RANGE_TABLE[i]);
        if (diff < minDiff) {
            minDiff = diff;
            targetIndex = i;
        }
    }

    verMotor.writeMicroseconds(VER_ANGLE_TABLE[targetIndex]);
    
    int autoSpeed = SHOOT_SPEED_TABLE[targetIndex];
    flyWheel.writeMicroseconds(autoSpeed);
    esc_l.writeMicroseconds(autoSpeed);
    esc_r.writeMicroseconds(autoSpeed);
}

void Shooter::fire(int speed) {
    flyWheel.writeMicroseconds(speed);
    esc_l.writeMicroseconds(speed);
    esc_r.writeMicroseconds(speed);
}

void Shooter::stop() {
    int s = RobotConfig::Chassis::PWM_STOP;
    flyWheel.writeMicroseconds(s);
    esc_l.writeMicroseconds(s);
    esc_r.writeMicroseconds(s);
    horMotor.writeMicroseconds(s);
}