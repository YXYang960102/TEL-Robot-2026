#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <Arduino.h>

namespace RobotConfig {

    // Chassis Drive System
    namespace Chassis {
        const int PIN_FR = 39; // Front Right
        const int PIN_BR = 40; // Back Right
        const int PIN_FL = 38; // Front Left
        const int PIN_BL = 41; // Back Left
        
        // PWM Motor Output Range
        const int PWM_MIN = 1000;
        const int PWM_MAX = 2000;
        const int PWM_STOP = 1500;

       
        const int SBUS_MIN = 172;
        const int SBUS_MAX = 1811;

        const int DEADZONE = 30; // deeadband
    }

    // Vision System
    namespace Vision {
        const long BAUD_RATE = 115200;

        const double PREDICTION_TD = 0.08;     
        const double FILTER_ALPHA = 0.7;      
        const double PREDICTION_WEIGHT = 0.3; 
    }

    // Shooter System
    namespace Shooter {
        const int PIN_HOR_MOTOR = 11;      // Rotation Motor
        const int PIN_VER_MOTOR = 12;      // Vertical Angle Motor
        const int PIN_FALCON = 8;          // Main Fire Motor (Falcon)
        const int PIN_FLYWHEEL_L = 9;      // Left Fire Wheel (ESC1)
        const int PIN_FLYWHEEL_R = 10;     // Right Fire Wheel (ESC2)

        // Rotation PID
        const double HOR_KP = 1.35;
        const double HOR_KI = 0.0;
        const double HOR_KD = 0.05;

        const float DEG_PER_COUNT = 360.0f / 4096.0f;
    }

    // Dribbler System
    namespace Dribbler {
        const int PIN_MOTOR = 44;          // Dribbler ESC
        const int PIN_SENSOR = 22;         // Dribbler Sensor (Infrared)
        const int SPEED_INTAKE = 1800;     // Intake PWM
        const int SPEED_STOP = 1500;       // Stop PWM
        const int SPEED_OUT = 1200;        // Outtake PWM
    }

    // Auto-aiming & Dashboard
    namespace Auto {
       
        const int TABLE_SIZE = 7;
        const double Y_RANGE_TABLE[] = {110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0};
        
        const int VER_ANGLE_TABLE[] = {2500, 2400, 2300, 2200, 2100, 2000, 1900};
        
        const int SHOOT_SPEED_TABLE[] = {1600, 1650, 1700, 1750, 1800, 1850, 1900};
        
        const int PIN_ULTRASONIC_TRIG = 36;
        const int PIN_ULTRASONIC_ECHO = 37;
    }

} 

#endif