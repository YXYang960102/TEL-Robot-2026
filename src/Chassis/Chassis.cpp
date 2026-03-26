#include "Chassis.h"
#include <Arduino.h>
#include "../Constants/RobotConstants.h"

void Chassis::begin(int fr_pin, int br_pin, int fl_pin, int bl_pin) {
    using namespace RobotConfig::Chassis;
    motor_fr.attach(fr_pin, PWM_MIN, PWM_MAX);
    motor_br.attach(br_pin, PWM_MIN, PWM_MAX);
    motor_fl.attach(fl_pin, PWM_MIN, PWM_MAX);
    motor_bl.attach(bl_pin, PWM_MIN, PWM_MAX);
}

void Chassis::drive(int x, int y, int rotate) {
    int stopPoint = RobotConfig::Chassis::PWM_STOP;
    motor_fr.write(x - y + stopPoint + rotate);
    motor_br.write(x - y + stopPoint - rotate);
    motor_fl.write(x + y - stopPoint - rotate);
    motor_bl.write(x + y - stopPoint + rotate);
}

void Chassis::stop() {
    using namespace RobotConfig::Chassis;
    motor_fr.writeMicroseconds(PWM_STOP);
    motor_br.writeMicroseconds(PWM_STOP);
    motor_fl.writeMicroseconds(PWM_STOP);
    motor_bl.writeMicroseconds(PWM_STOP);
}