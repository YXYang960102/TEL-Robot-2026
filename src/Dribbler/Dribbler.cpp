#include "Dribbler.h"
#include "../Constants/Pins.h"

int Dribbler::shoot_pice = 0;
bool Dribbler::sensorUp = false;

void Dribbler::init() {
    pinMode(PIN_DRIBBLE_UP, OUTPUT);
    pinMode(PIN_DRIBBLE_DOWN, OUTPUT);
}

void Dribbler::setShootRequest(int count) {
    shoot_pice = count;
}

int Dribbler::getShootRemaining() {
    return shoot_pice;
}

void Dribbler::update() {


    bool newBallDetected = digitalRead(PIN_DRIBBLE_DOWN) == LOW;

    if (newBallDetected && shoot_pice > 0) {
        shoot_pice--;
    }

    if (shoot_pice > 0) {
        run();
    } else {
        stop();
    }
}

void Dribbler::run() {
    digitalWrite(PIN_DRIBBLE_DOWN, HIGH);
    analogWrite(PIN_DRIBBLE_UP, 255);
}

void Dribbler::stop() {
    digitalWrite(PIN_DRIBBLE_DOWN, LOW);
    analogWrite(PIN_DRIBBLE_UP, 0);
}