#include "Vision.h"
#include "../Constants/VisionConstants.h"

using namespace VisionConst;

String Vision::rx = "";
double Vision::xPred = 0;

void Vision::init() {
    Serial1.begin(115200);
}

void Vision::update() {
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            parse(rx);
            rx = "";
        } else {
            rx += c;
        }
    }
}

void Vision::parse(String s) {
    int idx = s.indexOf(',');
    if (idx < 0) return;

    double x = s.substring(0, idx).toFloat();
    updatePrediction(x);
}

void Vision::updatePrediction(double x) {
    xPred = constrain(x, -X_MAX, X_MAX);
}

double Vision::getXPred() {
    return xPred;
}