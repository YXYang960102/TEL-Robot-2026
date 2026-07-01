#include "Vision.h"
#include "../Constants/VisionConstants.h"

using namespace VisionConst;

String Vision::rx = "";
double Vision::tx = 0;
double Vision::ty = 0;
double Vision::distance = 0;
int Vision::targetId = 0;
bool Vision::valid = false;

void Vision::init() {
    Serial1.begin(115200);
}

void Vision::update() {
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            parse(rx);
            rx = "";
        } else if (c != '\r') {
            if (rx.length() < 80) {
                rx += c;
            } else {
                rx = "";
            }
        }
    }
}

void Vision::parse(String s) {
    s.trim();
    if (s.length() == 0) return;

    String fields[5];
    int fieldCount = 0;
    int start = 0;

    while (fieldCount < 5) {
        int idx = s.indexOf(',', start);

        if (idx < 0) {
            fields[fieldCount++] = s.substring(start);
            break;
        }

        fields[fieldCount++] = s.substring(start, idx);
        start = idx + 1;
    }

    if (fieldCount < 1) return;

    double nextTx = fields[0].toFloat();
    double nextTy = fieldCount > 1 ? fields[1].toFloat() : 0;
    double nextDistance = fieldCount > 2 ? fields[2].toFloat() : 0;
    int nextTargetId = fieldCount > 3 ? fields[3].toInt() : 0;
    bool nextValid = fieldCount > 4 ? fields[4].toInt() == 1 : true;

    updatePrediction(nextTx, nextTy, nextDistance, nextTargetId, nextValid);
}

void Vision::updatePrediction(double nextTx, double nextTy, double nextDistance, int nextTargetId, bool nextValid) {
    valid = nextValid;

    if (valid) {
        tx = constrain(nextTx, -X_MAX, X_MAX);
        ty = nextTy;
        distance = nextDistance;
        targetId = nextTargetId;
    } else {
        tx = 0;
        ty = 0;
        distance = 0;
        targetId = 0;
    }
}

double Vision::getXPred() {
    return tx;
}

double Vision::getTx() {
    return tx;
}

double Vision::getTy() {
    return ty;
}

double Vision::getDistance() {
    return distance;
}

int Vision::getTargetId() {
    return targetId;
}

bool Vision::isValid() {
    return valid;
}
