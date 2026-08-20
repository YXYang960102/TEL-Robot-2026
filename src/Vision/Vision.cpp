#include "Vision.h"
#include "../Constants/VisionConstants.h"

#include <stdlib.h>

using namespace VisionConst;

String Vision::rx = "";
double Vision::tx = 0;
double Vision::ty = 0;
double Vision::distance = 0;
int Vision::targetId = 0;
bool Vision::valid = false;
unsigned long Vision::lastPacketMs = 0;
bool Vision::hasPacket = false;

void Vision::init() {
    Serial1.begin(SERIAL_BAUD);
    rx.reserve(PACKET_MAX_CHARS);
}

void Vision::update() {
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            if (!parse(rx)) {
                invalidateTarget();
            }
            rx = "";
        } else if (c != '\r') {
            if (rx.length() < PACKET_MAX_CHARS) {
                rx += c;
            } else {
                rx = "";
                invalidateTarget();
            }
        }
    }

    if (hasPacket && millis() - lastPacketMs > PACKET_TIMEOUT_MS) {
        invalidateTarget();
    }
}

bool Vision::parse(const String& input) {
    String s = input;
    s.trim();
    if (s.length() == 0) return false;

    int commaCount = 0;
    for (unsigned int i = 0; i < s.length(); i++) {
        if (s.charAt(i) == ',') {
            commaCount++;
        }
    }
    if (commaCount != 4) return false;

    String fields[5];
    int fieldCount = 0;
    int start = 0;

    while (fieldCount < 5 && start <= static_cast<int>(s.length())) {
        int idx = s.indexOf(',', start);

        if (idx < 0) {
            fields[fieldCount++] = s.substring(start);
            break;
        }

        fields[fieldCount++] = s.substring(start, idx);
        start = idx + 1;
    }

    if (fieldCount != 5) return false;

    char* end = nullptr;
    const double nextTx = strtod(fields[0].c_str(), &end);
    if (end == fields[0].c_str() || *end != '\0' || !isfinite(nextTx)) return false;

    const double nextTy = strtod(fields[1].c_str(), &end);
    if (end == fields[1].c_str() || *end != '\0' || !isfinite(nextTy)) return false;

    const double nextDistance = strtod(fields[2].c_str(), &end);
    if (end == fields[2].c_str() || *end != '\0' || !isfinite(nextDistance)) return false;

    const long nextTargetId = strtol(fields[3].c_str(), &end, 10);
    if (end == fields[3].c_str() || *end != '\0') return false;

    const long validValue = strtol(fields[4].c_str(), &end, 10);
    if (end == fields[4].c_str() || *end != '\0' || (validValue != 0 && validValue != 1)) {
        return false;
    }

    if (abs(nextTx) > X_MAX || abs(nextTy) > TY_MAX || nextDistance < 0 ||
        nextDistance > DISTANCE_MAX_MM || nextTargetId < TARGET_ID_MIN ||
        nextTargetId > TARGET_ID_MAX) {
        return false;
    }

    updatePrediction(nextTx, nextTy, nextDistance, static_cast<int>(nextTargetId), validValue == 1);
    return true;
}

void Vision::updatePrediction(double nextTx, double nextTy, double nextDistance, int nextTargetId, bool nextValid) {
    lastPacketMs = millis();
    hasPacket = true;
    valid = nextValid;

    if (valid) {
        tx = constrain(nextTx, -X_MAX, X_MAX);
        ty = nextTy;
        distance = nextDistance;
        targetId = nextTargetId;
    } else {
        invalidateTarget();
    }
}

void Vision::invalidateTarget() {
    valid = false;
    tx = 0;
    ty = 0;
    distance = 0;
    targetId = 0;
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
    return valid && isConnected();
}

bool Vision::isConnected() {
    return hasPacket && millis() - lastPacketMs <= PACKET_TIMEOUT_MS;
}

unsigned long Vision::getPacketAgeMs() {
    return hasPacket ? millis() - lastPacketMs : 0xFFFFFFFFUL;
}
