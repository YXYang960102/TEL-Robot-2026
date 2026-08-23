#include "Vision.h"
#include "../Constants/VisionConstants.h"

#include <stdlib.h>
#include <string.h>

using namespace VisionConstants;

String Vision::rx = "";
double Vision::tx = 0;
double Vision::ty = 0;
double Vision::distance = 0;
int Vision::targetId = 0;
bool Vision::valid = false;
unsigned long Vision::lastPacketMs = 0;
bool Vision::hasPacket = false;
bool Vision::sentReady = false;
unsigned long Vision::lastHeartbeatMs = 0;
Vision::OrinState Vision::orinState = Vision::OrinState::UNKNOWN;
unsigned long Vision::lastOrinStateMs = 0;

void Vision::init() {
    Serial1.begin(Transport::SERIAL_BAUD);
    rx.reserve(Transport::PACKET_MAX_CHARS);
}

void Vision::update() {
    sendHeartbeat();

    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            if (rx.startsWith("VISION_")) {
                parseOrinControl(rx);
            } else if (!parse(rx)) {
                invalidateTarget();
            }
            rx = "";
        } else if (c != '\r') {
            if (rx.length() < Transport::PACKET_MAX_CHARS) {
                rx += c;
            } else {
                rx = "";
                invalidateTarget();
            }
        }
    }

    if (hasPacket && millis() - lastPacketMs > Transport::PACKET_TIMEOUT_MS) {
        invalidateTarget();
    }

    if (orinState != OrinState::UNKNOWN &&
        millis() - lastOrinStateMs > Transport::PACKET_TIMEOUT_MS) {
        orinState = OrinState::UNKNOWN;
    }
}

void Vision::sendHeartbeat() {
    const unsigned long now = millis();

    if (!sentReady) {
        Serial1.print("MEGA_READY,");
        Serial1.print(Transport::PROTOCOL_VERSION);
        Serial1.print('\n');
        sentReady = true;
        lastHeartbeatMs = now;
        return;
    }

    if (now - lastHeartbeatMs < Transport::HEARTBEAT_INTERVAL_MS) {
        return;
    }
    lastHeartbeatMs = now;

    Serial1.print("MEGA_HEARTBEAT,");
    Serial1.print(Transport::PROTOCOL_VERSION);
    Serial1.print('\n');
}

namespace {

struct OrinLifecycleToken {
    const char* prefix;
    Vision::OrinState state;
};

const OrinLifecycleToken kOrinLifecycleTokens[] = {
    {"VISION_STANDBY,", Vision::OrinState::STANDBY},
    {"VISION_STARTING,", Vision::OrinState::STARTING},
    {"VISION_READY,", Vision::OrinState::READY},
    {"VISION_ERROR,", Vision::OrinState::ERROR},
};

}  // namespace

bool Vision::parseOrinControl(const String& input) {
    String s = input;
    s.trim();

    for (const OrinLifecycleToken& token : kOrinLifecycleTokens) {
        if (!s.startsWith(token.prefix)) {
            continue;
        }

        const int prefixLen = strlen(token.prefix);
        const int versionEnd = s.indexOf(',', prefixLen);
        const String versionField = versionEnd < 0
            ? s.substring(prefixLen)
            : s.substring(prefixLen, versionEnd);

        char* end = nullptr;
        const long version = strtol(versionField.c_str(), &end, 10);
        if (end == versionField.c_str() || *end != '\0' ||
            version != Transport::PROTOCOL_VERSION) {
            return false;
        }

        orinState = token.state;
        lastOrinStateMs = millis();
        return true;
    }

    return false;
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

    if (abs(nextTx) > Validation::MAX_ABS_TX ||
        abs(nextTy) > Validation::MAX_ABS_TY ||
        nextDistance < 0 ||
        nextDistance > Validation::MAX_DISTANCE_MM ||
        nextTargetId < Validation::MIN_TARGET_ID ||
        nextTargetId > Validation::MAX_TARGET_ID) {
        return false;
    }

    updatePrediction(
        nextTx,
        nextTy,
        nextDistance,
        static_cast<int>(nextTargetId),
        validValue == 1);
    return true;
}

void Vision::updatePrediction(
    double nextTx,
    double nextTy,
    double nextDistance,
    int nextTargetId,
    bool nextValid) {
    lastPacketMs = millis();
    hasPacket = true;
    valid = nextValid;

    if (valid) {
        tx = constrain(
            nextTx,
            -Validation::MAX_ABS_TX,
            Validation::MAX_ABS_TX);
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
    return hasPacket &&
           millis() - lastPacketMs <= Transport::PACKET_TIMEOUT_MS;
}

unsigned long Vision::getPacketAgeMs() {
    return hasPacket ? millis() - lastPacketMs : 0xFFFFFFFFUL;
}

Vision::OrinState Vision::getOrinState() {
    return orinState;
}

bool Vision::isVisionReady() {
    return orinState == OrinState::READY &&
           millis() - lastOrinStateMs <= Transport::PACKET_TIMEOUT_MS;
}
