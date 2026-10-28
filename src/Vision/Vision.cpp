#include "Vision.h"
#include "../Constants/VisionConstants.h"
#include "OperatorTarget.h"

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
unsigned long Vision::frameCount = 0;
bool Vision::predicted = false;
unsigned long Vision::predictedSinceMs = 0;
uint16_t Vision::seenMask = 0;
int8_t Vision::ringPercent = -1;
uint8_t Vision::ringSectors = 0;
bool Vision::hasPacket = false;
bool Vision::sentReady = false;
unsigned long Vision::lastHeartbeatMs = 0;
uint8_t Vision::selectedTargetId = OperatorTarget::NONE;
bool Vision::selectionDirty = false;
Vision::OrinState Vision::orinState = Vision::OrinState::UNKNOWN;
unsigned long Vision::lastOrinStateMs = 0;

void Vision::init() {
    Serial1.begin(Transport::SERIAL_BAUD);
    rx.reserve(Transport::PACKET_MAX_CHARS);
}

void Vision::update() {
    sendHeartbeat();
    if (selectionDirty && sentReady) {
        sendSelection(); // a new choice goes out now, not at the next heartbeat
    }

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
        seenMask = 0;
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
        sendSelection();
        return;
    }

    if (now - lastHeartbeatMs < Transport::HEARTBEAT_INTERVAL_MS) {
        return;
    }
    lastHeartbeatMs = now;

    Serial1.print("MEGA_HEARTBEAT,");
    Serial1.print(Transport::PROTOCOL_VERSION);
    Serial1.print('\n');
    // Resent with every heartbeat (100 ms): the Orin keeps its last valid
    // choice, so a restarted Orin has the current one again within 100 ms.
    sendSelection();
}

void Vision::sendSelection() {
    Serial1.print("MEGA_TARGET,");
    Serial1.print(Transport::PROTOCOL_VERSION);
    Serial1.print(',');
    Serial1.print(selectedTargetId);
    Serial1.print('\n');
    selectionDirty = false;
}

void Vision::setSelectedTarget(uint8_t visionId) {
    if (visionId == selectedTargetId) {
        return;
    }
    selectedTargetId = visionId;
    selectionDirty = true;
    predictedSinceMs = 0; // a new choice gets its own prediction budget
    // The frame in hand is for the old choice (or the Orin's own pick):
    // never aim at it after a switch. The Orin's next frame for the new
    // hole arrives within a camera cycle or two.
    invalidateTarget();
}

bool Vision::isPredicted() {
    return predicted;
}

uint16_t Vision::getSeenMask() {
    return seenMask;
}

int8_t Vision::getRingPercent() {
    return isValid() ? ringPercent : static_cast<int8_t>(-1);
}

uint8_t Vision::getRingSectors() {
    return isValid() ? ringSectors : 0;
}

unsigned long Vision::getFrameCount() {
    return frameCount;
}

uint8_t Vision::getSelectedTarget() {
    return selectedTargetId;
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
    // v3: tx,ty,distance,target_id,valid,seen_mask -- optionally followed
    // (2026-10-28) by ring_pct,ring_sectors, the depth ring's coverage
    if (commaCount != 5 && commaCount != 7) return false;

    String fields[8];
    int fieldCount = 0;
    int start = 0;

    while (fieldCount < 8 && start <= static_cast<int>(s.length())) {
        int idx = s.indexOf(',', start);

        if (idx < 0) {
            fields[fieldCount++] = s.substring(start);
            break;
        }

        fields[fieldCount++] = s.substring(start, idx);
        start = idx + 1;
    }

    if (fieldCount != commaCount + 1) return false;

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
    if (end == fields[4].c_str() || *end != '\0' || validValue < 0 || validValue > 2) {
        return false;
    }

    const long mask = strtol(fields[5].c_str(), &end, 10);
    if (end == fields[5].c_str() || *end != '\0' || mask < 0 || mask > Validation::MAX_SEEN_MASK) {
        return false;
    }

    long nextRingPct = -1;
    long nextRingSectors = 0;
    if (fieldCount == 8) {
        nextRingPct = strtol(fields[6].c_str(), &end, 10);
        if (end == fields[6].c_str() || *end != '\0' || nextRingPct < -1 || nextRingPct > 100) return false;
        nextRingSectors = strtol(fields[7].c_str(), &end, 10);
        if (end == fields[7].c_str() || *end != '\0' || nextRingSectors < 0 || nextRingSectors > 0xFF) return false;
    }

    const double maxAbsTx = validValue == 2 ? Validation::MAX_ABS_PREDICTED_TX : Validation::MAX_ABS_TX;
    if (abs(nextTx) > maxAbsTx ||
        abs(nextTy) > Validation::MAX_ABS_TY ||
        nextDistance < 0 ||
        nextDistance > Validation::MAX_DISTANCE_MM ||
        nextTargetId < Validation::MIN_TARGET_ID ||
        nextTargetId > Validation::MAX_TARGET_ID) {
        return false;
    }

    storeTarget(
        nextTx,
        nextTy,
        nextDistance,
        static_cast<int>(nextTargetId),
        static_cast<int>(validValue),
        static_cast<uint16_t>(mask));
    // The ring is about the frame's own target: only kept for a real
    // sighting of the hole the Mega aims at (isValid()).
    ringPercent = valid ? static_cast<int8_t>(nextRingPct) : static_cast<int8_t>(-1);
    ringSectors = valid && nextRingPct >= 0 ? static_cast<uint8_t>(nextRingSectors) : 0;
    return true;
}

// Stores one parsed frame (no prediction/extrapolation happens here).
void Vision::storeTarget(
    double nextTx,
    double nextTy,
    double nextDistance,
    int nextTargetId,
    int validCode,
    uint16_t nextSeenMask) {
    const unsigned long now = millis();
    lastPacketMs = now;
    hasPacket = true;
    frameCount++;
    seenMask = nextSeenMask;
    // A frame for a hole other than the selected one is "no target" (the
    // link itself is still alive: isConnected() stays true).
    valid = validCode == 1 && OperatorTarget::frameMatchesSelection(selectedTargetId, nextTargetId);

    // v3 prediction: only for the hole the operator chose, and for at most
    // PREDICTED_FOLLOW_MAX_MS from the first prediction. Only a real
    // sighting or a new selection restarts that budget -- "nothing" frames
    // in between don't, so an on / off stream can't turn the turret forever.
    const bool nextPredicted = validCode == 2 && selectedTargetId != OperatorTarget::NONE &&
                               nextTargetId == selectedTargetId;
    predicted = false;
    if (valid) {
        predictedSinceMs = 0;
    } else if (nextPredicted) {
        if (predictedSinceMs == 0) predictedSinceMs = now == 0 ? 1 : now;
        if (now - predictedSinceMs <= Transport::PREDICTED_FOLLOW_MAX_MS) {
            predicted = true;
            tx = constrain(nextTx, -Validation::MAX_ABS_TX, Validation::MAX_ABS_TX);
            ty = nextTy;
            distance = 0.0;
            targetId = nextTargetId;
            return;
        }
    }

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
    predicted = false;
    ringPercent = -1;
    ringSectors = 0;
    tx = 0;
    ty = 0;
    distance = 0;
    targetId = 0;
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
