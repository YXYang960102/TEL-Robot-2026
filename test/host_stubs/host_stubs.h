#pragma once

#include <Arduino.h>

#include <string>

namespace hoststub {

// What the fake subsystems were last told to do.
struct Recorder {
    bool chassisEnabled = false;
    double chassisLeft = 0.0;
    double chassisRight = 0.0;
    bool shooterEnabled = false;
    std::string rotateMode = "disabled";
    double rotateCommand = 0.0;
    double angleCommand = 0.0;
    double flywheelCommand = 0.0;
    double angleTargetDegrees = 135.0; // ANGT (stub keeps no clamp)
    unsigned long angleTargetSets = 0;
    double anglePulseUs = 0.0;         // Auto's shot-table pulse
    unsigned long anglePulseSets = 0;
    double aimOffsetPx = 0.0;          // Shooter::setRotateAimOffsetPx
    unsigned long statusCount = 0;
    uint8_t statusFlags = 0;
    unsigned long statusSeq = 0;
    uint16_t statusCode = 0;
    unsigned long aimCount = 0;        // aim frames (code bit 15), kept apart
    uint8_t aimFlags = 0;
    unsigned long aimSeq = 0;
    uint16_t aimCode = 0;
    unsigned long visionCount = 0;     // vision frames (code bits 15-14 = 01)
    uint8_t visionFlags = 0;
    unsigned long visionSeq = 0;
    uint16_t visionCode = 0;
    unsigned long seenCount = 0;       // seen frames (code bits 15-14 = 11)
    uint16_t seenCode = 0;
};

extern Recorder rec;
extern bool visionReady;      // Vision::isConnected()/isVisionReady()
extern bool visionValid;      // Vision::isValid() (a target is locked)
extern double visionDistanceMm;
extern int visionTargetId;    // Vision::getTargetId() (Orin vision id)
extern uint8_t visionSelectedId; // Vision::getSelectedTarget()
extern double visionTx;          // Vision::getTx()
extern double visionTy;          // Vision::getTy()
extern int anglePulseUs;      // Shooter::getAnglePulseUs()
extern int flywheelPulseUs;   // Shooter::getFlywheelPulseUs()
extern bool shooterReady;     // Shooter::isReady() (turret locked && flywheel up)

}
