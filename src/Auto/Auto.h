#pragma once

#include <stdint.h>

#include "../Control/ShotTableLookup.h"
#include "../IO/OperatorMode.h"

class Auto {
public:
    static void init();
    static void update();

    static double getShotDistanceM();
    static uint8_t getReadyBits();
    enum FireBlocker : uint16_t {
        BLOCK_TURRET = 1u << 0,
        BLOCK_FLYWHEEL = 1u << 1,
        BLOCK_NO_SHOT = 1u << 2,
        BLOCK_ELEVATION = 1u << 3,
        BLOCK_BALL = 1u << 4,       // tower: no ball (A14)
        BLOCK_SWITCHING = 1u << 5,
        BLOCK_PATH = 1u << 6,
        BLOCK_HOLE = 1u << 7,
        BLOCK_COOLDOWN = 1u << 8,
        BLOCK_DONE = 1u << 9,
        BLOCK_NO_CAMERA = 1u << 10,
    };
    static uint16_t getFireBlockers();

    static uint8_t getAimTarget();
    static uint8_t getRowShots();
    static uint8_t getAimShots();
    static bool isRowDone();
    static uint8_t getLitPriorityHole();
    static bool getTableElevationUs(double& us);

private:
    enum class FireState {
        IDLE,
        SPINNING_UP,
        READY,
        COOLDOWN
    };

    static void enterFireState(FireState next);
    static void updateRowCycle(OperatorMode mode);
    static void countRowShot();
    static void finishHoleSwitch();
    static void skipHoleNotFound(unsigned long now);
    static void skipHolesWithoutTable();
    static void startCycle(uint8_t hole);
    static uint8_t shotsForHole(uint8_t hole);

    static bool rowActive;
    static uint8_t rowStartHole;
    static uint8_t farColumn;
    static uint8_t rowStep;
    static uint8_t cycleHoles[12];
    static uint8_t cycleLength;
    static uint8_t rowShots;
    static bool rowDone;
    static uint8_t lastCycleHole;
    static uint16_t rowSelectSeq;
    static bool litActive;
    static uint8_t litHole;
    static uint8_t litShots;
    static uint8_t litServedMask; // bit vision id 1-3: served, waiting to go dark
    static bool holeSwitchPending;
    static bool holeSwitchFromLit;
    static unsigned long holeSwitchStartMs;
    static uint8_t notFoundAim;
    static unsigned long notFoundSinceMs;
    static unsigned long litLastSeenMs[4];
    static uint8_t lastAimTarget;
    static unsigned long aimChangedMs;
    static OperatorMode rowMode;
    static bool rowSecondHalf;
    static bool pathStartUsed;

    static FireState fireState;
    static unsigned long fireStateEnteredMs;
    static bool haveLastShot;
    static double lastShotFlywheel;
    static double lastShotElevationUs;
    static unsigned long lastShotMs;
    static uint8_t lastShotSelection;
    static unsigned long lastElevationSlewMs;
    static int lastShotTargetId;
    static uint8_t readyBits;
    static uint16_t fireBlockers;
    static bool semiFirePending;
    static unsigned long semiFireUntilMs;
    static double keptFlywheel;
    static double keptElevationUs;
    static double keptDistanceM;
    static uint8_t keptHole;
    static bool noTargetShot(ShotTableLookup::Result& out);
    static unsigned long elevationArrivedSinceMs;
    static bool slewElevation(double targetUs, unsigned long now);
};
