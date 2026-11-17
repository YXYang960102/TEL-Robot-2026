#include <Arduino.h>

#include "Auto/Auto.h"
#include "IO/AimTrimStore.h"
#include "IO/MechLink.h"
#include "IO/ManualControl.h"
#include "Vision/OperatorTarget.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"
#include "Art/Art.h"
#include "Chassis/Chassis.h"
#include "Telemetry/Telemetry.h"
#include "Sensors/TurretAngle.h"
#include "Constants/ArtConstants.h"
#include "Constants/ChassisConstants.h"
#include "Constants/DribblerConstants.h"
#include "Constants/ShooterConstants.h"

namespace pincheck {
constexpr uint8_t OUTPUTS[] = {
    ChassisConstants::LeftDrive::SIGNAL_PIN,
    ChassisConstants::RightDrive::SIGNAL_PIN,
    DribblerConstants::Tower::TRIGGER_PIN,
    DribblerConstants::Tower::DISABLE_PIN,
    ShooterConstants::Angle::ANGLE_PIN,
    ShooterConstants::Rotate::SIGNAL_PIN,
    ShooterConstants::Flywheel::SIGNAL_PIN,
    ArtConstants::Link::SIGNAL_PIN,
    ArtConstants::Sound::WAVE_PIN,
    ArtConstants::Sound::LAUGH_PIN,
    ArtConstants::Sound::READY_PIN,
};
constexpr uint8_t INPUTS[] = {
    DribblerConstants::Tower::READY_PIN,
};
constexpr uint8_t OPEN_SERIAL_PINS[] = {0, 1, 16, 17, 18, 19};
constexpr unsigned OUTPUT_COUNT = sizeof(OUTPUTS) / sizeof(OUTPUTS[0]);
constexpr unsigned INPUT_COUNT = sizeof(INPUTS) / sizeof(INPUTS[0]);
constexpr unsigned OPEN_SERIAL_COUNT = sizeof(OPEN_SERIAL_PINS) / sizeof(OPEN_SERIAL_PINS[0]);

constexpr bool notIn(uint8_t pin, const uint8_t* list, unsigned count) {
    return count == 0 || (list[0] != pin && notIn(pin, list + 1, count - 1));
}
constexpr bool allDistinct(const uint8_t* list, unsigned count) {
    return count == 0 ||
           (notIn(list[0], list + 1, count - 1) && allDistinct(list + 1, count - 1));
}
constexpr bool noneIn(const uint8_t* list, unsigned count, const uint8_t* reserved, unsigned reservedCount) {
    return count == 0 ||
           (notIn(list[0], reserved, reservedCount) &&
            noneIn(list + 1, count - 1, reserved, reservedCount));
}

static_assert(allDistinct(OUTPUTS, OUTPUT_COUNT),
              "Two outputs share a pin -- check the pin values in Chassis/Dribbler/ShooterConstants.h");
static_assert(noneIn(OUTPUTS, OUTPUT_COUNT, OPEN_SERIAL_PINS, OPEN_SERIAL_COUNT),
              "An output is on a serial port in use (0/1, 16-19)");
static_assert(noneIn(INPUTS, INPUT_COUNT, OUTPUTS, OUTPUT_COUNT) &&
                  noneIn(INPUTS, INPUT_COUNT, OPEN_SERIAL_PINS, OPEN_SERIAL_COUNT),
              "An input shares a pin with an output or a serial port in use");
static_assert(DribblerConstants::Tower::DISABLE_PIN == A15 && DribblerConstants::Tower::READY_PIN == A14,
              "Feed tower disable / ready lines are wired to A15 / A14");
}

void setup() {
    Serial.begin(115200);

    MechLink::init();
    ManualControl::init();
    AimTrimStore::init();
    Vision::init();
    Shooter::init();
    Dribbler::init();
    Art::init();
    Chassis::init();
    Telemetry::init();
    Auto::init();
    TurretAngle::init(); // AS5600 on I2C D20/D21: read only for now
}

void loop() {

    MechLink::update();
    Vision::update();
    TurretAngle::update();

    ManualControl::update();
    AimTrimStore::update();

    Vision::setSelectedTarget(OperatorTarget::visionIdForHole(Auto::getAimTarget()));
    Vision::setCaptureRequested(ManualControl::isCaptureOn());

    Auto::update();

    Shooter::update();
    Dribbler::update();
    Art::update();

    Chassis::update();
    Telemetry::update();

    delay(10);
}
