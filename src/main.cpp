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

// Pin sanity check. Each pin is defined once, in its subsystem's Constants
// file; nothing here has to be edited when a pin changes. The build only
// fails if two outputs are set to the same pin, or an output lands on a
// serial port in use (0/1 USB, 16/17 Serial2 = ELRS bridge, 18/19 Serial1 =
// Orin), or an input lands on an output or on such a port. Serial3 (14/15)
// is never opened, so its pins would be plain GPIO too.
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
constexpr uint8_t OPEN_SERIAL_PINS[] = {0, 1, 16, 17, 18, 19}; // Serial3 unused
constexpr unsigned OUTPUT_COUNT = sizeof(OUTPUTS) / sizeof(OUTPUTS[0]);
constexpr unsigned INPUT_COUNT = sizeof(INPUTS) / sizeof(INPUTS[0]);
constexpr unsigned OPEN_SERIAL_COUNT = sizeof(OPEN_SERIAL_PINS) / sizeof(OPEN_SERIAL_PINS[0]);

// C++11 constexpr (single return, recursion) for the AVR toolchain.
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
// The tower lines are on the analog header (Jeremy, 2026-11-07).
static_assert(DribblerConstants::Tower::DISABLE_PIN == A15 && DribblerConstants::Tower::READY_PIN == A14,
              "Feed tower disable / ready lines are wired to A15 / A14");
}  // namespace pincheck

void setup() {
    Serial.begin(115200);

    MechLink::init();
    ManualControl::init();
    AimTrimStore::init(); // the saved aim / elevation trim (EEPROM)
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

    // ManualControl handles both operator sources (wired USB Dashboard and
    // the wireless ELRS channels MechLink just read) and applies its own
    // Chassis/Dribbler/Shooter calls while armed, before Auto::update(),
    // which steps aside whenever ManualControl::isActive().
    ManualControl::update();
    AimTrimStore::update(); // saves a new trim, one byte per loop

    // The operator's hole (keys 1-9 / CH3) -> the Orin's numbering. Before
    // Auto, so a switch drops the old hole's frame in this same loop.
    // Auto's aim target: the operator's hole, or in full auto the row
    // cycle's current one (2026-10-31; one loop behind Auto, harmless).
    Vision::setSelectedTarget(OperatorTarget::visionIdForHole(Auto::getAimTarget()));
    // P (2026-11-14): photo capture on the Orin, sent with the heartbeat.
    Vision::setCaptureRequested(ManualControl::isCaptureOn());

    // Rotate mode arbitration and the auto-fire sequence both live in
    // Auto::update() now (moved out of here 2026-10-06) so "what mode is
    // each subsystem in" is decided in one place.
    Auto::update();

    Shooter::update();
    Dribbler::update();
    Art::update(); // reacts to this loop's shot

    Chassis::update();
    Telemetry::update();

    delay(10);
}
