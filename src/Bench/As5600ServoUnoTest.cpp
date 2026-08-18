#include <Arduino.h>
#include "As5600ServoTestSubsystem.h"

namespace {

As5600ServoTestSubsystem testSubsystem;

}  // namespace

void setup() {
    Serial.begin(115200);

    As5600ServoTestConfig config;
    testSubsystem.begin(config);
}

void loop() {
    testSubsystem.update();
    delay(20);
}
