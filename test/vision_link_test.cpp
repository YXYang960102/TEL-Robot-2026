// Host test for the REAL src/Vision/Vision.cpp (Orin link, protocol v2):
// MEGA_READY/HEARTBEAT,2, MEGA_TARGET,2,<vision id> on change and with every
// heartbeat, v1 lifecycle lines refused, and -- the safety part -- while a
// hole is selected only that hole's frames are valid; a switch drops the
// frame in hand. Uses test/vision_stubs/Arduino.h, not test/host_stubs.
#include <assert.h>
#include <stdio.h>

#include <string>

#include "../src/Vision/OperatorTarget.h"
#include "../src/Vision/Vision.h"

namespace visionstub {
unsigned long nowMs = 0;
}
FakeSerial1 Serial1;

namespace {

void orin(const std::string& line) { Serial1.input += line + "\n"; }

void tick(unsigned long ms = 10) {
    visionstub::nowMs += ms;
    Vision::update();
}

size_t count(const std::string& needle) {
    size_t n = 0;
    for (size_t i = Serial1.output.find(needle); i != std::string::npos;
         i = Serial1.output.find(needle, i + 1)) n++;
    return n;
}

}  // namespace

int main() {
    visionstub::nowMs = 1000;
    Vision::init();
    tick();
    // First call: READY, then the (empty) selection right away.
    assert(Serial1.output == "MEGA_READY,2\nMEGA_TARGET,2,0\n");

    // Heartbeat every 100 ms, each followed by the selection.
    Serial1.output.clear();
    for (int i = 0; i < 30; i++) tick(10);
    assert(count("MEGA_HEARTBEAT,2\n") == 3 && count("MEGA_TARGET,2,0\n") == 3);

    // Lifecycle: v2 accepted, v1 (an old Orin) refused.
    orin("VISION_READY,1");
    tick();
    assert(!Vision::isVisionReady());
    orin("VISION_READY,2");
    tick();
    assert(Vision::isVisionReady());

    // No selection: the Orin's own pick (any id) is used.
    orin("12.5,-3,4200,11,1");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 11 && Vision::getDistance() == 4200);

    // Select key 1 (left-bottom) -> vision 10: sent at once, not at the next
    // heartbeat, and the frame in hand (hole 11) is dropped immediately.
    Serial1.output.clear();
    Vision::setSelectedTarget(OperatorTarget::visionIdForHole(1));
    assert(!Vision::isValid() && Vision::getTx() == 0 && Vision::isConnected());
    tick(1);
    assert(Serial1.output == "MEGA_TARGET,2,10\n");
    assert(Vision::getSelectedTarget() == 10);
    // Setting the same choice again sends nothing extra.
    Serial1.output.clear();
    Vision::setSelectedTarget(10);
    tick(1);
    assert(Serial1.output.empty());

    // A frame for another hole (an Orin that hasn't switched yet, or a
    // bonus hole) is "no target"; the link itself stays connected.
    orin("8,2,3900,11,1");
    tick();
    assert(!Vision::isValid() && Vision::isConnected() && Vision::getTargetId() == 0);
    orin("8,2,3900,2,1");
    tick();
    assert(!Vision::isValid());
    // The selected hole's frame is used.
    orin("-20,5,3500,10,1");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 10 && Vision::getTx() == -20 &&
           Vision::getDistance() == 3500);
    // The Orin's "not seen": 0,0,0,0,0 -> no target.
    orin("0,0,0,0,0");
    tick();
    assert(!Vision::isValid());
    orin("-20,5,3500,10,1");
    tick();
    assert(Vision::isValid());
    // Stale: no frame for > 300 ms -> no target.
    for (int i = 0; i < 35; i++) tick(10);
    assert(!Vision::isValid() && !Vision::isConnected());

    // The selection rides every heartbeat, so a restarted Orin gets it back.
    Serial1.output.clear();
    for (int i = 0; i < 20; i++) tick(10);
    assert(count("MEGA_TARGET,2,10\n") == 2);

    // Back to none: the Orin's pick is accepted again.
    Vision::setSelectedTarget(OperatorTarget::NONE);
    orin("1,1,5000,5,1");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 5);

    printf("vision_link_test: all asserts passed\n");
    return 0;
}
