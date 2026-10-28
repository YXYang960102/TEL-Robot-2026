// Host test for the REAL src/Vision/Vision.cpp (Orin link, protocol v3 since
// 2026-10-27; v2 before): MEGA_READY/HEARTBEAT,3, MEGA_TARGET,3,<vision id> on change and with every
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
    assert(Serial1.output == "MEGA_READY,3\nMEGA_TARGET,3,0\n");

    // Heartbeat every 100 ms, each followed by the selection.
    Serial1.output.clear();
    for (int i = 0; i < 30; i++) tick(10);
    assert(count("MEGA_HEARTBEAT,3\n") == 3 && count("MEGA_TARGET,3,0\n") == 3);

    // Lifecycle: v3 accepted, v2 (an older Orin) refused.
    orin("VISION_READY,2");
    tick();
    assert(!Vision::isVisionReady());
    orin("VISION_READY,3");
    tick();
    assert(Vision::isVisionReady());

    // No selection: the Orin's own pick (any id) is used.
    orin("12.5,-3,4200,11,1,0");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 11 && Vision::getDistance() == 4200);

    // Select key 1 (left-bottom) -> vision 10: sent at once, not at the next
    // heartbeat, and the frame in hand (hole 11) is dropped immediately.
    Serial1.output.clear();
    Vision::setSelectedTarget(OperatorTarget::visionIdForHole(1));
    assert(!Vision::isValid() && Vision::getTx() == 0 && Vision::isConnected());
    tick(1);
    assert(Serial1.output == "MEGA_TARGET,3,10\n");
    assert(Vision::getSelectedTarget() == 10);
    // Setting the same choice again sends nothing extra.
    Serial1.output.clear();
    Vision::setSelectedTarget(10);
    tick(1);
    assert(Serial1.output.empty());

    // A frame for another hole (an Orin that hasn't switched yet, or a
    // bonus hole) is "no target"; the link itself stays connected.
    orin("8,2,3900,11,1,0");
    tick();
    assert(!Vision::isValid() && Vision::isConnected() && Vision::getTargetId() == 0);
    orin("8,2,3900,2,1,0");
    tick();
    assert(!Vision::isValid());
    // The selected hole's frame is used.
    orin("-20,5,3500,10,1,0");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 10 && Vision::getTx() == -20 &&
           Vision::getDistance() == 3500);
    // The Orin's "not seen": 0,0,0,0,0 -> no target.
    orin("0,0,0,0,0,0");
    tick();
    assert(!Vision::isValid());
    orin("-20,5,3500,10,1,0");
    tick();
    assert(Vision::isValid());
    // Stale: no frame for > 300 ms -> no target.
    for (int i = 0; i < 35; i++) tick(10);
    assert(!Vision::isValid() && !Vision::isConnected());

    // The selection rides every heartbeat, so a restarted Orin gets it back.
    Serial1.output.clear();
    for (int i = 0; i < 20; i++) tick(10);
    assert(count("MEGA_TARGET,3,10\n") == 2);

    // Back to none: the Orin's pick is accepted again.
    Vision::setSelectedTarget(OperatorTarget::NONE);
    orin("1,1,5000,5,1,0");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 5);

    // ---- v3 (2026-10-27) ----
    // A v2 frame (five fields) is refused now, and like any malformed line
    // it drops the target in hand.
    orin("2,2,3000,5,1");
    tick();
    assert(!Vision::isValid() && Vision::getTx() == 0);
    // The detected-holes mask rides every frame.
    orin("1,1,5000,5,1,1176");
    tick();
    assert(Vision::getSeenMask() == 1176 && !Vision::isPredicted());
    // Bonus hole selected (vision id 2): its frames are used.
    Vision::setSelectedTarget(OperatorTarget::visionIdForHole(11));
    tick(1);
    assert(count("MEGA_TARGET,3,2\n") >= 1);
    orin("-30,-200,2900,2,1,2");
    tick();
    assert(Vision::isValid() && Vision::getTargetId() == 2);
    // Seen without depth (valid 1, distance 0): still valid, no distance.
    orin("-30,-200,0,2,1,2");
    tick();
    assert(Vision::isValid() && Vision::getDistance() == 0);
    // Predicted (valid 2): aim only -- not valid, tx clamped to 400, no
    // distance; only for the selected hole.
    orin("760,-40,0,2,2,48");
    tick();
    assert(!Vision::isValid() && Vision::isPredicted() && Vision::getTx() == 400 &&
           Vision::getDistance() == 0 && Vision::getSeenMask() == 48);
    orin("760,-40,0,5,2,48");                           // not the selected hole
    tick();
    assert(!Vision::isPredicted());
    // Budget: predictions stop counting PREDICTED_FOLLOW_MAX_MS after the
    // first one, even with "nothing" frames in between...
    orin("760,-40,0,2,2,48");
    tick();
    assert(Vision::isPredicted());
    for (int i = 0; i < 40; i++) {                       // 40 x 100 ms = 4 s
        orin(i % 2 ? "0,0,0,0,0,48" : "700,-40,0,2,2,48");
        tick(100);
    }
    orin("700,-40,0,2,2,48");
    tick();
    assert(!Vision::isPredicted());
    // ...until the hole is really seen, or a new selection.
    orin("-30,-200,2900,2,1,2");
    tick();
    assert(Vision::isValid());
    orin("700,-40,0,2,2,48");
    tick();
    assert(Vision::isPredicted());
    // 2026-10-28: optional ring_pct,ring_sectors (8 fields); kept only with
    // a real sighting of the selected hole; 6 fields still accepted.
    orin("-30,-200,2900,2,1,2,62,60");
    tick();
    assert(Vision::isValid() && Vision::getRingPercent() == 62 && Vision::getRingSectors() == 60);
    orin("-30,-200,2900,2,1,2");
    tick();
    assert(Vision::isValid() && Vision::getRingPercent() == -1 && Vision::getRingSectors() == 0);
    orin("-30,-200,2900,5,1,2,62,60");                  // not the selected hole
    tick();
    assert(!Vision::isValid() && Vision::getRingPercent() == -1);
    orin("-30,-200,2900,2,1,2,101,0");                   // out of range: refused
    orin("-30,-200,2900,2,1,2,50,256");
    orin("-30,-200,2900,2,1,2,50");                      // 7 fields
    tick();
    assert(!Vision::isValid());
    orin("700,-40,0,2,2,48,-1,0");
    tick();
    assert(Vision::isPredicted() && Vision::getRingPercent() == -1);
    // Stale link: no prediction, no mask.
    for (int i = 0; i < 35; i++) tick(10);
    assert(!Vision::isPredicted() && Vision::getSeenMask() == 0);
    // Out-of-range fields are refused: valid 3, mask > 4095, tx > 400 when
    // not predicted.
    orin("1,1,5000,2,3,0");
    orin("1,1,5000,2,1,4096");
    orin("500,1,5000,2,1,0");
    tick();
    assert(!Vision::isValid());

    printf("vision_link_test: all asserts passed\n");
    return 0;
}
