#include <assert.h>
#include <stdint.h>
#include "../src/Control/VisionRotateBenchState.h"

int main() {
    // Boot: neutral, disabled.
    {
        VisionRotateBenchState s;
        assert(s.pulse() == 1500 && !s.enabled);
    }

    // Vision ignored while disabled.
    {
        VisionRotateBenchState s;
        s.onVision(100, true, 10);
        assert(s.direction == 0 && s.pulse() == 1500);
    }

    // Deadband: tx just inside/outside +-20px (tightened from 25, 2026-10-05).
    // Just past the deadband is still inside the slow zone (<=70px), so it
    // creeps, not full speed.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(20, true, 1); assert(s.direction == 0); // at the edge, not beyond
        s.onVision(21, true, 2); assert(s.direction == 1 && s.pulse() == 1350); // creep, INVERTED
        s.onVision(-20, true, 3); assert(s.direction == 0);
        s.onVision(-21, true, 4); assert(s.direction == -1 && s.pulse() == 1650); // creep, INVERTED
        s.onVision(0, true, 5); assert(s.direction == 0 && s.pulse() == 1500);
    }

    // Speed profile: stop (<=20px) -> PositionDecelerationProfile ramp,
    // floored at the creep speed near the deadband, linearly increasing to
    // full speed exactly at the slow-zone edge (70px, narrowed from 80,
    // 2026-10-05) and beyond. This is a genuinely gradual slowdown (Jeremy's
    // requested behavior, reusing the same profile already used for Shooter
    // elevation), not a hand-rolled two-step jump between fixed speeds.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(35, true, 1); assert(s.direction == 1 && s.pulse() == 1350); // at the floor (300*35/70=150)
        s.onVision(49, true, 2); assert(s.direction == 1 && s.pulse() == 1290); // mid-ramp (300*49/70=210)
        s.onVision(70, true, 3); assert(s.direction == 1 && s.pulse() == 1200); // full speed at the slow-zone edge
        s.onVision(-49, true, 4); assert(s.direction == -1 && s.pulse() == 1710); // mid-ramp, mirrored
    }

    // valid=0 forces direction 0 even with a large tx.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(200, true, 1); assert(s.direction == 1);
        s.onVision(200, false, 2); assert(s.direction == 0);
    }

    // VISION_TIMEOUT: USB pings keep arriving (link alive) but no vision
    // line ever does (Orin disconnected/crashed) -> stop after 500ms.
    {
        VisionRotateBenchState s;
        s.command("E", 0); // lastUsbPing=lastVisionRx=0
        s.command("P", 100); assert(s.enabled);
        s.command("P", 200); assert(s.enabled);
        s.command("P", 300); assert(s.enabled);
        s.command("P", 400); assert(s.enabled);
        s.tick(501);
        assert(!s.enabled);
        assert(!strcmp(s.stopReason, "VISION_TIMEOUT"));
    }

    // TIMEOUT: vision keeps arriving but the browser tab stops pinging
    // (tab crashed/USB dropped) -> stop after 250ms, before VISION_TIMEOUT
    // would ever fire.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(0, true, 200);
        s.tick(251);
        assert(!s.enabled);
        assert(!strcmp(s.stopReason, "TIMEOUT"));
    }

    // LIMIT, with and without noLimits bypass.
    {
        VisionRotateBenchState s;
        s.command("E", 200);
        s.onVision(100, true, 201); // direction=1
        s.applyLimits(false, true);
        assert(!s.enabled && !s.noLimits);

        s.command("E_NO_LIMITS", 300);
        s.onVision(100, true, 301);
        s.applyLimits(false, true);
        assert(s.enabled && s.noLimits && s.pulse() == 1200); // INVERTED
    }

    // E clears any stale noLimits/direction from a prior session.
    {
        VisionRotateBenchState s;
        s.command("E_NO_LIMITS", 0);
        s.onVision(100, true, 1);
        s.command("E", 800);
        assert(s.enabled && !s.noLimits && s.direction == 0);
    }

    // BAD_COMMAND.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        assert(!s.command("garbage", 1) && !s.enabled);
        assert(!strcmp(s.stopReason, "BAD_COMMAND"));
    }

    // Explicit "X" stop.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(100, true, 1);
        s.command("X", 2);
        assert(!s.enabled && s.direction == 0 && s.pulse() == 1500);
    }

    // "P" while disabled is a harmless no-op, not BAD_COMMAND.
    {
        VisionRotateBenchState s;
        assert(s.command("P", 5));
        assert(!s.enabled);
    }

    // 32-bit millis() rollover (COMMAND_TIMEOUT watchdog path).
    {
        VisionRotateBenchState s;
        s.command("E", UINT32_MAX - 100);
        s.tick(100); assert(s.enabled);
        s.tick(152); assert(!s.enabled);
    }

    return 0;
}
