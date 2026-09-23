#include <assert.h>
#include <stdint.h>
#include "../src/Control/RotateBenchState.h"
int main() {
    RotateBenchState s;
    assert(s.pulse() == 1500 && !s.enabled);
    s.command("M,0,1", 0); assert(s.pulse() == 1500);
    s.command("E", 10); s.command("M,0,1", 20); assert(s.pulse() == 2000);
    s.command("M,0,0", 30); assert(s.pulse() == 1500);
    s.command("M,0,-1", 40); assert(s.pulse() == 1000);
    s.tick(291); assert(!s.enabled && s.pulse() == 1500);
    s.command("M,0,1", 292); assert(!s.enabled);
    s.command("E", 300); s.command("M,0,1", 301);
    for (uint32_t t=350; t<700; t+=50) s.command("M,0,1", t);
    // JOG_CAP removed 2026-09-19 (Jeremy, supervised manual testing): sustained
    // holding with regular heartbeats no longer auto-disables on its own.
    s.tick(701); assert(s.enabled);
    s.command("E", 800); assert(!s.command("M,1,1", 801) && !s.enabled);
    s.command("E", 900); s.command("X", 901); assert(!s.enabled);
    s.command("E", UINT32_MAX-100); s.command("M,0,-1", UINT32_MAX-99);
    s.tick(100); assert(s.enabled);
    s.tick(152); assert(!s.enabled); // 32-bit millis rollover.
    s.command("E",200);s.command("M,0,1",201);s.applyLimits(true,true);
    assert(!s.enabled && !s.noLimits);
    s.command("E_NO_LIMITS",300);s.command("M,0,1",301);s.applyLimits(true,true);
    assert(s.enabled && s.noLimits && s.pulse()==2000);
    for(uint32_t t=350;t<700;t+=50)s.command("M,0,1",t);
    s.tick(701);assert(s.enabled && s.noLimits); // same JOG_CAP removal, no-limits mode
    s.command("E",800);assert(!s.noLimits);
    s.command("E_NO_LIMITS",900);s.tick(1151);assert(!s.noLimits && !s.enabled);
    s.command("E_NO_LIMITS",1200);s.command("X",1201);assert(!s.noLimits);
}
