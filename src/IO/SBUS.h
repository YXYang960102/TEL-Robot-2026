#pragma once
#include <sbus.h>



class SBUS {
public:
    static void init();
    static void update();

    static int ch0, ch1, ch2, ch3, ch8;

    // True on a lost/failsafe frame. Nothing previously checked this —
    // ch0..ch8 used to just freeze at their last value on signal loss, which
    // is a real gap once anything is allowed to act autonomously on these
    // channels (see OperatorMode).
    static bool signalLost;

    // Dedicated operator-mode switch input (full-auto/semi-auto/full-manual
    // keypad, transmitted over ELRS to this SBUS receiver). Provisional:
    // repurposes ch8, which was computed but never used by anything else
    // (Chassis only reads ch0/ch1/ch3). Re-point this at whichever channel
    // the real keypad actually lands on once it's built.
    static int modeChannel;

private:
    static bfs::SbusRx sbus;
    static bfs::SbusData data;
};
