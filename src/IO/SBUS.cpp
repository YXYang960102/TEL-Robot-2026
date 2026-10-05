#include "SBUS.h"

namespace {
// No frame (dropout) for longer than this counts as signal lost, on top of
// the in-band lost_frame/failsafe flags the library already parses per frame.
constexpr unsigned long SBUS_FRAME_TIMEOUT_MS = 200;
unsigned long lastFrameMs = 0;
bool everReceivedFrame = false;
}

bfs::SbusRx SBUS::sbus(&Serial2);
bfs::SbusData SBUS::data;

int SBUS::ch0 = 1500;
int SBUS::ch1 = 1500;
int SBUS::ch2 = 1500;
int SBUS::ch3 = 1500;
int SBUS::ch8 = 1500;
bool SBUS::signalLost = true; // fail safe until the first frame ever arrives
int SBUS::modeChannel = 1500;

void SBUS::init() {
    sbus.Begin();
}

void SBUS::update() {
    if (sbus.Read()) {
        data = sbus.data();
        lastFrameMs = millis();
        everReceivedFrame = true;

        ch0 = map(data.ch[0],170,1820,1000,2000);
        ch1 = map(data.ch[3],170,1820,1200,1800);
        ch2 = map(data.ch[2],170,1820,1000,2000);
        ch3 = map(data.ch[1],170,1820,500,-500);
        ch8 = map(data.ch[8],170,1820,1500,2000);
        modeChannel = ch8;
    }

    signalLost = !everReceivedFrame ||
        data.lost_frame || data.failsafe ||
        (millis() - lastFrameMs) > SBUS_FRAME_TIMEOUT_MS;
}