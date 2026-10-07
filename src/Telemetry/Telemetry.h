#pragma once

class Telemetry {
public:
    static void init();
    static void update();

private:
    static unsigned long lastSendMs;
};
