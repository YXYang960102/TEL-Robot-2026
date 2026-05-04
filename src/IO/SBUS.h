#ifndef SBUS_H
#define SBUS_H
#include <sbus.h>



class SBUS {
public:
    static void init();
    static void update();

    static int ch0, ch1, ch2, ch3, ch8;

private:
    static bfs::SbusRx sbus;
    static bfs::SbusData data;
};

#endif