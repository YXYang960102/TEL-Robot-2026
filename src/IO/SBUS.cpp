#include "SBUS.h"

bfs::SbusRx SBUS::sbus(&Serial2);
bfs::SbusData SBUS::data;

int SBUS::ch0 = 1500;
int SBUS::ch1 = 1500;
int SBUS::ch2 = 1500;
int SBUS::ch3 = 1500;
int SBUS::ch8 = 1500;

void SBUS::init() {
    sbus.Begin();
}

void SBUS::update() {
    if (sbus.Read()) {
        data = sbus.data();

        ch0 = map(data.ch[0],170,1820,1000,2000);
        ch1 = map(data.ch[3],170,1820,1200,1800);
        ch2 = map(data.ch[2],170,1820,1000,2000);
        ch3 = map(data.ch[1],170,1820,500,-500);
        ch8 = map(data.ch[8],170,1820,1500,2000);
    }
}