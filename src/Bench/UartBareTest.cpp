#include <Arduino.h>
#include "../Constants/UartBareBenchConstants.h"

// Bare Mega + USB ONLY. No Servo library or actuator initialization.
// RX1 pull-up is to Mega's 5V: DO NOT connect Orin or any other equipment.
namespace {
using namespace UartBareBenchConstants;
enum Phase { QUIET, TRANSMIT, DONE };
Phase phase = DONE;
uint32_t phaseStart=0, lastTx=0, lastReport=0;
uint32_t rxBytes=0, txFrames=0;
uint8_t sample[SAMPLE_SIZE], sampleLength=0;
const char* name() { return phase==QUIET ? "QUIET" : phase==TRANSMIT ? "TX" : "DONE"; }
void report(bool finalReport) {
    Serial.print(finalReport ? F("RESULT,") : F("STAT,"));
    Serial.print(name());Serial.print(F(",rx_bytes="));Serial.print(rxBytes);
    Serial.print(F(",tx_frames="));Serial.print(txFrames);
    Serial.print(F(",rx_pin="));Serial.print(digitalRead(RX_PIN));
    Serial.print(F(",first_bytes_hex="));
    for(uint8_t i=0;i<sampleLength;++i){
        if(i)Serial.print(' ');
        if(sample[i]<16)Serial.print('0');
        Serial.print(sample[i],HEX);
    }
    Serial.println();
}
void beginPhase(Phase next) {
    phase=next;phaseStart=lastTx=lastReport=millis();
    rxBytes=txFrames=sampleLength=0;
    Serial.print(F("PHASE,"));Serial.println(name());
}
}
void setup() {
    Serial.begin(BAUD);
    Serial1.begin(BAUD);
    pinMode(RX_PIN,INPUT_PULLUP);
    Serial.println(F("UART_BARE_DIAG,1; USB ONLY; NO ORIN/MOTOR/SHIELD/JUMPER"));
    Serial.println(F("RX1 pullup ON; 10s QUIET -> 10s TX -> DONE; send R to repeat"));
    beginPhase(QUIET);
}
void loop() {
    // Bounded acquisition and aggregate reporting avoid per-byte USB flooding.
    for(uint8_t n=0;n<64 && Serial1.available();++n){
        uint8_t b=Serial1.read();
        if(phase!=DONE){++rxBytes;if(sampleLength<SAMPLE_SIZE)sample[sampleLength++]=b;}
    }
    for(uint8_t n=0;n<16 && Serial.available();++n){
        char c=Serial.read();
        if((c=='R'||c=='r')&&phase==DONE)beginPhase(QUIET);
    }
    const uint32_t now=millis();
    if(phase==DONE)return;
    if(uint32_t(now-phaseStart)>=PHASE_MS){
        report(true);
        if(phase==QUIET)beginPhase(TRANSMIT);
        else {phase=DONE;Serial.println(F("DONE; send R to repeat; TX stopped"));}
        return;
    }
    if(phase==TRANSMIT && uint32_t(now-lastTx)>=TX_PERIOD_MS){
        lastTx=now;Serial1.print(PROBE);++txFrames;
    }
    if(uint32_t(now-lastReport)>=REPORT_MS){lastReport=now;report(false);}
}
