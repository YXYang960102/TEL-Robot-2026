#include <assert.h>
#include <math.h>
#include "../src/Control/ServoPositionJog.h"
int main() {
    ServoPositionJog jog(1500);
    for (int i=0; i<100; ++i) jog.step(0.3,10,1000,1000,2000);
    assert(fabs(jog.value()-1800)<1e-6);
    assert(jog.step(0,20,1000,1000,2000)==1800);
    assert(jog.step(0.3,1000,1000,1000,2000)==1800);
    assert(jog.step(NAN,10,1000,1000,2000)==1800);
    for (int i=0; i<100; ++i) jog.step(0.3,10,1000,1000,2000);
    assert(jog.value()==2000);
    for (int i=0; i<400; ++i) jog.step(-0.3,10,1000,1000,2000);
    assert(jog.value()==1000);
    assert(jog.step(0.3,10,1000,1000,2000)==1003);
}
