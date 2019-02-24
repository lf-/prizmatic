#define DEBUG_PRIZMATIC 1

#include "prizmatic.hpp"

PRIZMatic prizm;

void setup() {
    Serial.begin(9600);
    prizm.PrizmBegin();
    prizm.setMotorInvert(1, 1);

    prizm.begin_rc_control(300);
    prizm.drive_steps(300, {
                               {-8625, 8634},
                               {-2310, -2294},
                               {-1667, 1673},
                           });
}

void loop() {}