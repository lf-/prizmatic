#define DEBUG_PRIZMATIC 1

#include "prizmatic.hpp"

PRIZMatic prizm;

const int SERVO_COLLECT = 39;
const int SERVO_LAUNCH = 130;
const int SERVO_DRIVE = 70;

void setup() {
    Serial.begin(9600);
    delay(200);
    prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
    prizm.setMotorInvert(2, 1);

    prizm.begin_rc_control(300);
}

void loop() {}