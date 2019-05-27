#include "prizmatic.hpp"

PRIZMatic prizm{};

const int SERVO_COLLECT = 39;
const int SERVO_LAUNCH = 130;
const int SERVO_DRIVE = 70;

void setup() {
    Serial.begin(9600);
    delay(200);
    // prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
    prizm.setMotorInvert(2, 1);

    prizm.begin_rc_control(300);

    prizm.drive_sensor(300, 200, []() { return prizm.readSonicSensorMM(1); });
}

void loop() {}