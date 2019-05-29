#include "prizmatic.hpp"

PRIZMatic prizm{};

const int SERVO_COLLECT = 39;
const int SERVO_LAUNCH = 130;
const int SERVO_DRIVE = 70;

// void motor_variable(PRIZMatic prizm, );

void setup() {
    Serial.begin(9600);
    delay(200);
    prizm.dump_eeprom_steps();
    prizm.PrizmBegin();

    prizm.setServoPosition(1, 18);
    prizm.setServoPosition(2, 22);
    prizm.setServoSpeed(1, 30);
    prizm.setServoSpeed(2, 30);

    prizm.setMotorInvert(2, 1);
    prizm.begin_rc_control(200);
    prizm.drive_steps(100, {});
    prizm.wait_for_start_button();

    prizm.setServoPosition(1, 5);
    prizm.setServoPosition(2, 0);
    delay(500);

    prizm.wait_for_start_button();
    delay(2000);
    // prizm.begin_rc_control(200, true);

    prizm.drive_steps(200, {});

    prizm.setServoPosition(1, 90);
    delay(2000);

    prizm.drive_steps(200, {});

    prizm.setServoPosition(2, 90);
    delay(2000);

    // prizm.begin_rc_control(100, true);
    prizm.drive_steps(200, {});
    prizm.begin_kb_control(100, true);

    // prizm.drive_sensor(300, 200, []() { return prizm.readSonicSensorMM(1);
    // });
}

void loop() {}