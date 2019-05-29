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
    // prizm.begin_kb_control(200, true);
    prizm.drive_steps(100, {
                               {-570, -570},
                           });
    prizm.wait_for_start_button();

    prizm.setServoPosition(1, 5);
    prizm.setServoPosition(2, 0);
    delay(500);

    prizm.wait_for_start_button();
    delay(2000);
    // prizm.begin_rc_control(200, true);

    // prizm.begin_kb_control(100, true);
    prizm.drive_steps(200, {
                               {2810, 2809},
                               {720, -718},
                               {-2486, -2487},
                               {689, -687},
                               {-1261, -1262},
                               {-319, -319},
                           });

    prizm.setServoPosition(1, 90);
    delay(2000);

    prizm.drive_steps(200, {
                               {1673, 1673},
                               {-715, 716},
                               {-2889, -2890},
                               {710, -711},
                               {-5340, -5342},
                               {715, -715},
                               {-5381, -5382},
                               {727, -727},
                               {-2775, -2777},
                               {-747, 747},
                               {-2606, -2609},
                               {590, -812},
                               {-2244, -2246},
                           });
    // prizm.begin_kb_control(100, true);

    prizm.setServoPosition(2, 90);
    delay(2000);

    // prizm.begin_rc_control(100, true);
    prizm.drive_steps(200, {
                               {7453, 7455},
                               {732, -731},
                               {-7973, -7976},
                               {688, -688},
                               {-2276, -2278},
                           });
    prizm.begin_rc_control(100, true);

    // prizm.drive_sensor(300, 200, []() { return prizm.readSonicSensorMM(1);
    // });
}

void loop() {}