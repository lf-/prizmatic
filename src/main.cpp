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
    prizm.setServoPosition(2, 5);
    delay(500);

    prizm.wait_for_start_button();
    // prizm.begin_rc_control(200, true);

    // prizm.begin_kb_control(100, true);
    prizm.drive_steps_sloped(720, {
                                      {2810, 2809},
                                      {720, -718},
                                      {-2486, -2487},
                                      {689, -687},
                                      {-2000, -2000},
                                  });

    prizm.setServoPosition(1, 90);
    delay(1000);

    prizm.drive_steps_sloped(720, {
                                      {2097, 2097},
                                      {-736, 741},
                                      {-2760, -2762},
                                      {706, -705},
                                      {-5047, -5049},
                                      {710, -710},
                                      {-5180, -5181},
                                      {690, -690},
                                      {-1907, -1909},
                                      {-472, 472},
                                      {-3098, -3100},
                                      {497, -496},
                                      {-954, -954},
                                  });

    prizm.setServoPosition(2, 90);
    delay(1000);

    // prizm.begin_kb_control(100, true);

    // prizm.begin_rc_control(100, true);
    prizm.drive_steps_sloped(720, {
                                      {6800, 6800},
                                      {738, -737},
                                      {-7500, -7500},
                                      {545, -545},
                                      {-1730, -1731},
                                  });
    prizm.begin_rc_control(100, true);

    // prizm.drive_sensor(300, 200, []() { return prizm.readSonicSensorMM(1);
    // });
}

void loop() {}