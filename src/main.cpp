#define DEBUG_PRIZMATIC 1

#include "prizmatic.hpp"

PRIZMatic prizm;

void setup() {
    Serial.begin(9600);
    delay(200);
    prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
    prizm.setMotorInvert(2, 1);

    // while (true) {
    //     DBGN("FWD REV ");
    //     DBG(prizm.read_rc(fwd_rev_pin));
    // }

    // prizm.setCRServoState(1, 100);
    // delay(1000);
    // prizm.setCRServoState(1, 0);
    prizm.setServoSpeed(1, 100);
    prizm.setServoPosition(1, 39);

    // prizm.begin_rc_servo_test(1);
    // while (true) {
    //     delay(3000);
    //     prizm.setServoPosition(1, 130);
    //     delay(1000);
    // }
    prizm.drive_steps(300, {
                               {-9886, -9878},
                           });
    prizm.drive_steps(75, {
                              {90, -86},
                              {-1150, -1148},
                              {-252, -244},
                          });
    prizm.setServoSpeed(1, 25);
    prizm.setServoPosition(1, 80);
    prizm.drive_steps(300, {
                               {10302, 10300},
                               {1675, -1664},
                               {3569, 3566},
                               {1034, -1026},
                               {1678, 1671},
                               {1877, -1871},
                               {0, 0},
                               {11569, 11561},
                               {-2306, 2294},
                               {5259, 5253},
                               {-2297, 2291},
                               {10000, 10000},
                           });
    prizm.setServoSpeed(1, 100);
    prizm.setServoPosition(1, 130);
    prizm.begin_rc_control(300);
    // prizm.drive_steps(
    //     300, {
    //              {10102, 10094}, {1662, -1655}, {2740, 2736},  {396, -386},
    //              {2307, 2302},   {2517, -2508}, {5463, 5456},  {5669, 5669},
    //              {0, 0},         {-2292, 2295}, {405, -399},   {4412, 4414},
    //              {0, 0},         {0, 0},        {-1869, 1873}, {2076, 2086},
    //              {-610, 620},    {3142, 3139},  {-389, 403},   {6305, 6307},
    //          });
}

void loop() {}