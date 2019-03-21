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

    prizm.setServoSpeed(1, 100);
    prizm.setServoPosition(1, SERVO_COLLECT);

    prizm.drive_steps(300, {
                               {-9886, -9878},
                           });

    prizm.drive_steps(75, {{-406, -404}});
    prizm.drive_steps(30, {
                              {-35, 37},
                              {-823, -827},
                              {55, -50},
                              {-152, -148},
                              {9, -1},
                              {-48, -46},
                          });

    prizm.setServoSpeed(1, 25);
    prizm.setServoPosition(1, SERVO_DRIVE);

    prizm.drive_steps(300,
                      {{1267, 1254}, {-191, 193}, {3148, 3159}, {224, -219},
                       {4844, 4840}, {806, -773}, {2109, 2102}, {813, -798},
                       {1258, 1260}, {806, -772}, {2109, 2096}, {1041, -1028},
                       {1258, 1248}, {613, -607}, {1435, 1399}, {609, -588},
                       {6743, 6744}, {-580, 599}, {2308, 2311}, {-1267, 1266},
                       {2099, 2104}, {-568, 591}, {2511, 2522}, {-1207, 1222},
                       {406, 419},   {-825, 826}, {1677, 1684}, {-185, 187},
                       {3984, 3981}, {-177, 172}, {4641, 4640}});

    prizm.setServoSpeed(1, 100);
    prizm.setServoPosition(1, SERVO_LAUNCH);
    prizm.begin_rc_control(300);
}

void loop() {}