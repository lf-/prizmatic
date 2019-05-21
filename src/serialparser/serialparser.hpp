#include "sysheaders.h"
#pragma once
namespace serialparser {

uint8_t const max_nargs = 4;

enum class Command {
    None,
    SetServo,
    GetEncoders,
    GetUltrasonic,
    GetInfrared,
    DriveSteps,
    GetRC,
    TestMotors,
};

struct CommandDef {
    const char *name;
    Command cmd;
    uint8_t nargs;
};

CommandDef const allCommands[] = {
    {"set_servo", Command::SetServo, 2},
    {"get_encoders", Command::GetEncoders, 0},
    {"get_ultrasonic", Command::GetUltrasonic, 0},
    {"get_infrared", Command::GetInfrared, 0},
    {"drive_steps", Command::DriveSteps, 3},
    {"get_rc", Command::GetRC, 0},
    {"test_motors", Command::TestMotors, 0},
};

struct ParseResult {
    Command cmd;
    long args[max_nargs];
};

ParseResult parse(Stream *ser);

}  // namespace serialparser