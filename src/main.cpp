#include "prizmatic.hpp"

#include "GroveColorSensor.h"

namespace {
PRIZMatic prizm{};

template <typename T>
struct RGB {
    T r;
    T g;
    T b;

    static RGB<int16_t> from_sensor(GroveColorSensor& sens) {
        RGBC sensor_reading = sens.readRGB();
        RGB<int16_t> res{sensor_reading.r, sensor_reading.g, sensor_reading.b};
        return res;
    }

    float dist_from(RGB other) {
        RGB<float> diff = {this->r - other.r, this->g - other.g,
                           this->b - other.b};
        return sqrtf(diff.r * diff.r + diff.g * diff.g + diff.b * diff.b);
    }
};

enum class State {
    WaitingForBall,
    DrivingToPosition,
    Dropping,
    ReturningToStart,
};

using RGBi = RGB<int16_t>;

float const equality_dist = 25.f;
int const speed = 250;
int const SERVO_PUSH = 45;
int const SERVO_REST = 180;

int const SERVO_CHANNEL = 4;

struct ColourAssociation {
    RGBi col;
    int position;
};

const ColourAssociation associations[] = {
    {{249, 72, 5}, 100},   // yellow
    {{8, 94, 11}, 200},    // green
    {{10, 17, 131}, 300},  // blue
    {{190, 10, 6}, 600},   // red
};

int position_for_colour(RGBi colour) {
    for (auto ca : associations) {
        if (ca.col.dist_from(colour) < equality_dist) {
            return ca.position;
        }
    }
    return -1;
}

}  // namespace

void setup() {
    Serial.begin(9600);
    delay(200);
    // prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
    GroveColorSensor sens{};
    sens.ledStatus = 1;

    prizm.setServoPosition(SERVO_CHANNEL, SERVO_REST);

    // prizm.setServoPosition(1, SERVO_REST);
    DBG("Calibrating...");
    State state = State::WaitingForBall;
    // while (true) {
    //     auto temp = sens.readRGB();
    //     DBGN(temp.r);
    //     DBGN(", ");
    //     DBGN(temp.g);
    //     DBGN(", ");
    //     DBGN(temp.b);
    //     DBGN(", ");
    //     DBG(temp.c);
    // }

    RGBC sens_reading;
    int pos = -1;
    while (true) {
        // Literally just ignore next line
        // DBG((int)state);

        switch (state) {
            case State::WaitingForBall:
                sens_reading = sens.readRGB();
                if (sens_reading.c < 75) {
                    delay(200);
                    continue;
                }

                DBGN("R:");
                DBGN(sens_reading.r);
                DBGN(", G:");
                DBGN(sens_reading.g);
                DBGN(", B:");
                DBG(sens_reading.b);

                delay(1000);
                pos = position_for_colour(RGBi::from_sensor(sens));
                if (pos == -1) {
                    DBG("Unrecognized colour!");
                    prizm.flash_red(500);
                    continue;
                }

                prizm.setMotorTarget(1, speed, pos);
                state = State::DrivingToPosition;
                break;

            case State::DrivingToPosition:
                if (prizm.readMotorBusy(1)) {
                    continue;
                }
                prizm.setMotorPower(1, 0);  // Stops humming
                state = State::Dropping;
                break;

            case State::Dropping:
                prizm.setServoPosition(SERVO_CHANNEL, SERVO_PUSH);
                delay(500);
                prizm.setServoPosition(SERVO_CHANNEL, SERVO_REST);
                prizm.setMotorTarget(1, speed, 0);
                state = State::ReturningToStart;
                break;

            case State::ReturningToStart:
                if (prizm.readMotorBusy(1)) {
                    continue;
                }
                prizm.setMotorPower(1, 0);  // Stops humming
                state = State::WaitingForBall;
                break;
        }
    }
}

void loop() {}