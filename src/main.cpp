#include "prizmatic.hpp"

#include "GroveColorSensor.h"

namespace {
PRIZMatic prizm{};
GroveColorSensor sens{};

template <typename T>
struct RGB {
    T r;
    T g;
    T b;

    static RGB<int16_t> from_sensor() {
        RGB<int16_t> res{};
        sens.readRGB(&res.r, &res.g, &res.b);
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
int const speed = 100;
int const SERVO_PUSH = 90;
int const SERVO_REST = 0;

struct ColourAssociation {
    RGBi col;
    int position;
};

ColourAssociation associations[] = {
    {{255, 0, 0}, 1500},  // red
    {{0, 255, 0}, 2500},  // green
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
    Serial.println("how do commputers work");
    prizm.PrizmBegin();
    sens.ledStatus = 1;

    // prizm.setServoPosition(1, SERVO_REST);
    prizm.begin_rc_control(50);
    DBG("Calibrating...");
    RGBi baseline = RGBi::from_sensor();
    State state = State::WaitingForBall;

    int pos = -1;
    while (true) {
        switch (state) {
            case State::WaitingForBall:
                if (baseline.dist_from(RGBi::from_sensor()) > equality_dist) {
                    delay(200);
                    continue;
                }
                delay(1000);
                pos = position_for_colour(RGBi::from_sensor());
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
                state = State::Dropping;
                break;

            case State::Dropping:
                prizm.setServoPosition(1, SERVO_PUSH);
                delay(1000);
                prizm.setMotorTarget(1, speed, 0);
                state = State::ReturningToStart;
                break;

            case State::ReturningToStart:
                if (prizm.readMotorBusy(1)) {
                    continue;
                }
                state = State::WaitingForBall;
                break;
        }
    }
}

void loop() {}