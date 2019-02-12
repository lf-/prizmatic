#pragma once
#include <PRIZM.h>
#include <stdint.h>
#include <initializer_list>

#ifdef DEBUG_PRIZMATIC
#define DBG(s) Serial.println(s)
#define DBGN(s) Serial.print(s)
#else
#define DBG(s)
#define DBGN(s)
#endif

struct Step {
    long left;
    long right;
};

class PRIZMatic : public PRIZM {
   public:
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void wait_for_start_button();
    void begin_rc_control(long speed);
    int8_t read_rc(uint8_t pin);
};

void PRIZMatic::drive_steps(long speed, std::initializer_list<Step> steps) {
    for (auto step : steps) {
        this->setMotorTargets(speed, step.left, speed, step.right);
        while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
        }
        this->resetEncoders();
    }
}

void PRIZMatic::wait_for_start_button() {
    DBG("waiting for start button....");
    while (!this->readStartButton()) {
    }
    DBG("started!");
}

template <typename T>
T inline clamp(T val, T min, T max) {
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    }
    return val;
}

int8_t PRIZMatic::read_rc(uint8_t pin) {
    const auto pulseWidth = pulseIn(pin, HIGH, 100000);
    if (pulseWidth == 0) {
        return 0;
    }
    return (clamp<signed long>((pulseWidth - 1200), 0, 510) - 255) / 2;
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void PRIZMatic::begin_rc_control(long speed) {
    /*
     * Drop into RC control and record inputs
     * This function deliberately restricts movements to on/off control at the
     * same speed as would be automatically driven to attempt to ensure maximum
     * reproducibility.
     */

    // change these if signalling switch doesn't work
    const int8_t switch_on = 50;
    const int8_t switch_off = -50;
    const uint8_t signal_switch_pin = 17;  // Port A1
    const uint8_t turn_pin = 15;           // Port A2
    const uint8_t fwd_rev_pin = 16;        // Port A3

    bool done = 0;
    unsigned long done_timer = 0;

    while (!done) {
        delay(400);
        auto sw = -read_rc(signal_switch_pin);
        // DBG("switch:");
        // DBG(sw);
        if (sw > switch_on) {
            if (!done_timer) {
                // start timer
                done_timer = millis() + 1000;
            }
        } else {
            // switch is off, detect falling edge.
            if (done_timer && millis() > done_timer) {
                Serial.print('{');
                Serial.print(this->readEncoderCount(1));
                Serial.print(", ");
                Serial.print(this->readEncoderCount(2));
                Serial.print("}\n");
                this->resetEncoders();
                done_timer = 0;

                this->setRedLED(0);
                delay(200);
                this->setRedLED(1);
                delay(200);
                this->setRedLED(0);
            } else {
                done_timer = 0;
            }
        }

        auto throttle = -read_rc(fwd_rev_pin);
        auto turn = read_rc(turn_pin);
        DBGN("throttle: ");
        DBGN(throttle);
        DBGN("turn: ");
        DBG(turn);
        if (turn > switch_on || turn < switch_off) {
            auto turn_dir = sgn<int8_t>(turn);
            this->setMotorSpeeds(speed * turn_dir, -speed * turn_dir);
            // we want to ignore throttle inputs when turning
            DBGN("turn dir:");
            DBG(turn_dir);
            continue;
        }

        if (throttle > switch_on || throttle < switch_off) {
            auto motor_speed = speed * sgn<int8_t>(throttle);
            this->setMotorSpeeds(motor_speed, motor_speed);
            DBGN("Set motor speed:");
            DBG(motor_speed);
            continue;
        }

        this->setMotorSpeeds(0, 0);
    }
}