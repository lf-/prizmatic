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

int const BRAKE = 125;

struct Step {
    long left;
    long right;
};

namespace {
Step saved_steps[100];
uint8_t n_saved_steps;

const uint8_t signal_switch_pin = 17;  // Port A1
const uint8_t turn_pin = 15;           // Port A2
const uint8_t fwd_rev_pin = 16;        // Port A3
}  // namespace

class PRIZMatic : public PRIZM {
   public:
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void wait_for_start_button();
    void begin_rc_control(long speed);
    int8_t read_rc(uint8_t pin);
    void debug_controller();
    void send_steps();
};

template <typename T>
int sgn(T val);

void PRIZMatic::drive_steps(long speed, std::initializer_list<Step> steps) {
    for (auto step : steps) {
        // this->setMotorTargets(speed, step.left, speed, step.right);
        DBGN("Driving step: ");
        DBGN(step.left);
        DBGN(", ");
        DBG(step.right);

        this->resetEncoders();
        this->setMotorPowers(sgn(step.left) * 30, sgn(step.right) * 30);
        bool done1 = false;
        bool done2 = false;
        while (!(done1 && done2)) {
            auto enccount1 = abs(this->readEncoderCount(1));
            DBGN("enccount1: ");
            DBG(enccount1);
            if (enccount1 > abs(step.left)) {
                DBG("DONE1");
                this->setMotorPower(1, BRAKE);
                done1 = true;
            }
            auto enccount2 = abs(this->readEncoderCount(2));
            DBGN("enccount2: ");
            DBG(enccount2);
            if (enccount2 > abs(step.right)) {
                DBG("DONE2");
                this->setMotorPower(2, BRAKE);
                done2 = true;
            }
        }
        delay(500);
    }
    this->resetEncoders();
    return;
}

void PRIZMatic::wait_for_start_button() {
    DBG("waiting for start button....");
    while (!this->readStartButton()) {
    }
    DBG("started!");
}

void PRIZMatic::debug_controller() {
    while (!this->readStartButton()) {
        Serial.print("Signal Switch: ");
        Serial.println(this->read_rc(signal_switch_pin));

        Serial.print("FWD/REV: ");
        Serial.println(this->read_rc(fwd_rev_pin));

        Serial.print("Turn: ");
        Serial.println(this->read_rc(turn_pin));
        delay(400);
    }
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

void PRIZMatic::send_steps() {
    for (uint8_t i = 0; i < n_saved_steps; ++i) {
        Serial.print('{');
        Serial.print(saved_steps[i].left);
        Serial.print(", ");
        Serial.print(saved_steps[i].right);
        Serial.print("},\n");
    }
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

    bool done = false;
    unsigned long timer_start = 0;

    while (!done) {
        delay(100);
        auto sw = -read_rc(signal_switch_pin);
        // DBG("switch:");
        // DBG(sw);
        if (sw > switch_on) {
            if (!timer_start) {
                // start timer
                timer_start = millis();
            }
        } else if (sw < switch_on && timer_start) {
            auto curr_time = millis();
            // switch is off, detect falling edge.
            if (curr_time > timer_start + 1000) {
                DBG("Exiting RC control");
                this->send_steps();
                // write over saved steps next time
                n_saved_steps = 0;
                done = true;
            } else if (curr_time > timer_start + 50) {
                DBG("Saving step...");
                // we are negating this because motor 1 is intended to be
                // inverted
                saved_steps[n_saved_steps] = {this->readEncoderCount(1),
                                              this->readEncoderCount(2)};
                ++n_saved_steps;
                this->resetEncoders();
                timer_start = 0;

                this->setRedLED(0);
                delay(200);
                this->setRedLED(1);
                delay(200);
                this->setRedLED(0);
            } else {
                timer_start = 0;
            }
        }

        auto throttle = -read_rc(fwd_rev_pin);
        auto turn = read_rc(turn_pin);
        // DBGN("throttle: ");
        // DBGN(throttle);
        // DBGN("turn: ");
        // DBG(turn);
        if (turn > switch_on || turn < switch_off) {
            auto turn_dir = sgn<int8_t>(turn);
            this->setMotorPowers(speed * turn_dir, -speed * turn_dir);
            // we want to ignore throttle inputs when turning
            DBGN("turn dir:");
            DBG(turn_dir);
            continue;
        }

        if (throttle > switch_on || throttle < switch_off) {
            auto motor_speed = speed * sgn<int8_t>(throttle);
            this->setMotorPowers(motor_speed, motor_speed);
            DBGN("Set motor speed:");
            DBG(motor_speed);
            continue;
        }

        this->setMotorSpeeds(0, 0);
    }
}