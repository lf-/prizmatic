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

uint8_t const signal_switch_pin = 17;  // Port A1
uint8_t const turn_pin = 15;           // Port A2
uint8_t const fwd_rev_pin = 16;        // Port A3
}  // namespace

class PRIZMatic : public PRIZM {
   public:
    PRIZMatic();
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void wait_for_start_button();
    void begin_rc_control(long speed);
    int8_t read_rc(uint8_t pin);
    void debug_controller();
    void send_steps();

    // wrap the parent class to save invert state
    void setMotorInvert(int channel, int invert);

    bool channel_inverts[3];
};

template <typename T>
int sgn(T val);

PRIZMatic::PRIZMatic() : channel_inverts{false, false} {}

void PRIZMatic::setMotorInvert(int channel, int invert) {
    if (channel == 1 || channel == 2) {
        this->channel_inverts[channel] = invert;
        PRIZM::setMotorInvert(channel, invert);
    }
}

int get_invert_factor(bool inv) {
    if (inv) {
        return -1;
    } else {
        return 1;
    }
}

void PRIZMatic::drive_steps(long speed, std::initializer_list<Step> steps) {
    for (auto step : steps) {
        // this->setMotorTargets(speed, step.left, speed, step.right);
        DBGN("Driving step: ");
        DBGN(step.left);
        DBGN(", ");
        DBG(step.right);

        this->resetEncoders();
        this->setMotorTargets(speed, step.left, speed, step.right);
        while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
        }
        delay(500);
        DBGN("ENCODER COUNTS AFTER MOVE: ");
        DBGN(this->readEncoderCount(1));
        DBGN(", ");
        DBG(this->readEncoderCount(2));
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
                saved_steps[n_saved_steps] = {
                    get_invert_factor(channel_inverts[1]) *
                        this->readEncoderCount(1),
                    get_invert_factor(channel_inverts[2]) *
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
    return;
}