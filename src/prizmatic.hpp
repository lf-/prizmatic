#pragma once

// this is terrible.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <EEPROM.h>
#include <PRIZM.h>
#include <stdint.h>
#include <initializer_list>
#pragma GCC diagnostic pop

#include "serialparser/serialparser.hpp"

#ifdef DEBUG_PRIZMATIC
#define DBG(s) Serial.println(s)
#define DBGN(s) Serial.print(s)
#else
#define DBG(s)
#define DBGN(s)
#endif

#ifdef PRIZMATIC_TETRIX_MOTOR
long degrees_in_counts(float degs) { return 1440.f * (degs / 360.f); }
#else
long degrees_in_counts(float degs) { return 3960.f * (degs / 360.f); }
#endif

int const BRAKE = 125;

struct Step {
    long left;
    long right;
};

namespace {
int const MAX_STEPS = 50;
Step saved_steps[MAX_STEPS];
uint8_t n_saved_steps;

uint8_t const signal_switch_pin = 17;  // Port A1
uint8_t const turn_pin = 15;           // Port A2
uint8_t const fwd_rev_pin = 16;        // Port A3

uint8_t const SONIC_PIN = 2;  // Port D2
}  // namespace

class PRIZMatic : public PRIZM {
   public:
    PRIZMatic();
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void wait_for_start_button();
    void begin_rc_control(long speed, bool continue_next = false);
    void begin_rc_servo_test(uint8_t servonum);
    int8_t read_rc(uint8_t pin);
    void debug_controller();
    void send_steps();
    float readSonicSensorMM(uint8_t pin);

    void dump_eeprom_steps();

    // wrap the parent class to save invert state
    void setMotorInvert(int channel, int invert);

   private:
    bool channel_inverts[3];
    void handle_serial();
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
    return clamp<long>(map(pulseWidth, 1200, 1700, -127, 127), -127, 127);
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

void PRIZMatic::begin_rc_servo_test(uint8_t servonum) {
    while (true) {
        int8_t const pos = this->read_rc(fwd_rev_pin);
        uint8_t const servopos = map(pos, -128, 127, 0, 180);
        DBGN("\rServo position: ");
        DBGN(servopos);
        DBGN("       ");
        this->setServoPosition(servonum, servopos);
        delay(20);
    }
}

struct EEPROMSavedSteps {
    uint8_t nsteps;
    Step steps[MAX_STEPS];
};

static_assert(sizeof(EEPROMSavedSteps) < E2END + 1,
              "EEPROM saved steps larger than EEPROM size");

void save_steps_to_eeprom() {
    EEPROMSavedSteps steps;
    steps.nsteps = n_saved_steps;
    memcpy(steps.steps, saved_steps, sizeof(Step) * MAX_STEPS);
    EEPROM.put<EEPROMSavedSteps>(0, steps);
}

void PRIZMatic::dump_eeprom_steps() {
    EEPROMSavedSteps steps;
    Serial.println(F("Getting steps..."));
    EEPROM.get<EEPROMSavedSteps>(0, steps);
    Serial.println(F("Saved Steps:"));
    for (int i = 0; i < steps.nsteps; ++i) {
        Serial.print("{");
        auto const the_step = steps.steps[i];
        Serial.print(the_step.left);
        Serial.print(", ");
        Serial.print(the_step.right);
        Serial.println("},");
    }
}

void PRIZMatic::handle_serial() {
    serialparser::ParseResult parsed = serialparser::parse(&Serial);
    switch (parsed.cmd) {
        case serialparser::Command::GetEncoders:
            Serial.print("{");
            Serial.print(this->readEncoderCount(1));
            Serial.print(", ");
            Serial.print(this->readEncoderCount(2));
            Serial.println("},");
            break;
        case serialparser::Command::GetInfrared:
            Serial.println(F("Not implemented: getting infrared distance"));
            break;
        case serialparser::Command::GetUltrasonic:
            Serial.println(this->readSonicSensorMM(SONIC_PIN));
            break;
        case serialparser::Command::SetServo:
            this->setServoPosition(parsed.args[0], parsed.args[1]);
            break;
        case serialparser::Command::None:
        default:
            Serial.println(F("Command not recognized!"));
            // this is a terrible way to do this, thanks arduino!
            while (Serial.available() > 0) Serial.read();
            return;
    }
}

void PRIZMatic::begin_rc_control(long speed, bool continue_next) {
    /*
     * Drop into RC control and record inputs
     * This function deliberately restricts movements to on/off control at the
     * same speed as would be automatically driven to attempt to ensure maximum
     * reproducibility.
     */
    DBG("Beginning RC control");

    // change these if signalling switch doesn't work
    const int8_t switch_on = 50;
    const int8_t switch_off = -50;

    bool done = false;
    unsigned long timer_start = 0;

    while (!done) {
        if (Serial.available() > 0) {
            this->handle_serial();
        }
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
                save_steps_to_eeprom();
                // write over saved steps next time
                if (!continue_next) {
                    n_saved_steps = 0;
                }
                done = true;
            } else if (curr_time > timer_start + 50) {
                DBG("Saving step...");
                if (n_saved_steps > MAX_STEPS) {
                    n_saved_steps = 0;
                }
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
        // DBGN(" turn: ");
        // DBG(turn);
        if (turn > switch_on || turn < switch_off) {
            auto turn_dir = sgn<int8_t>(turn);
            this->setMotorSpeeds(speed * turn_dir, -speed * turn_dir);
            // we want to ignore throttle inputs when turning
            // DBGN("turn dir:");
            // DBG(turn_dir);
            continue;
        }

        if (throttle > switch_on || throttle < switch_off) {
            auto motor_speed = speed * sgn<int8_t>(throttle);
            this->setMotorSpeeds(motor_speed, motor_speed);
            // DBGN("Set motor speed:");
            // DBG(motor_speed);
            continue;
        }

        this->setMotorSpeeds(0, 0);
    }
    return;
}

float PRIZMatic::readSonicSensorMM(
    uint8_t pin) {  // Returns distance of object from sensor in millimeters

    delayMicroseconds(1000);  // added in version 2 to help with reading
                              // accuracy, can't read sonic sensors very fast
    int duration;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
    duration = pulseIn(pin, HIGH);
    return duration / 2.9f /
           2.f;  // convert time of echo to millimeters distance
}
