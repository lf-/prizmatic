#include <prizmatic.hpp>

#ifdef PRIZMATIC_TETRIX_MOTOR
long degrees_in_counts(float degs) { return 1440.f * (degs / 360.f); }
#else
long degrees_in_counts(float degs) { return 3960.f * (degs / 360.f); }
#endif

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

float get_counts_for_driven_distance(float dist) {
    return dist / tetrix_wheel_radius * (1440.f / (2.f * PI));
}

void PRIZMatic::drive_sensor(long speed, float distance, GetDistance fn) {
    this->setMotorSpeeds(speed, speed);
    while (distance - fn() > final_speed_dist) {
        delay(100);
    }
    this->resetEncoders();
    float countsToDrive = get_counts_for_driven_distance(distance - fn());
    this->setMotorTargets(100, countsToDrive, 100, countsToDrive);
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

template <typename T>
T inline clamp(T val, T min, T max) {
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    }
    return val;
}

template <typename T>
T inline min(T a, T b) {
    return (a > b ? b : a);
}

void PRIZMatic::drive_steps_sloped(long maxspeed,
                                   std::initializer_list<Step> steps) {
    auto const minspeed = 125L;
    auto const accel = 3L;
    auto magic_accel_offset = maxspeed * 2 / 3;
    auto magic_runoff = 0;
    for (auto step : steps) {
        // this->setMotorTargets(speed, step.left, speed, step.right);
        DBGN("Driving step: ");
        DBGN(step.left);
        DBGN(", ");
        DBG(step.right);

        this->resetEncoders();
        this->setMotorTargets(minspeed, step.left, minspeed, step.right);
        while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
            auto enc1 = abs(this->readEncoderCount(1));
            auto enc2 = abs(this->readEncoderCount(2));
            auto left = abs(step.left);
            auto right = abs(step.right);
            auto spd =
                clamp(min(left / 2 - abs(enc1 - (left - magic_runoff) / 2 +
                                         magic_accel_offset),
                          right / 2 - abs(enc2 - (right - magic_runoff) / 2 +
                                          magic_accel_offset)) /
                              accel +
                          minspeed,
                      minspeed, maxspeed);
            this->setMotorTargets(spd, step.left, spd, step.right);
            DBG(spd);
            delay(100);
        }
        delay(200);
        DBGN("ENCODER COUNTS AFTER MOVE: ");
        DBGN(this->readEncoderCount(1));
        DBGN(", ");
        DBG(this->readEncoderCount(2));
    }
    this->resetEncoders();
    return;
}

void PRIZMatic::drive_steps_sloped_betterer(long maxspeed,
                                            std::initializer_list<Step> steps) {
    auto const minspeed = 125L;

    for (auto step : steps) {
        // this->setMotorTargets(speed, step.left, speed, step.right);
        DBGN("Driving step: ");
        DBGN(step.left);
        DBGN(", ");
        DBG(step.right);

        this->resetEncoders();
        this->setMotorTargets(minspeed, step.left, minspeed, step.right);
        while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
            auto enc1 = abs(this->readEncoderCount(1));
            auto enc2 = abs(this->readEncoderCount(2));
            auto spd = 0L;
            auto const acceleration = 30L;
            auto const deacceleration = 1L;
            auto const pos_initial = 0L;
            auto pos_final = min(abs(step.left), abs(step.right));
            auto const jerk_initial = 100L;
            auto const jerk_final = 50L;
            if (min(enc1, enc2) <
                (acceleration * pos_initial - deacceleration * pos_final +
                 jerk_initial - jerk_final) /
                    (acceleration - deacceleration)) {
                spd = acceleration * (min(enc1, enc2) - pos_initial) +
                      jerk_initial;
            } else {
                spd =
                    deacceleration * (min(enc1, enc2) - pos_final) + jerk_final;
            }

            spd = clamp(spd, jerk_initial, 750L);

            this->setMotorTargets(spd, step.left, spd, step.right);
            DBG(spd);
            delay(100);
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
void PRIZMatic::drive_mm(long speed, std::initializer_list<MMStep> steps) {
    for (auto s : steps) {
        this->drive_steps_sloped(speed,
                                 {{get_counts_for_driven_distance(s.left),
                                   get_counts_for_driven_distance(s.right)}});
    }
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

void PRIZMatic::test_motors() {
    this->setMotorInvert(1, 0);
    this->setMotorInvert(2, 0);
    Serial.println(F("Testing LEFT, first # should be increasing"));
    this->setMotorPower(1, 100);
    delay(200);
    this->dump_encoder_counts();
    this->setMotorPower(1, BRAKE);
    delay(1000);
    this->resetEncoders();
    Serial.println(F("Testing RIGHT, second # should be increasing"));
    this->setMotorPower(2, 100);
    delay(200);
    this->dump_encoder_counts();
    this->setMotorPower(2, BRAKE);
    delay(1000);
    Serial.println(F("Setting motors SPEED 'forward'"));
    this->setMotorSpeeds(500, 500);
    delay(1000);
    this->setMotorSpeeds(0, 0);
    delay(1000);
    this->resetEncoders();
    Serial.println(F("Setting motor TARGETS 'forward'"));
    this->setMotorTargets(300, 4000, 300, 4000);
}

void PRIZMatic::dump_encoder_counts() {
    Serial.print("{");
    Serial.print(this->readEncoderCount(1));
    Serial.print(", ");
    Serial.print(this->readEncoderCount(2));
    Serial.println("},");
}

void PRIZMatic::handle_serial() {
    serialparser::ParseResult parsed = serialparser::parse(&Serial);
    switch (parsed.cmd) {
        case serialparser::Command::GetEncoders:
            this->dump_encoder_counts();
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
        case serialparser::Command::SetServoSpeed:
            this->setServoSpeed(parsed.args[0], parsed.args[1]);
            break;
        case serialparser::Command::DriveSteps:
            DBGN(F("Driving steps: spd, l, r: "));
            DBGN(parsed.args[0]);
            DBGN(" ");
            DBGN(parsed.args[1]);
            DBGN(" ");
            DBG(parsed.args[2]);
            this->setMotorTargets(parsed.args[0], parsed.args[1],
                                  parsed.args[0], parsed.args[1]);
            while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
            }
            this->resetEncoders();
            break;
        case serialparser::Command::DriveSSteps:
            DBGN(F("Driving steps: spd, l, r: "));
            DBGN(parsed.args[0]);
            DBGN(" ");
            DBGN(parsed.args[1]);
            DBGN(" ");
            DBG(parsed.args[2]);
            this->drive_steps_sloped(parsed.args[0],
                                     {{parsed.args[1], parsed.args[1]}});
            break;

        case serialparser::Command::DriveMM:
            DBGN(F("Driving steps: spd, l, r: "));
            DBGN(parsed.args[0]);
            DBGN(" ");
            DBGN(parsed.args[1]);
            DBGN(" ");
            DBG(parsed.args[2]);
            this->drive_mm(parsed.args[0], {{parsed.args[1], parsed.args[1]}});
            break;
        case serialparser::Command::GetRC:
            Serial.println(this->read_rc(parsed.args[0]));
            break;
        case serialparser::Command::TestMotors:
            this->test_motors();
            break;
        case serialparser::Command::None:
        default:
            Serial.println(F("Command not recognized!"));
            // this is a terrible way to do this, thanks arduino!
            while (Serial.available() > 0) Serial.read();
            return;
    }
}

void PRIZMatic::begin_kb_control(long speed, bool continue_next) {
    /*
     * Drop into RC control and record inputs
     * This function deliberately restricts movements to on/off control at the
     * same speed as would be automatically driven to attempt to ensure maximum
     * reproducibility.
     */
    DBG("Beginning RC control");

    bool done = false;
    unsigned long timer_start = 0;

    while (!done) {
        uint8_t inpBuf;
        if (Serial.available() > 0) {
            auto temp = Serial.read();
            if (temp < 0) {
                continue;
            }
            inpBuf = temp;
        } else {
            continue;
        }
        DBGN(F("DEBUG INCOMING: WASDSig:"));
        Serial.println(inpBuf, 16);

        bool inp_w = inpBuf & 1;
        bool inp_a = (inpBuf >> 1) & 1;
        bool inp_s = (inpBuf >> 2) & 1;
        bool inp_d = (inpBuf >> 3) & 1;
        bool inp_signal = (inpBuf >> 4) & 1;
        auto sw = inp_signal;

        DBG("switch:");
        DBG(sw);
        if (this->readStartButton() || sw) {
            if (!timer_start) {
                // start timer
                timer_start = millis();
            }
        } else if ((!this->readStartButton() || !sw) && timer_start) {
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

        int8_t throttle = inp_w ? inp_w : -inp_s;
        int8_t turn = inp_d ? inp_d : -inp_a;
        DBGN("throttle: ");
        DBGN(throttle);
        DBGN(" turn: ");
        DBG(turn);
        if (turn > 0 || turn < 0) {
            auto turn_dir = sgn<int8_t>(turn);
            this->setMotorSpeeds(speed * turn_dir, -speed * turn_dir);
            // we want to ignore throttle inputs when turning
            // DBGN("turn dir:");
            // DBG(turn_dir);
            continue;
        }

        if (throttle > 0 || throttle < 0) {
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
        if (this->readStartButton() || sw > switch_on) {
            if (!timer_start) {
                // start timer
                timer_start = millis();
            }
        } else if ((this->readStartButton() || sw < switch_on) && timer_start) {
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

        auto throttle = read_rc(fwd_rev_pin);
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