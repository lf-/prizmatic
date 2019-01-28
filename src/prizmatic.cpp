#include <PRIZM.h>
#include <initializer_list>
#include <stdint.h>

struct Step
{
    long left;
    long right;
};

class PRIZMatic : PRIZM
{
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void wait_for_start_button();
    void begin_rc_control(long speed);
    int8_t read_rc(uint8_t pin);
};

void
PRIZMatic::drive_steps(long speed, std::initializer_list<Step> steps)
{
    for (auto step : steps) {
        this->setMotorTargets(speed, step.left, speed, step.right);
        while (this->readMotorBusy(1) || this->readMotorBusy(2)) {
        }
        this->resetEncoders();
    }
}

void
PRIZMatic::wait_for_start_button()
{
    while (!this->readStartButton()) {
    }
}

int8_t
PRIZMatic::read_rc(uint8_t pin)
{
    return map(pulseIn(pin, HIGH, 25000), 500, 2500, -INT8_MIN, INT8_MAX);
}

template<typename T>
int
sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void
PRIZMatic::begin_rc_control(long speed)
{
    /*
     * Drop into RC control and record inputs
     * This function deliberately restricts movements to on/off control at the
     * same speed as would be automatically driven to attempt to ensure maximum
     * reproducibility.
     */

    // change these if signalling switch doesn't work
    const int8_t switch_on = 50;
    const int8_t switch_off = -50;
    const uint8_t signal_switch_pin = 3; // Port D3
    const uint8_t fwd_rev_pin = 4;       // Port D4
    const uint8_t turn_pin = 5;          // Port D5

    bool done = 0;
    unsigned long done_timer = 0;

    while (!done) {
        auto sw = read_rc(signal_switch_pin);
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

        auto throttle = read_rc(fwd_rev_pin);
        auto turn = read_rc(turn_pin);
        if (turn > switch_on || turn < switch_off) {
            auto turn_dir = sgn<int8_t>(turn);
            this->setMotorSpeeds(speed * turn_dir, -speed * turn_dir);
            // we want to ignore throttle inputs when turning
            continue;
        }

        if (throttle > switch_on || throttle < switch_off) {
            auto motor_speed = speed * sgn<int8_t>(throttle);
            this->setMotorSpeeds(motor_speed, motor_speed);
            continue;
        }

        this->setMotorSpeeds(0, 0);
    }
}

void
setup()
{}

void
loop()
{}
