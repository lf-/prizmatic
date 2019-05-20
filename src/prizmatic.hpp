#pragma once
#include "prizheaders.h"

#include "serialparser/serialparser.hpp"
#include "config.h"

#ifdef DEBUG_PRIZMATIC
#define DBG(s) Serial.println(s)
#define DBGN(s) Serial.print(s)
#else
#define DBG(s)
#define DBGN(s)
#endif

long degrees_in_counts(float degs);

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


