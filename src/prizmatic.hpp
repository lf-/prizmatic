#pragma once
#include "prizheaders.h"

#include "config.h"
#include "serialparser/serialparser.hpp"

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

struct MMStep {
    float left;
    float right;
};

namespace {
int const MAX_STEPS = 50;
Step saved_steps[MAX_STEPS];
uint8_t n_saved_steps;

uint8_t const signal_switch_pin = 17;  // Port A1
uint8_t const turn_pin = 15;           // Port A2
uint8_t const fwd_rev_pin = 16;        // Port A3

uint8_t const SONIC_PIN = 2;  // Port D2

float const final_speed_dist = 10.f;      // 10mm
float const tetrix_wheel_radius = 48.8f;  // millimeters
}  // namespace

typedef float (*GetDistance)();

class PRIZMatic : public PRIZM {
   public:
    PRIZMatic();
    void drive_steps(long speed, std::initializer_list<Step> steps);
    void drive_steps_sloped(long speed, std::initializer_list<Step> steps);
    void drive_steps_sloped_betterer(long speed,
                                     std::initializer_list<Step> steps);
    void drive_mm(long speed, std::initializer_list<MMStep> steps);
    void drive_sensor(long speed, float distance, GetDistance fn);

    void wait_for_start_button();

    void begin_rc_control(long speed, bool continue_next = false);
    void begin_kb_control(long speed, bool continue_next = false);

    void begin_rc_servo_test(uint8_t servonum);
    void send_steps();
    float readSonicSensorMM(uint8_t pin);

    void dump_eeprom_steps();
    int8_t read_rc(uint8_t pin);

    // wrap the parent class to save invert state
    void setMotorInvert(int channel, int invert);
    void debug_controller();

   private:
    bool channel_inverts[3];
    void handle_serial();
    void test_motors();
    void dump_encoder_counts();
};

template <typename T>
int sgn(T val);
