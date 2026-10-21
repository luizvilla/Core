/* Host stubs of the OwnTech APIs used by src/main.cpp. Every call that
 * changes the hardware is recorded in an event log, with the critical task
 * tick at which it happened. */
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "arm_math.h"

#define printk(...) do { if (stub_verbose) printf(__VA_ARGS__); } while (0)
extern bool stub_verbose;

typedef enum { LEG1 = 0, LEG2, LEG3, LEG4, ALL } leg_t;
typedef enum { V1_LOW = 0, V2_LOW, V_HIGH, I1_LOW, I2_LOW, I_HIGH, SENSOR_COUNT } sensor_t;
typedef uint32_t gpio_flags_t;
const gpio_flags_t OUTPUT = 1;
const float32_t NO_VALUE = -10000;

struct Event { uint32_t tick; std::string what; };
extern std::vector<Event> events;
extern uint32_t tick;
void log_event(const std::string &what);

struct PowerStub {
    float32_t duty[4] = {0, 0, 0, 0};
    float32_t dmin[4] = {0.1F, 0.1F, 0.1F, 0.1F};
    float32_t dmax[4] = {0.9F, 0.9F, 0.9F, 0.9F};
    bool on[4] = {false, false, false, false};
    int convention[4] = {0, 0, 0, 0};   /* 1 buck, 2 boost */
    void initBuck(leg_t l);
    void initBoost(leg_t l);
    void setDutyCycleMin(leg_t l, float32_t d) { dmin[l] = d; }
    void setDutyCycleMax(leg_t l, float32_t d) { dmax[l] = d; }
    void setDutyCycle(leg_t l, float32_t d);
    void start(leg_t l);
    void stop(leg_t l);
};
struct SensorsStub {
    float32_t value[SENSOR_COUNT] = {0};
    void enableDefaultTwistSensors() {}
    void setConversionParametersLinear(sensor_t, float32_t, float32_t) {}
    float32_t getLatestValue(sensor_t s, uint8_t * = nullptr) { return value[s]; }
};
struct ShieldStub { PowerStub power; SensorsStub sensors; };
extern ShieldStub shield;

struct GpioStub {
    void configurePin(uint8_t, gpio_flags_t) {}
    void setPin(uint8_t pin);
    void resetPin(uint8_t pin);
};
struct LedStub { void turnOn() {} void turnOff() {} };
struct SpinStub { GpioStub gpio; LedStub led; };
extern SpinStub spin;

struct TaskStub {
    uint32_t createBackground(void (*)()) { return 0; }
    void createCritical(void (*)(), uint32_t) {}
    void startBackground(uint32_t) {}
    void startCritical() {}
    void suspendBackgroundMs(uint32_t) {}
};
extern TaskStub task;
