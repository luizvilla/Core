/* Host tests of src/main.cpp: sequencing, interlocks, protections, keys.
 * Uses the real single_phase_inverter and control_library (pid, trigo,
 * transform) with stubbed OwnTech APIs. */
#include <deque>
#include <cmath>
#include "stubs.h"

bool stub_verbose = false;
std::vector<Event> events;
uint32_t tick = 0;
ShieldStub shield;
SpinStub spin;
TaskStub task;
static std::deque<int> keys;
/* When set, V_AC is a 325 V peak 50 Hz sine */
static bool grid_on = false;

void log_event(const std::string &what) { events.push_back({tick, what}); }
void PowerStub::initBuck(leg_t l) { convention[l] = 1; log_event("initBuck " + std::to_string(l + 1)); }
void PowerStub::initBoost(leg_t l) { convention[l] = 2; log_event("initBoost " + std::to_string(l + 1)); }
void PowerStub::setDutyCycle(leg_t l, float32_t d)
{
    duty[l] = d < dmin[l] ? dmin[l] : (d > dmax[l] ? dmax[l] : d);
}
void PowerStub::start(leg_t l) { on[l] = true; log_event("start " + std::to_string(l + 1)); }
void PowerStub::stop(leg_t l)
{
    if (l == ALL) { for (int i = 0; i < 4; i++) on[i] = false; log_event("stop ALL"); return; }
    on[l] = false; log_event("stop " + std::to_string(l + 1));
}
void GpioStub::setPin(uint8_t pin) { log_event("pin " + std::to_string(pin) + " high"); }
void GpioStub::resetPin(uint8_t pin) { log_event("pin " + std::to_string(pin) + " low"); }
int console_getchar(void) { int k = keys.front(); keys.pop_front(); return k; }

#define main firmware_main
#include "main.cpp"
#undef main

static int failures = 0;
#define CHECK(cond, ...) do {                       \
        bool ok_ = (cond);                          \
        printf(ok_ ? "  ok   " : "  FAIL ");        \
        if (!ok_) failures++;                       \
        printf(__VA_ARGS__);                        \
        printf("\n");                               \
    } while (0)

static void key(char k) { keys.push_back(k); loop_communication_task(); }
static void run(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        if (grid_on)
            shield.sensors.value[V2_LOW] = 325.0F * sinf(W0 * TS * (float32_t)tick);
        loop_critical_task();
        tick++;
    }
}
static long find(const std::string &what, uint32_t from = 0)
{
    for (const Event &e : events)
        if (e.tick >= from && e.what == what) return (long)e.tick;
    return -1;
}
static void reset_world()
{
    events.clear();
    for (int i = 0; i < SENSOR_COUNT; i++) shield.sensors.value[i] = 0.0F;
    key('i');
    run(2);
    events.clear();
}

/* Pin 21 high = supply on with HV_SUPPLY_ON_LEVEL = true */
static const char *SUPPLY_ON = "pin 21 high";
static const char *SUPPLY_OFF = "pin 21 low";

/* DC bus present, AC overcurrent and DC overvoltage values, in the units of
 * the build (raw counts by default, V / A with MEASUREMENTS_CALIBRATED) */
static const float32_t VDC_OK = MEASUREMENTS_CALIBRATED ? 100.0F : 1000.0F;
static const float32_t VDC_LOW = MEASUREMENTS_CALIBRATED ? 2.0F : 20.0F;
static const float32_t VDC_TRIP = MEASUREMENTS_CALIBRATED ? 450.0F : 3300.0F;
static const float32_t IAC_TRIP = MEASUREMENTS_CALIBRATED ? 3.5F : 4001.0F;
static const float32_t IAC_ZERO = MEASUREMENTS_CALIBRATED ? 0.0F : 2000.0F;

int main()
{
    firmware_main();
    printf("== setup\n");
    CHECK(shield.power.convention[2] == 1, "LEG3 buck");
    CHECK(shield.power.convention[3] == 1, "LEG4 buck (unipolar default)");
    CHECK(fabsf(shield.power.dmin[3] - 0.05F) < 1e-6F && fabsf(shield.power.dmax[3] - 0.95F) < 1e-6F,
          "LEG4 duty limits 0.05 - 0.95 after its init");
    CHECK(find(SUPPLY_OFF) >= 0 && find(SUPPLY_ON) < 0, "bridge supply off at setup");

    printf("== inverter fixed u, start sequence\n");
    reset_world();
    shield.sensors.value[V_HIGH] = VDC_OK;
    key('v'); key('k'); key('k'); key('k');    /* u = 0.03 */
    run(HV_SUPPLY_SETTLE_TICKS + 10);
    long t_on = find(SUPPLY_ON), t_l3 = find("start 3"), t_l4 = find("start 4");
    CHECK(t_on >= 0 && t_l3 >= 0 && t_l4 >= 0, "supply on, LEG3 and LEG4 started");
    CHECK(t_l3 - t_on >= (long)HV_SUPPLY_SETTLE_TICKS - 1,
          "PWM starts %ld ticks after the supply (>= %u)", t_l3 - t_on, HV_SUPPLY_SETTLE_TICKS - 1);
    CHECK(fabsf(shield.power.duty[2] - 0.515F) < 1e-4F && fabsf(shield.power.duty[3] - 0.485F) < 1e-4F,
          "unipolar: d3 = 0.515, d4 = 1 - d3 (%.3f, %.3f)", shield.power.duty[2], shield.power.duty[3]);
    CHECK(!shield.power.on[1], "boost not started by the inverter");

    printf("== inverter stop sequence\n");
    events.clear();
    key('x'); run(3);
    long t_s3 = find("stop 3"), t_off = find(SUPPLY_OFF);
    CHECK(t_s3 >= 0 && t_off >= 0 && t_off >= t_s3, "PWM off, then supply off");
    bool stop_first = false;
    for (const Event &e : events) { if (e.what == "stop 3") { stop_first = true; break; } if (e.what == SUPPLY_OFF) break; }
    CHECK(stop_first, "stop 3 logged before the supply off");

    printf("== bridge supply only (key z)\n");
    reset_world();
    key('z'); run(1000);
    CHECK(find(SUPPLY_ON) >= 0 && find(SUPPLY_OFF) < 0, "z: supply on and kept on (100 ms)");
    CHECK(find("start 3") < 0 && find("start 4") < 0, "z: no PWM");
    CHECK(hv_supply_on, "log field: supply on");
    events.clear();
    key('z'); run(2);
    CHECK(find(SUPPLY_OFF) >= 0 && !hv_supply_on, "z again: supply off");
    /* z with no DC bus does not try to start anything */
    events.clear();
    shield.sensors.value[V_HIGH] = 0.0F;
    key('z'); run(500);
    CHECK(find(SUPPLY_OFF) < 0, "z with no DC bus: supply stays on (no start check)");
    /* inverter start takes over the supply */
    shield.sensors.value[V_HIGH] = VDC_OK;
    events.clear();
    key('v'); run(HV_SUPPLY_SETTLE_TICKS + 5);
    CHECK(find("start 3") >= 0 && find(SUPPLY_OFF) < 0, "z then v: supply kept, PWM after the wait");
    key('z');
    CHECK(hv_supply_on && shield.power.on[2], "z refused while the inverter runs");
    events.clear();
    key('x'); run(3);
    CHECK(find(SUPPLY_OFF) >= 0 && !hv_supply_on, "x: supply off");
    key('z'); run(2); key('i'); run(2);
    CHECK(!hv_supply_on, "z then i: supply off");
    key('z'); run(2);
    shield.sensors.value[I_HIGH] = IAC_TRIP; run(1);
    CHECK(fault && !hv_supply_on, "z then fault: supply off");
    shield.sensors.value[I_HIGH] = IAC_ZERO; key('i'); run(2);
    CHECK(!fault && !hv_supply_on, "fault cleared, supply still off");

    printf("== grid relay (key r)\n");
    reset_world();
    key('r'); run(1);
    CHECK(find("pin 22 high") >= 0 && relay_closed, "r: relay closed (pin 22 high)");
    events.clear();
    key('r'); run(1);
    CHECK(find("pin 22 low") >= 0 && !relay_closed, "r again: relay open");
    key('r'); run(1); key('i'); run(1);
    CHECK(!relay_closed, "i opens the relay");
    key('r'); run(1);
    shield.sensors.value[V_HIGH] = VDC_TRIP; run(1);
    CHECK(fault && !relay_closed, "fault opens the relay");
    key('r'); run(1);
    CHECK(!relay_closed, "r refused (stays open) during a fault");
    shield.sensors.value[V_HIGH] = 0.0F; key('i'); run(2);

    printf("== start interlock (no DC bus, check on)\n");
    reset_world();
    CHECK(!bus_check_enabled, "bus check off by default");
    key('d');
    CHECK(bus_check_enabled, "d: bus check on");
    shield.sensors.value[V_HIGH] = VDC_LOW;
    key('s'); run(HV_SUPPLY_SETTLE_TICKS + 10);
    CHECK(find("start 3") < 0, "not started with V_DC = %.0f (below %.0f)", VDC_LOW, V_DC_MIN_START);
    CHECK(inverter_state == INV_OFF, "inverter back to off");
    run(3);
    CHECK(find(SUPPLY_OFF) >= 0, "supply switched off again");
    events.clear();
    key('v'); run(HV_SUPPLY_SETTLE_TICKS + 10);
    CHECK(find("start 3") < 0 && inverter_state == INV_OFF, "fixed u refused too with the check on");
    run(3);
    key('d');
    CHECK(!bus_check_enabled, "d again: bus check off");

    printf("== check off: starts without DC bus (ramped by hand)\n");
    reset_world();
    shield.sensors.value[V_HIGH] = 0.0F;
    key('v'); run(HV_SUPPLY_SETTLE_TICKS + 5);
    CHECK(find("start 3") >= 0 && find("start 4") >= 0, "v with V_DC = 0: PWM on");
    CHECK(inverter_state == INV_FIXED && find(SUPPLY_OFF) < 0, "still running, supply on");
    shield.sensors.value[V_HIGH] = VDC_OK; run(10);
    CHECK(shield.power.on[2], "bus ramped up: still running");
    key('x'); run(3);
    events.clear();
    shield.sensors.value[V_HIGH] = 0.0F;
    key('s'); run(HV_SUPPLY_SETTLE_TICKS + 5);
    CHECK(find("start 3") >= 0, "s with V_DC = 0 and the check off: PWM on");
    key('x'); run(3);

    printf("== open-loop sine\n");
    reset_world();
    shield.sensors.value[V_HIGH] = VDC_OK;
    key('s'); for (int i = 0; i < 8; i++) key('k');   /* m = 0.4 */
    run(HV_SUPPLY_SETTLE_TICKS + 5);
    float32_t dmin3 = 1, dmax3 = 0, worst_sum = 0;
    for (int i = 0; i < 400; i++)
    {
        run(1);
        float32_t d3 = shield.power.duty[2], d4 = shield.power.duty[3];
        if (d3 < dmin3) dmin3 = d3;
        if (d3 > dmax3) dmax3 = d3;
        if (fabsf(d3 + d4 - 1.0F) > worst_sum) worst_sum = fabsf(d3 + d4 - 1.0F);
    }
    CHECK(fabsf(dmax3 - 0.7F) < 0.01F && fabsf(dmin3 - 0.3F) < 0.01F,
          "m = 0.4: d3 from %.3f to %.3f (0.3 - 0.7)", dmin3, dmax3);
    CHECK(worst_sum < 1e-5F, "unipolar: d3 + d4 = 1 all along");

    printf("== modulation switch\n");
    key('b');
    CHECK(modulation == UNIPOLAR, "refused while running");
    key('x'); run(3);
    events.clear();
    key('b');
    CHECK(modulation == BIPOLAR && shield.power.convention[3] == 2, "bipolar: LEG4 boost convention");
    CHECK(inverter.getModulation() == BIPOLAR, "library modulation follows");
    CHECK(fabsf(shield.power.dmin[3] - 0.05F) < 1e-6F, "LEG4 limits set again after the re-init");
    key('v'); run(HV_SUPPLY_SETTLE_TICKS + 5);
    CHECK(fabsf(shield.power.duty[2] - shield.power.duty[3]) < 1e-6F, "bipolar: d4 = d3");
    key('x'); run(3); key('n');
    CHECK(modulation == UNIPOLAR && shield.power.convention[3] == 1, "back to unipolar: LEG4 buck");

    printf("== protections\n");
    reset_world();
    shield.sensors.value[V_HIGH] = VDC_OK;
    shield.sensors.value[I_HIGH] = IAC_ZERO;
    key('p'); key('v'); run(HV_SUPPLY_SETTLE_TICKS + 5);
    CHECK(shield.power.on[1] && shield.power.on[2], "boost and inverter running");
    events.clear();
    shield.sensors.value[I_HIGH] = IAC_TRIP;
    run(1);
    CHECK(fault && find("stop ALL") >= 0 && find(SUPPLY_OFF) >= 0, "AC overcurrent: stop ALL, supply off");
    CHECK(!shield.power.on[1] && !shield.power.on[3], "all legs off");
    shield.sensors.value[I_HIGH] = IAC_ZERO;
    key('p');
    CHECK(boost_state == BOOST_OFF, "keys refused while in fault");
    key('i'); run(2);
    CHECK(!fault, "fault cleared with i");
    shield.sensors.value[V_HIGH] = VDC_TRIP;
    key('p'); run(2);
    CHECK(fault && !shield.power.on[1], "DC overvoltage trips the boost");
    shield.sensors.value[V_HIGH] = VDC_OK;
    if (!MEASUREMENTS_CALIBRATED)
    {
        /* raw: only the upper side trips (zero current reads ~2000) */
        key('i'); run(2);
        shield.sensors.value[I_HIGH] = 0.0F;
        run(2);
        CHECK(!fault, "raw: I_AC = 0 counts does not trip (no lower threshold)");
        shield.sensors.value[I_HIGH] = 4000.0F;
        run(2);
        CHECK(!fault, "raw: I_AC = 4000 counts does not trip (> 4000 only)");
        shield.sensors.value[I_HIGH] = IAC_ZERO;
    }

    printf("== boost voltage loop\n");
    reset_world();
    shield.sensors.value[V_HIGH] = 10.0F;
    key('g'); run(200);
    CHECK(shield.power.on[1], "boost started in voltage loop");
    CHECK(duty_cycle > 0.0F && duty_cycle <= 0.7F, "PID duty within 0 - 0.7 (%.3f)", duty_cycle);
    key('i'); run(2);

    if (!MEASUREMENTS_CALIBRATED)
    {
        printf("== grid forming / following refused with raw measurements\n");
        reset_world();
        shield.sensors.value[V_HIGH] = VDC_OK;
        key('f');
        CHECK(inverter_state == INV_OFF, "f refused");
        key('w');
        CHECK(inverter_state == INV_OFF, "w refused");
        run(HV_SUPPLY_SETTLE_TICKS + 5);
        CHECK(find(SUPPLY_ON) < 0 && find("start 3") < 0, "no supply, no PWM");
    }
    else
    {
        printf("== grid forming\n");
        reset_world();
        shield.sensors.value[V_HIGH] = 100.0F;
        key('f'); for (int i = 0; i < 4; i++) key('k');   /* Vd ref 20 V */
        run(HV_SUPPLY_SETTLE_TICKS + 5);
        CHECK(shield.power.on[2] && shield.power.on[3], "forming: PWM on after the supply wait");
        float32_t worst = 0;
        for (int i = 0; i < 400; i++)
        {
            run(1);
            float32_t e = fabsf(shield.power.duty[2] + shield.power.duty[3] - 1.0F);
            if (e > worst) worst = e;
        }
        CHECK(worst < 1e-4F, "forming, unipolar: LEG4 = 1 - LEG3 from getDutyLeg2()");
        key('x'); run(3);

        printf("== grid following\n");
        reset_world();
        grid_on = true;
        /* Bus too low to reproduce a 325 V peak grid (300 x 0.9 < 325) */
        shield.sensors.value[V_HIGH] = 300.0F;
        key('w');
        run(20000);
        CHECK(inverter.getSync(), "PLL locked on the 50 Hz sine");
        CHECK(find("start 3") < 0, "no PWM while V_DC x U_MAX < grid peak");
        /* Bus high enough */
        shield.sensors.value[V_HIGH] = 380.0F;
        run(400);
        long t_start = find("start 3");
        CHECK(t_start >= 0, "PWM on once the bus is high enough");
        CHECK(!inverter.getSync() || t_start > find(SUPPLY_ON), "PWM after the supply");
        /* PWM reproduces the grid voltage before any injection: u = v / V_DC */
        float32_t worst_v = 0;
        for (int i = 0; i < 400; i++)
        {
            run(1);
            float32_t v = shield.sensors.value[V2_LOW];
            float32_t u = 2.0F * shield.power.duty[2] - 1.0F;
            float32_t e = fabsf(u * 380.0F - v);
            if (e > worst_v) worst_v = e;
        }
        printf("  max |u V_DC - v_grid| = %.1f V\n", worst_v);
        CHECK(worst_v < 30.0F, "duty follows the grid voltage before injection (< 30 V)");
        CHECK(fabsf(shield.power.duty[2] + shield.power.duty[3] - 1.0F) < 1e-4F, "unipolar LEG4 = 1 - LEG3");
        key('e');
        CHECK(following_power_on, "e: power injection requested");
        key('x'); run(3);
        CHECK(!shield.power.on[2] && find(SUPPLY_OFF, (uint32_t)t_start) >= 0, "stopped, supply off");
        grid_on = false;
    }

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
