/* Replays PowerAPI::initMode() for LEG1..LEG4 of the micro-verter overlay:
 * initBoost(LEG1), initBoost(LEG2), initBuck(LEG3), initBoost(LEG4),
 * LEG3/LEG4 with leg-frequency = 50 kHz, then the runtime changes. */
#include "fake_regs.h"

#define MASTER_RSTS (LL_HRTIM_RESETTRIG_MASTER_PER | LL_HRTIM_RESETTRIG_MASTER_CMP1 | \
    LL_HRTIM_RESETTRIG_MASTER_CMP2 | LL_HRTIM_RESETTRIG_MASTER_CMP3 | LL_HRTIM_RESETTRIG_MASTER_CMP4)

static const hrtim_tu_number_t leg_tu[4] = {PWMA, PWMC, PWMD, PWME};
static const uint32_t leg_freq[4] = {0, 0, 50000, 50000};
static const hrtim_switch_convention_t leg_conv[4] = {PWMx2, PWMx2, PWMx1, PWMx2};

static void align(void)  /* PowerAPI::alignOwnFrequencyLegs() */
{
    hrtim_tu_number_t group[4]; uint8_t n = 0;
    for (int i = 0; i < 4; i++)
        if (leg_freq[i] == 50000 && hrtim_tu_has_own_frequency(leg_tu[i]))
            group[n++] = leg_tu[i];
    if (n > 1) hrtim_cnt_reset_sync(group, n);
}

static void init_mode(int i)
{
    if (!hrtim_get_status(PWMA)) hrtim_init_default_all();   /* initVariableFrequency */
    hrtim_frequency_set(200000, 50000);
    if (leg_freq[i]) hrtim_tu_frequency_set(leg_tu[i], leg_freq[i]);
    hrtim_set_modulation(leg_tu[i], UpDwn);
    hrtim_set_switch_convention(leg_tu[i], leg_conv[i]);
    hrtim_tu_init(leg_tu[i]);
    hrtim_phase_shift_set(leg_tu[i], 0);
    hrtim_dt_set(leg_tu[i], 100, 100);
    align();
}

int main(void)
{
    fake_regs_init();
    for (int i = 0; i < 4; i++) init_mode(i);

    printf("  PER A/C/D/E = %u %u %u %u\n", TIMREG(PWMA).PERxR, TIMREG(PWMC).PERxR,
           TIMREG(PWMD).PERxR, TIMREG(PWME).PERxR);
    CHECK(TIMREG(PWMA).PERxR == 6800 && TIMREG(PWMC).PERxR == 6800,
          "LEG1/LEG2: 200 kHz up-down (6800)");
    CHECK(TIMREG(PWMD).PERxR == 27200 && TIMREG(PWME).PERxR == 27200,
          "LEG3/LEG4: 50 kHz up-down (27200)");
    CHECK((TIMREG(PWMD).RSTxR & MASTER_RSTS) == 0 && (TIMREG(PWME).RSTxR & MASTER_RSTS) == 0,
          "LEG3/LEG4: no master reset");
    CHECK((TIMREG(PWMC).RSTxR & LL_HRTIM_RESETTRIG_MASTER_PER) != 0, "LEG2: reset on master PER");
    CHECK((HRTIM1->sCommonRegs.CR2 & (HRTIM_CR2_TDRST | HRTIM_CR2_TERST)) ==
          (HRTIM_CR2_TDRST | HRTIM_CR2_TERST), "LEG3/LEG4 counters reset together");
    CHECK((HRTIM1->sCommonRegs.CR2 & (HRTIM_CR2_TARST | HRTIM_CR2_TCRST)) == 0,
          "LEG1/LEG2 counters not reset");
    CHECK(hrtim_period_get_us(PWMD) == 20, "LEG3 period 20 us");

    /* shield.power.setFrequency(150000): boost only */
    hrtim_change_frequency(150000);
    CHECK(TIMREG(PWMD).PERxR == 27200, "LEG3 unchanged by setFrequency(150 kHz)");
    CHECK(TIMREG(PWMA).PERxR == 9066, "LEG1 at 150 kHz up-down (9066)");

    /* shield.power.setFrequency(LEG3, 80000) */
    CHECK(hrtim_tu_change_frequency(PWMD, 80000) == 0, "LEG3 to 80 kHz");
    CHECK(TIMREG(PWMD).PERxR == 17000 && TIMREG(PWME).PERxR == 27200,
          "LEG3 17000, LEG4 still 27200");
    CHECK(hrtim_tu_change_frequency(PWMA, 80000) == -1, "LEG1 (shared) refused");

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
