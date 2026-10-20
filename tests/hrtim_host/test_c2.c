/* Commit 2: per-unit frequency at init, master paths skip the unit */
#include "fake_regs.h"

int main(void)
{
    fake_regs_init();
    hrtim_init_default_all();
    hrtim_frequency_set(200000, 50000);

    CHECK(hrtim_tu_frequency_set(PWMD, 40000) == -1, "40 kHz refused (min 50 kHz)");
    CHECK(!hrtim_tu_has_own_frequency(PWMD), "refusal leaves PWMD on master frequency");
    CHECK(hrtim_tu_frequency_set(PWMD, 50000) == 0, "50 kHz accepted");
    CHECK(hrtim_tu_frequency_set(PWME, 50000) == 0, "50 kHz accepted on PWME");

    /* initMode() calls this again for every leg */
    hrtim_frequency_set(200000, 50000);
    CHECK(hrtim_tu_frequency_get(PWMD) == 50000, "PWMD keeps 50 kHz after hrtim_frequency_set");
    CHECK(hrtim_tu_frequency_get(PWMA) == 200000, "PWMA follows master (200 kHz)");

    hrtim_set_modulation(PWMA, Lft_aligned);
    hrtim_tu_init(PWMA);
    hrtim_set_modulation(PWMD, UpDwn);
    hrtim_tu_init(PWMD);
    hrtim_set_modulation(PWME, Lft_aligned);
    hrtim_tu_init(PWME);

    CHECK(TIMREG(PWMA).PERxR == 13600, "PERAR = 13600 (200 kHz)");
    CHECK(TIMREG(PWMD).PERxR == 27200, "PERDR = 27200 (50 kHz up-down)");
    CHECK(TIMREG(PWME).PERxR == 54400, "PERER = 54400 (50 kHz left-aligned)");
    CHECK(CKPSC(PWMD) == 1 && CKPSC(PWME) == 1, "PWMD/PWME ckpsc = 1");
    printf("  period_get_us: PWMA=%u PWMD=%u PWME=%u\n", hrtim_period_get_us(PWMA),
           hrtim_period_get_us(PWMD), hrtim_period_get_us(PWME));
    CHECK(hrtim_period_get_us(PWMD) == 20 && hrtim_period_get_us(PWME) == 20,
          "period_get_us = 20 us on PWMD/PWME");

    hrtim_change_frequency(150000);
    CHECK(TIMREG(PWMA).PERxR == 18133, "PERAR follows master change (18133)");
    CHECK(TIMREG(PWMD).PERxR == 27200, "PERDR unchanged by master change");
    CHECK(TIMREG(PWME).PERxR == 54400, "PERER unchanged by master change");
    CHECK(hrtim_tu_frequency_get(PWMD) == 50000, "PWMD frequency field unchanged");

    /* Second init of the same unit (initBuck then initBoost) */
    hrtim_frequency_set(200000, 50000);
    hrtim_tu_init(PWMD);
    CHECK(TIMREG(PWMD).PERxR == 27200, "PERDR still 27200 after a second init");
    printf("%d failure(s)\n", failures);
    return failures != 0;
}
