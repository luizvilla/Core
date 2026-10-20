/* Commit 4: runtime frequency change of an independent unit, CMP3 rescale */
#include "fake_regs.h"

int main(void)
{
    fake_regs_init();
    hrtim_init_default_all();
    hrtim_frequency_set(200000, 50000);
    hrtim_tu_frequency_set(PWMD, 50000);
    hrtim_set_modulation(PWMA, Lft_aligned);  hrtim_tu_init(PWMA);
    hrtim_set_modulation(PWMD, UpDwn);        hrtim_tu_init(PWMD);

    hrtim_duty_cycle_set(PWMD, 13600);           /* 50 % of 27200 */
    hrtim_tu_cmp_set(PWMD, CMP3xR, 6800);        /* ADC trigger at 25 % */
    hrtim_duty_cycle_set(PWMA, 3400);            /* 25 % of 13600 */
    hrtim_tu_cmp_set(PWMA, CMP3xR, 6800);        /* 50 % */

    CHECK(hrtim_tu_change_frequency(PWMA, 100000) == -1, "PWMA follows master: refused");
    CHECK(hrtim_tu_change_frequency(PWMD, 30000) == -1, "30 kHz below minimum: refused");
    CHECK(TIMREG(PWMD).PERxR == 27200, "PERDR unchanged after refusals");

    CHECK(hrtim_tu_change_frequency(PWMD, 80000) == 0, "PWMD to 80 kHz");
    printf("  PERDR=%u CMP1DR=%u CMP3DR=%u\n", TIMREG(PWMD).PERxR,
           TIMREG(PWMD).CMP1xR, TIMREG(PWMD).CMP3xR);
    CHECK(TIMREG(PWMD).PERxR == 17000, "PERDR = 17000 (80 kHz up-down)");
    CHECK(TIMREG(PWMD).CMP1xR == 8500, "CMP1DR = 8500 (still 50 %)");
    CHECK(TIMREG(PWMD).CMP3xR == 4250, "CMP3DR = 4250 (still 25 %)");
    CHECK(hrtim_tu_frequency_get(PWMD) == 80000, "frequency_get = 80000");
    CHECK(hrtim_period_get_us(PWMD) == 12, "period_get_us = 12 (12.5 us truncated)");
    CHECK(TIMREG(PWMA).PERxR == 13600, "PERAR unchanged");

    hrtim_change_frequency(150000);
    printf("  PERAR=%u CMP1AR=%u CMP3AR=%u\n", TIMREG(PWMA).PERxR,
           TIMREG(PWMA).CMP1xR, TIMREG(PWMA).CMP3xR);
    CHECK(TIMREG(PWMA).CMP3xR == 9066, "master change rescales CMP3AR (50 %)");
    CHECK(TIMREG(PWMD).PERxR == 17000, "PERDR unchanged by master change");

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
