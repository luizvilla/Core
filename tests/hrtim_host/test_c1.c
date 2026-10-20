/* Commit 1: hrtim_change_frequency() period rounding */
#include "fake_regs.h"

int main(void)
{
    fake_regs_init();
    hrtim_init_default_all();
    hrtim_frequency_set(200000, 50000);
    hrtim_set_modulation(PWMA, Lft_aligned);
    hrtim_tu_init(PWMA);

    /* 170 MHz x 32 / 150 kHz / 2^ckpsc = 18133.3 */
    hrtim_change_frequency(150000);
    printf("MPER=%u PERAR=%u\n", HRTIM1->sMasterRegs.MPER, TIMREG(PWMA).PERxR);
    CHECK(HRTIM1->sMasterRegs.MPER == 18133, "MPER = 18133 at 150 kHz");
    CHECK(TIMREG(PWMA).PERxR == 18133, "PERAR = 18133 at 150 kHz");
    printf("%d failure(s)\n", failures);
    return failures != 0;
}
