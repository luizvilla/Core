/* Commit 3: independent units are not reset by the master */
#include "fake_regs.h"

#define MASTER_RSTS (LL_HRTIM_RESETTRIG_MASTER_PER | LL_HRTIM_RESETTRIG_MASTER_CMP1 | \
    LL_HRTIM_RESETTRIG_MASTER_CMP2 | LL_HRTIM_RESETTRIG_MASTER_CMP3 | LL_HRTIM_RESETTRIG_MASTER_CMP4)

int main(void)
{
    fake_regs_init();
    hrtim_init_default_all();
    hrtim_frequency_set(200000, 50000);
    hrtim_tu_frequency_set(PWMD, 50000);
    hrtim_tu_frequency_set(PWME, 50000);

    hrtim_set_modulation(PWMA, Lft_aligned);  hrtim_tu_init(PWMA);
    hrtim_set_modulation(PWMC, Lft_aligned);  hrtim_tu_init(PWMC);
    hrtim_set_modulation(PWMD, UpDwn);        hrtim_tu_init(PWMD);
    hrtim_set_modulation(PWME, UpDwn);        hrtim_tu_init(PWME);

    printf("  RSTAR=0x%08x RSTCR=0x%08x RSTDR=0x%08x RSTER=0x%08x\n",
           TIMREG(PWMA).RSTxR, TIMREG(PWMC).RSTxR, TIMREG(PWMD).RSTxR, TIMREG(PWME).RSTxR);
    CHECK((TIMREG(PWMD).RSTxR & MASTER_RSTS) == 0, "PWMD: no master reset source");
    CHECK((TIMREG(PWME).RSTxR & MASTER_RSTS) == 0, "PWME: no master reset source");
    CHECK(TIMREG(PWMA).RSTxR & LL_HRTIM_RESETTRIG_MASTER_PER, "PWMA: still reset on master PER");
    CHECK(TIMREG(PWMC).RSTxR & LL_HRTIM_RESETTRIG_MASTER_PER, "PWMC: still reset on master PER");

    CHECK(hrtim_phase_shift_set(PWMD, 1000) == -1, "phase shift on PWMD refused");
    CHECK((TIMREG(PWMD).RSTxR & MASTER_RSTS) == 0, "PWMD still free-running after refusal");
    CHECK(hrtim_phase_shift_set(PWMD, 0) == 0, "zero phase shift on PWMD accepted");
    CHECK(hrtim_phase_shift_set(PWMC, 1000) == 0, "phase shift on PWMC accepted");
    CHECK(TIMREG(PWMC).RSTxR & LL_HRTIM_RESETTRIG_MASTER_CMP2, "PWMC reset on master CMP2 (shifted)");

    hrtim_change_frequency(150000);
    CHECK((TIMREG(PWMD).RSTxR & MASTER_RSTS) == 0, "PWMD free-running after master change");

    /* Current mode refused on an independent unit */
    hrtim_tu_frequency_set(PWMF, 50000);
    hrtim_pwm_mode_set(PWMF, CURRENT_MODE);
    CHECK(hrtim_tu_init(PWMF) == 0, "current mode on own-frequency unit refused (returns 0)");

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
