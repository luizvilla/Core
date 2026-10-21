/* Inverter legs in up-down mode: buck (PWMx1) on timer D, boost (PWMx2) on
 * timer E, as initBuck(LEG3) / initBoost(LEG4) do for the bipolar modulation.
 * Expected: same output 1 events on both units, only the swap bit differs,
 * dead time enabled on both. */
#include "fake_regs.h"

static void init_leg(hrtim_tu_number_t tu, hrtim_switch_convention_t conv)
{
    /* Order of PowerAPI::initMode() */
    if (!hrtim_get_status(PWMA)) hrtim_init_default_all();
    hrtim_frequency_set(200000, 50000);
    hrtim_tu_frequency_set(tu, 50000);
    hrtim_set_modulation(tu, UpDwn);
    hrtim_set_switch_convention(tu, conv);
    hrtim_tu_init(tu);
    hrtim_phase_shift_set(tu, 0);
    hrtim_dt_set(tu, 100, 100);
}

int main(void)
{
    fake_regs_init();
    init_leg(PWMD, PWMx1);
    init_leg(PWME, PWMx2);

    uint32_t setd = TIMREG(PWMD).SETx1R, rstd = TIMREG(PWMD).RSTx1R;
    uint32_t sete = TIMREG(PWME).SETx1R, rste = TIMREG(PWME).RSTx1R;
    uint32_t cr2 = HRTIM1->sCommonRegs.CR2;
    printf("  D: SET1=0x%08x RST1=0x%08x  E: SET1=0x%08x RST1=0x%08x  CR2=0x%08x\n",
           setd, rstd, sete, rste, cr2);

    CHECK(setd == sete && rstd == rste, "same output 1 set / reset events on D and E");
    CHECK(rstd == LL_HRTIM_OUTPUTRESET_TIMCMP1 && setd == 0,
          "output 1: reset on CMP1 only (up-down: set on CMP1 when counting down)");
    CHECK((cr2 & HRTIM_CR2_SWPD) == 0, "timer D (buck): outputs not swapped");
    CHECK((cr2 & HRTIM_CR2_SWPE) != 0, "timer E (boost): outputs swapped");
    CHECK((TIMREG(PWMD).OUTxR & HRTIM_OUTR_DTEN) && (TIMREG(PWME).OUTxR & HRTIM_OUTR_DTEN),
          "dead time enabled on D and E");
    CHECK((TIMREG(PWMD).TIMxCR2 & HRTIM_TIMCR2_UDM) && (TIMREG(PWME).TIMxCR2 & HRTIM_TIMCR2_UDM),
          "D and E in up-down counting mode");
    CHECK(TIMREG(PWMD).PERxR == 27200 && TIMREG(PWME).PERxR == 27200, "D and E at 50 kHz");

    /* Re-init of E as buck (modulation switch back to unipolar) */
    init_leg(PWME, PWMx1);
    CHECK((HRTIM1->sCommonRegs.CR2 & HRTIM_CR2_SWPE) == 0, "E re-initialised as buck: swap cleared");
    CHECK(TIMREG(PWME).PERxR == 27200, "E still at 50 kHz after the re-init");

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
