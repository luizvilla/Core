/* Commit 5: synchronous counter reset */
#include "fake_regs.h"

int main(void)
{
    fake_regs_init();
    hrtim_init_default_all();
    hrtim_tu_number_t tus[2] = {PWMD, PWME};
    HRTIM1->sCommonRegs.CR2 = 0;
    hrtim_cnt_reset_sync(tus, 2);
    uint32_t cr2 = HRTIM1->sCommonRegs.CR2;
    printf("  CR2=0x%08x\n", cr2);
    CHECK(cr2 == (HRTIM_CR2_TDRST | HRTIM_CR2_TERST), "CR2 = TDRST | TERST only");
    HRTIM1->sCommonRegs.CR2 = 0;
    hrtim_cnt_reset_sync(tus, 0);
    CHECK(HRTIM1->sCommonRegs.CR2 == 0, "count 0: no write");
    printf("%d failure(s)\n", failures);
    return failures != 0;
}
