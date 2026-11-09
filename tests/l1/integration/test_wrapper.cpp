/* Copyright (c) 2026 OWNTECH TECHNOLOGIES
 * SPDX-License-Identifier: LGPL-2.1
 */
/** @file test_wrapper.cpp
 * @brief L1-INT-01: compare the ABI wrapper against direct unchanged controlStep calls.
 * @details Exercises output parity, per-instance state, sample delivery and ABI errors.
 */
#include "app/control_step.h"
#include "contract.h"
#include "check.h"
#include <cmath>
#include <algorithm>
extern "C" {
void* pulsim_cblock_init(int,int);
void pulsim_cblock_step(const double*,int,double*,int,double,double,void**);
void pulsim_cblock_term(void*);
}
/** @brief Drive identical synthetic inputs through direct and ABI entry points.
 * @return tc::finish exit status.
 */
int main() {
    constexpr double dt=0.0001;
    ControlState direct{};const int init=controlInit(direct);
    void* state=pulsim_cblock_init(INPUT_COUNT,OUTPUT_COUNT);
    bool parity=init==0 && state;double max_error=0;float held=0;bool held_verified=false;
    bool staged=false,bridge_on=false,relay_cmd=false,relay_closed=false;uint16_t hold=0;  // main.cpp hardware loop (app-v3)
    for (int k=0;k<13000 && parity;++k) {
        double in[INPUT_COUNT]={40,0,0,390,230*std::sqrt(2.)*std::sin(2*3.141592653589793*50*k*dt),0,1,0,1,400};
        in[MODE]=k>=10 && k<80 ? STARTUP : k>=80 && k<100 ? BUSREG : IDLE;
        in[VAC_UPDATED]=(k>=12000 && k<12200) ? 0:1;
        double out[OUTPUT_COUNT]{};pulsim_cblock_step(in,INPUT_COUNT,out,OUTPUT_COUNT,k*dt,dt,&state);
        Measurements m;
        if (in[VAC_UPDATED]) held=float(in[VAC]);
        else held_verified=true;
        m.v_low=float(in[VLOW]);m.i_low_1=float(in[ILOW1]);m.i_low_2=float(in[ILOW2]);
        m.v_dc_bus=float(in[VBUS]);m.v_ac=held;m.i_ac=float(in[IAC]);m.v_ac_updated=in[VAC_UPDATED]!=0;m.relay_closed=relay_closed;
        OperatorCommands cmd;cmd.mode=static_cast<boost_mode>(int(in[MODE]));cmd.inverter_stage=INV_SETTLE;cmd.v_bus_ref=400;
        const auto expected=controlStep(direct,m,cmd);
        if (expected.bridge_enable) {if (staged) {bridge_on=true;direct.bridge_pwm_applied=true;} else {bridge_on=false;direct.bridge_pwm_applied=false;staged=true;}}
        else {bridge_on=false;direct.bridge_pwm_applied=false;staged=false;}
        direct.inverter.setPwmEnabled(direct.bridge_pwm_applied && !direct.open_loop_active);
        if (expected.relay_close!=relay_cmd) {relay_cmd=expected.relay_close;hold=0;}
        else if (hold<app::relay_contact_ticks) ++hold;
        else relay_closed=relay_cmd;
        const double values[8]={expected.boost_duty.leg1,expected.boost_duty.leg2,double(expected.leg1_enable),double(expected.leg2_enable),double(expected.boost_mode_changed),double(expected.boost_pwm_update),expected.bridge_duty,double(bridge_on)};
        for (int j=0;j<8;++j)max_error=std::max(max_error,std::abs(values[j]-out[j]));
        parity=out[BLOCK_ERROR]==0 && out[INVOCATIONS]==k+1 && out[VAC_AGE]==direct.grid_voltage_age_ticks &&
            out[COLD_TICK]==direct.cold_loop_tick && out[SYNCHRONIZED]==double(direct.inverter.getSync()) && max_error==0;
    }
    /** @test L1-INT-01.a: Native wrapper matches direct application calls exactly. */
    {tc::Criterion c("L1-INT-01.a","Native ABI matches direct controlStep, including mode changes and held VAC");c.check("same outputs/state",parity&&held_verified);c.near("maximum output error",max_error,0,0);}
    pulsim_cblock_term(state);
    void* first=pulsim_cblock_init(INPUT_COUNT,OUTPUT_COUNT);void* second=pulsim_cblock_init(INPUT_COUNT,OUTPUT_COUNT);
    double in[INPUT_COUNT]={40,0,0,0,0,0,1,0,0,400},a[OUTPUT_COUNT]{},b[OUTPUT_COUNT]{};
    pulsim_cblock_step(in,INPUT_COUNT,a,OUTPUT_COUNT,0,dt,&first);
    pulsim_cblock_step(in,INPUT_COUNT,a,OUTPUT_COUNT,dt,dt,&first);
    pulsim_cblock_step(in,INPUT_COUNT,b,OUTPUT_COUNT,0,dt,&second);
    /** @test L1-INT-01.b: Each C-Block owns independent state. */
    {tc::Criterion c("L1-INT-01.b","Independent C-Block contexts start fresh");c.check("independent counters",a[INVOCATIONS]==2&&b[INVOCATIONS]==1);}
    pulsim_cblock_step(in,INPUT_COUNT,a,OUTPUT_COUNT,3*dt,dt,&first);
    /** @test L1-INT-01.c: Missing/wrong timing or ABI cannot execute silent demo code. */
    {tc::Criterion c("L1-INT-01.c","Invalid ABI and skipped ticks report block_error");c.check("timing rejected",a[BLOCK_ERROR]==1);c.check("wrong input count rejected",pulsim_cblock_init(1,OUTPUT_COUNT)==nullptr);}
    pulsim_cblock_term(first);pulsim_cblock_term(second);
    return tc::finish();
}
