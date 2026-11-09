/* Copyright (c) 2026 OWNTECH TECHNOLOGIES
 * SPDX-License-Identifier: LGPL-2.1
 */
/** @file wrapper.cpp
 * @brief Thin Pulsim C-Block ABI around the unchanged Core application control step.
 * @details L1-INT-01 checks native invocation, timing, independent state and reset.
 * No application sequencing, controller tuning or PWM hardware model lives here.
 */
#include "app/control_step.h"
#include "contract.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>

/** @brief Independent state owned by one C-Block instance. */
struct Context {
    ControlState control{}; ///< Same state type as main.cpp and the L0 tests.
    uint64_t calls=0;       ///< Number of actual controlStep invocations.
    double previous_time=0; ///< Last native invocation time (s).
    float held_vac=0;       ///< Last delivered VAC, matching sensor last-value hold.
    bool failed=false;     ///< Invalid ABI/timing makes this block inert.
};
/** @brief Mark a contract error without calling application code.
 * @param out Caller-owned output buffer. @param size Buffer length.
 */
static void error(double* out,int size) {
    if (!out || size<0) return;
    std::fill(out,out+size,0.0);
    if (size>BLOCK_ERROR) out[BLOCK_ERROR]=1.0;
}
extern "C" {
/** @brief Contract version for loader preflight. @return ABI version. */
unsigned core_contract_version() {return 2;}
/** @brief Compiled input count. @return Required input-buffer length. */
unsigned core_input_count() {return INPUT_COUNT;}
/** @brief Compiled output count. @return Required output-buffer length. */
unsigned core_output_count() {return OUTPUT_COUNT;}
/** @brief Compiled application period. @return Control period in seconds. */
double core_control_period() {return double(app::control_task_period)*1e-6;}
/** @brief Allocate an independent firmware block and run controlInit once.
 * @param n_in Input count. @param n_out Output count.
 * @return Context pointer, or null on invalid contract/init failure.
 */
void* pulsim_cblock_init(int n_in,int n_out) {
    if (n_in!=INPUT_COUNT || n_out!=OUTPUT_COUNT) return nullptr;
    auto* c=new(std::nothrow) Context;
    if (c && controlInit(c->control)!=0) {delete c;return nullptr;}
    return c;
}
/** @brief Destroy this block's state. @param state Context allocated by init. */
void pulsim_cblock_term(void* state) {delete static_cast<Context*>(state);}
/** @brief Execute exactly one unmodified application control step.
 * @param in Input buffer matching Input. @param n_in Input length.
 * @param out Output buffer matching Output. @param n_out Output length.
 * @param t Current simulated time (s). @param dt Block sample period (s).
 * @param state Address of this block's context pointer.
 * @note Samples/commands are scenario inputs. Mode changes and cold scheduling
 * are owned by controlStep. A contract/timing error disables this context.
 */
void pulsim_cblock_step(const double* in,int n_in,double* out,int n_out,
                        double t,double dt,void** state) {
    auto* c=state ? static_cast<Context*>(*state):nullptr;
    const double period=core_control_period();
    if (!c || !in || !out || n_in!=INPUT_COUNT || n_out!=OUTPUT_COUNT) {error(out,n_out);return;}
    if (c->failed || !std::isfinite(t) || std::abs(dt-period)>1e-12 ||
        (c->calls && std::abs(t-c->previous_time-period)>1e-10) ||
        !std::isfinite(in[MODE]) || in[MODE]<IDLE || in[MODE]>MPPT ||
        std::floor(in[MODE])!=in[MODE] ||
        (in[VAC_UPDATED]!=0 && in[VAC_UPDATED]!=1) ||
        !std::isfinite(in[SYNC_REQUEST]) || in[SYNC_REQUEST]<INV_OFF || in[SYNC_REQUEST]>INV_INJECT ||
        std::floor(in[SYNC_REQUEST])!=in[SYNC_REQUEST]) {
        c->failed=true;error(out,n_out);return;
    }
    Measurements m;
    m.v_low=float(in[VLOW]);m.i_low_1=float(in[ILOW1]);m.i_low_2=float(in[ILOW2]);
    m.v_dc_bus=float(in[VBUS]);m.i_ac=float(in[IAC]);m.v_ac_updated=in[VAC_UPDATED]!=0;
    if (m.v_ac_updated) c->held_vac=float(in[VAC]);
    m.v_ac=c->held_vac;
    OperatorCommands cmd;
    cmd.mode=static_cast<boost_mode>(int(in[MODE]));cmd.inverter_stage=static_cast<InverterStage>(int(in[SYNC_REQUEST]));cmd.v_bus_ref=float(in[VBUS_REF]);
    const ControlOutputs result=controlStep(c->control,m,cmd);
    ++c->calls;c->previous_time=t;
    auto& ctrl=c->control;const auto ab=ctrl.inverter.getVab();const auto dq=ctrl.inverter.getVdq();
    const double values[OUTPUT_COUNT]={result.boost_duty.leg1,result.boost_duty.leg2,
        double(result.leg1_enable),double(result.leg2_enable),double(result.boost_mode_changed),double(result.boost_pwm_update),
        result.bridge_duty,double(result.bridge_enable),result.grid_angle,ctrl.inverter.getw(),
        ctrl.grid_observer.getVoltageRms(),ctrl.grid_observer.getFrequencyHz(),double(ctrl.grid_observer.getTripReason()),
        double(ctrl.inverter.getMode()),double(ctrl.inverter.getSync()),double(ctrl.grid_observer.canOperate()),
        double(result.burst_skip),double(result.burst_period),ctrl.boost.getILow1Ref(),ctrl.boost.getILow2Ref(),
        ab.alpha,ab.beta,dq.d,dq.q,double(ctrl.grid_voltage_age_ticks),double(ctrl.cold_loop_tick),
        double(ctrl.boost.getMode()),double(c->calls),0.,double(ctrl.inverter.isPwmRequested()),
        double(ctrl.inverter.isGridConnectionRequested()),double(ctrl.inverter.getFaultReason()),double(result.boost_trip)};
    std::copy(values,values+OUTPUT_COUNT,out);
}
}
