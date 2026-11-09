#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file runtime.py
## @brief Attach the real native C-Block and hold hardware commands between its ticks.
"""Application sequencing remains exclusively in Core/src/app/control_step.cpp."""
from pathlib import Path
import ctypes,json,os
import numpy as np
import pulsim as p
HERE=Path(__file__).resolve().parent
CONTRACT=json.loads((HERE/'contract.json').read_text())
LIB=Path(os.environ.get('L1_CBLOCK_LIB',HERE/'firmware_block/build/libcore_control.so'))  # study variants: another build
NAMES=CONTRACT['outputs'];INDEX={n:i for i,n in enumerate(NAMES)}

def verify_library():
    """Fail early on a missing/wrong native ABI rather than running demo code."""
    if p.__version__!="2.0.0":raise RuntimeError("L1 requires pinned pulsim==2.0.0")
    dll=ctypes.CDLL(str(LIB))
    for name,expected in [('core_contract_version',3),('core_input_count',len(CONTRACT['inputs'])),('core_output_count',len(NAMES))]:
        getattr(dll,name).restype=ctypes.c_uint
        if getattr(dll,name)()!=expected:raise ValueError('Wrong C-Block contract: '+name)
    dll.core_control_period.restype=ctypes.c_double
    if abs(dll.core_control_period()-CONTRACT['control_period_s'])>1e-12:raise ValueError('Control period mismatch')
    return dll

def attach(builder,wires,dt):
    """Load the native C-Block via Pulsim's own API; return its live handle."""
    verify_library()
    # Pulsim 2.0 resolves current-input state offsets before adding output sources.
    # Predeclare output nodes so the node count cannot shift those offsets.
    for name in NAMES:builder.node('core_'+name)
    block=p.add_c_block(builder,inputs=wires,outputs=[('v','core_'+n,'0') for n in NAMES],
                         dt=CONTRACT['control_period_s'],lib=str(LIB),name='Core',sim_dt=dt)
    raw_extra=block.b_extra_fn
    rows=[builder.pool.branch_var_id_for_source(builder.branch_id_of(f'Core_out{i}'),builder.graph) for i in range(len(NAMES))]
    cache={'fire':None,'extra':None}
    def diagnostic_extra(t):
        # Observer intentionally emits NaN telemetry on invalid/stale samples.
        # Preserve those raw block outputs in logs, but do not inject a NaN
        # diagnostic-only voltage into the electrical matrix. Actuator NaNs fail.
        # The stamp depends only on the block outputs, which change once per control
        # tick (n_fires), not per electrical step: rebuild it only when they change.
        # Pulsim copies the returned list before summing, so sharing it is safe.
        if cache['fire']==block.n_fires:return cache['extra']
        if not np.isfinite(block.outputs[:8]).all():raise RuntimeError('Non-finite actuator command')
        extra=raw_extra(t)
        for i in range(8,len(NAMES)):
            if not np.isfinite(block.outputs[i]):extra[rows[i]]=0.
        cache['fire']=block.n_fires;cache['extra']=extra
        return extra
    block.b_extra_fn=diagnostic_extra
    return block

class Actuators:
    """Apply target-style mode-change/update precedence to latched boost outputs, model the HRTIM
    burst on leg 1 (ADR 0003), and (contract app-v3) the inverter bridge and the grid relay as the
    application commands them: unipolar 50 kHz bridge with the driver's duty limits and the duty
    taking effect after the update delay; relay contact closing/opening after its mechanical times."""
    def __init__(self,bridge=None):
        cfg=CONTRACT['actuators'];self.br={**cfg['bridge'],**(bridge or {})};self.rl=cfg['relay']
        self.bridge_on=False;self.bridge_duty=0.5;self.bridge_pending=[]  # [(t_effective, on, duty)]
        self.relay_cmd=False;self.relay_cmd_t=-1.;self.contact=False;self.t=0.
        self.enabled=[False,False];self.duties=[0.,0.]
        self.burst_cmd=(0,0)      # (skip, period) last commanded
        self.burst_now=(0,0)      # (skip, period) in force for the current burst period
        self.burst_cycle=None     # index of the burst period in force
    def apply(self,out,t=0.):
        """Apply one returned ControlOutputs vector at control time t; no update leaves state held."""
        q=lambda n:out[INDEX[n]];self.t=t
        if q('block_error'):raise RuntimeError('Native C-Block contract/timing failure')
        if q('boost_mode_changed'):self.enabled=[False,False]
        if q('boost_pwm_update'):
            cfg=CONTRACT['actuators'];self.enabled=[bool(q('en_leg1')),bool(q('en_leg2'))]
            self.duties=[np.floor(np.clip(q('duty_leg'+str(i+1)),cfg['duty_min'],cfg['duty_max'])*cfg['timer_counts'])/cfg['timer_counts'] for i in range(2)]
            skip=int(q('burst_skip')) if self.enabled[0] else 0
            self.burst_cmd=(skip,int(q('burst_period')))
        else:self.burst_cmd=(0,0)
        # Bridge (main.cpp: setInverterDutyCycle then outputs on; off stops both legs at once).
        on=bool(q('en_bridge'));d=float(np.clip(q('duty_bridge'),self.br['duty_min'],self.br['duty_max']))
        self.bridge_pending.append((t+(self.br['update_delay_s'] if on else 0.),on,d))
        # Relay command (main.cpp drives GPIO 22 from out.relay_close); the contact follows mechanically.
        cmd=bool(q('relay_close'))
        if cmd!=self.relay_cmd:self.relay_cmd=cmd;self.relay_cmd_t=t
    def burst_off(self,t,period):
        """Whether leg 1 sits in an idle (skipped) PWM period at time t.
        The skip count is preloaded: a new command takes effect at the next burst period."""
        skip,per=self.burst_cmd
        if skip<=0 and self.burst_now[0]<=0:
            self.burst_now=(0,0);self.burst_cycle=None;return False
        n=int(np.floor((t+1e-14)/period))  # PWM period index
        per_now=self.burst_now[1] or per
        cycle=n//per_now if per_now else 0
        if self.burst_cycle is None or cycle!=self.burst_cycle:
            # start of a burst period (or burst just started): load the latest command
            self.burst_now=(skip,per) if per else (0,0);self.burst_cycle=n//per if per else None
            if not per:return False
        skip_now,per_now=self.burst_now
        return per_now>0 and (n%per_now)<skip_now
    def gates(self,t):
        """Return physical gates, with complementary center-aligned boost PWM. The carrier is
        shifted so the control ticks (multiples of the control period, where the firmware
        block samples) fall at the board's ADC instant: adc_trigger_phase of the period
        after the start of the carrier, the low switch being on around half a period."""
        cfg=CONTRACT['actuators'];period=1/cfg['dc_pwm_hz'];tc=t+cfg.get('adc_trigger_phase',0.)*period
        phase=(tc+1e-14)%period;g={}
        off1=self.burst_off(tc,period) if self.enabled[0] else False
        for i in range(2):
            d=self.duties[i];start=(1-d)*period/2;end=(1+d)*period/2
            on=self.enabled[i] and not (i==0 and off1)
            g[f'DC{i+1}_QL']=on and start+cfg['deadtime_s']<=phase<end
            g[f'DC{i+1}_QH']=on and (phase<start or phase>=end+cfg['deadtime_s'])
        while self.bridge_pending and self.bridge_pending[0][0]<=t+1e-12:
            _,self.bridge_on,self.bridge_duty=self.bridge_pending.pop(0)
        if self.bridge_on:
            T=1/self.br['pwm_hz'];ph=(t+1e-14)%T;td=self.br['deadtime_s']
            def win(dd):
                start=(1-dd)*T/2;end=(1+dd)*T/2
                return (start+td<=ph<end),(ph<start or ph>=end+td)
            g['INV_AH'],g['INV_AL']=win(self.bridge_duty);g['INV_BH'],g['INV_BL']=win(1-self.bridge_duty)
        else:
            for n in ['INV_AH','INV_AL','INV_BH','INV_BL']:g[n]=False
        if self.relay_cmd_t>=0:
            delay=self.rl['close_s'] if self.relay_cmd else self.rl['open_s']
            if t+1e-12>=self.relay_cmd_t+delay:self.contact=self.relay_cmd
        g['Relay_L']=g['Relay_N']=self.contact
        return g
