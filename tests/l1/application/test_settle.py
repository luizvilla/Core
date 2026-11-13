#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file test_settle.py
## @brief L1-APP-06: the application's step 1 (stage SETTLE) on the switched model, bridge switching at 50 ns.
##
## Runs scenarios/settle_step1.json (about one hour) only when L1_LONG=1; otherwise reports L1-APP-06 BLOCKED.
## Criteria (procedure §6, tab Application):
##   L1-APP-06.a  OFF → VOLTAGE_SYNC → PWM_WAIT → PWM_SETTLE, no inverter fault, no bench trip
##   L1-APP-06.b  bridge PWM no earlier than 100 ms after grid qualification (the synchronization dwell)
##   L1-APP-06.c  relay never commanded
##   L1-APP-06.d  INFO: bridge duty span and inverter inductor current RMS over the last 100 ms of PWM_SETTLE
## @test L1-APP-06
"""No averaged model, no injected sensor values: the unchanged controlStep in the C-Block."""
from pathlib import Path
import json,os,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from runner import run,save
from runtime import HERE

VSI_VOLTAGE_SYNC,VSI_FAULT,VSI_PWM_SETTLE,VSI_PWM_WAIT=1,7,8,9  # inverterTypes.h
SYNC_DWELL_S=0.1  # single_phase_inverter SYNC_MIN_TIME


def emit(id,verdict,description,limit,value):
    print('\t'.join(['@@',id,verdict,description,str(limit),str(value)]),flush=True)
    return verdict!='FAIL'


def first(rows,pred):
    """@brief Time of the first row where pred holds, or None."""
    return next((r['time_s'] for r in rows if pred(r)),None)


def main():
    if os.environ.get('L1_LONG')!='1':
        emit('L1-APP-06','BLOCKED','Step 1 (SETTLE) on the switched model','run on request',
             'about one hour at 50 ns (scenarios/settle_step1.json): run with L1_LONG=1')
        return 0
    out=Path(os.environ.get('CORE_TEST_OUT',HERE.parent/'out/l1-development'))
    scenario=json.loads((HERE/'scenarios/settle_step1.json').read_text())
    result=run(scenario);save(result,out/'settle_step1')
    t,signals,rows,manifest=result
    ok=True
    modes=[int(r['inverter_mode']) for r in rows]
    t_sync=first(rows,lambda r:r['inverter_mode']==VSI_VOLTAGE_SYNC)
    t_wait=first(rows,lambda r:r['inverter_mode']==VSI_PWM_WAIT)
    t_settle=first(rows,lambda r:r['inverter_mode']==VSI_PWM_SETTLE)
    in_order=None not in (t_sync,t_wait,t_settle) and t_sync<t_wait<t_settle
    clean=VSI_FAULT not in modes and all(r['bench_trip']==0 and r['inverter_fault']==0 for r in rows)
    # @test L1-APP-06.a: the real step-1 sequence, without fault or bench trip.
    ok=emit('L1-APP-06.a','PASS' if in_order and clean and manifest['finite_states'] else 'FAIL',
            'Step 1: OFF → VOLTAGE_SYNC → PWM_WAIT → PWM_SETTLE, no fault, no bench trip','sequence in order, no fault',
            {'VOLTAGE_SYNC':t_sync,'PWM_WAIT':t_wait,'PWM_SETTLE':t_settle,'fault':not clean}) and ok
    t_qual=first(rows,lambda r:r['grid_qualified'])
    t_bridge=first(rows,lambda r:r['en_bridge'])
    # VOLTAGE_SYNC is entered as soon as step 1 is commanded; the dwell only counts once the grid is qualified.
    gated=None not in (t_qual,t_bridge) and t_bridge-t_qual>=SYNC_DWELL_S-1e-9
    # @test L1-APP-06.b: no bridge before qualification and the synchronization dwell.
    ok=emit('L1-APP-06.b','PASS' if gated else 'FAIL','Bridge PWM only after qualification and 100 ms of synchronization',
            '≥ 100 ms after qualification',{'qualified':t_qual,'bridge':t_bridge,'dwell_s':None if None in (t_qual,t_bridge) else t_bridge-t_qual}) and ok
    relay=max(max(r['relay_requested'],r['relay_close']) for r in rows)
    # @test L1-APP-06.c: step 1 never commands the relay.
    ok=emit('L1-APP-06.c','PASS' if relay==0 else 'FAIL','Relay never commanded in step 1','always open',relay) and ok
    if t_settle is not None:
        end=t[-1];w=(t>=max(t_settle,end-0.1))
        duty=signals['duty_bridge'][w];il=signals['I(AC_L1)'][w]
        emit('L1-APP-06.d','INFO','PWM_SETTLE, last 100 ms: bridge duty span and inverter inductor current RMS','informational',
             {'duty_min':float(duty.min()),'duty_max':float(duty.max()),'I(AC_L1)_rms_A':float(np.sqrt(np.mean(il**2)))})
    return 0 if ok else 1


if __name__=='__main__':sys.exit(main())
