#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file test_model.py
## @brief L1-INT-03/04: real C-Block against the imported switched model.
"""These are integration checks, not unblocking gated formal catalogue cases."""
from pathlib import Path
import json,os,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from runner import run,save
from runtime import HERE,CONTRACT
from app_constants import STARTUP_CURRENT_MAX

def emit(id,ok,description,value):
    """Report one criterion using the campaign's existing protocol."""
    print('\t'.join(['@@',id,'PASS' if ok else 'FAIL',description,'all observations hold',str(value)]),flush=True);return ok

def main():
    """Exercise nominal grid coupling, short startup and true sample loss."""
    out=Path(os.environ.get('CORE_TEST_OUT',HERE.parent/'out/l1-development'));ok=True
    results={}
    for name in ['idle_healthy','startup_short','vac_dropout']:
        scenario=json.loads((HERE/'scenarios'/f'{name}.json').read_text())
        result=run(scenario);save(result,out/name);results[name]=result
    t,signals,rows,manifest=results['idle_healthy']
    calls=np.array([r['invocations'] for r in rows]);times=np.array([r['time_s'] for r in rows])
    # @test L1-INT-03.a: Only real native invocations, correct cadence and no hidden bridge request.
    ok=emit('L1-INT-03.a',np.array_equal(calls,np.arange(1,len(rows)+1)) and np.max(abs(np.diff(times)-CONTRACT['control_period_s']))<1e-10 and manifest['finite_states'] and all(r['en_bridge']==r['relay_requested']==r['block_error']==0 for r in rows),'Refined switched model runs actual Core C-Block at the declared period',len(rows)) and ok
    # @test L1-INT-03.b: Nominal integration can qualify/synchronize without inverter or relay enable.
    ok=emit('L1-INT-03.b',any(r['grid_qualified'] for r in rows) and any(r['synchronized'] for r in rows) and all(r['en_bridge']==r['inverter_pwm_requested']==r['relay_requested']==0 for r in rows),'Nominal 50 Hz integration qualifies and synchronizes with bridge and relay off',next((r['time_s'] for r in rows if r['synchronized']),None)) and ok
    # @test L1-INT-03.c: Current input indexing remains correct after adding C-Block output nodes.
    finite_current=np.max(abs(signals['ILow1']-signals['IP(DC1_Iin)']))
    # Samples share storage/control ticks here; interpolation only affects floating boundaries.
    ok=emit('L1-INT-03.c',finite_current<1e-6 and np.max(abs(signals['ILow2']-signals['IP(DC2_Iin)']))<1e-6,'Physical current probe inputs retain native current offsets',finite_current) and ok
    _,_,rows,manifest=results['startup_short'];changed=[r for r in rows if r['boost_mode_changed']]
    # @test L1-INT-03.d: The scenario command reaches exactly its requested tick; Core owns scheduling.
    ok=emit('L1-INT-03.d',len(changed)==1 and abs(changed[0]['time_s']-.001)<1e-12 and changed[0]['cold_tick']==0 and changed[0]['boost_mode']==1 and abs(changed[0]['i_ref1']-STARTUP_CURRENT_MAX)<1e-6 and all(r['i_ref1']<=STARTUP_CURRENT_MAX+1e-6 and r['i_ref2']==0 for r in rows) and manifest['finite_states'],f'Tick-aligned STARTUP: cold update on the command tick, current reference clamped to {STARTUP_CURRENT_MAX:g} A on leg 1',[(r['time_s'],r['i_ref1']) for r in changed]) and ok
    _,signals,rows,manifest=results['vac_dropout'];trip=[r for r in rows if r['observer_trip']==1];fresh=[r for r in rows if r['vac_updated']==1]
    # @test L1-INT-04.a: Real VAC delivery loss holds the sample, ages to 200 and trips invalid measurement.
    ok=emit('L1-INT-04.a',bool(trip) and trip[0]['vac_age_ticks']==200 and abs(trip[0]['time_s']-fresh[-1]['time_s']-.02)<1e-10 and manifest['finite_states'],'VAC delivery dropout triggers Core stale detection after exactly 200 periods',trip[0]['time_s'] if trip else None) and ok
    # @test L1-INT-04.b: The electrical grid remains healthy during sample loss.
    mask=signals['vac_updated']==0
    ok=emit('L1-INT-04.b',np.ptp(signals['VP(Grid_voltage)'][mask])>600 and all(r['block_error']==0 for r in rows),'Sample-loss fixture does not switch off the physical grid',float(np.ptp(signals['VP(Grid_voltage)'][mask]))) and ok
    raise SystemExit(0 if ok else 1)
if __name__=='__main__':main()
