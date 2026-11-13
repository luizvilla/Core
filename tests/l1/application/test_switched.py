#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file test_switched.py
## @brief Formal switched application criteria with explicit partial-run limits.
"""No averaged model, controller retuning, injected sensor values or hidden load."""
from pathlib import Path
import csv,json,os,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from runner import run,save,pll_aligned
from runtime import HERE
from app_constants import STARTUP_CURRENT_MAX,BOOST_TRIP_CURRENT,BURST_PERIOD,CONTROL_PERIOD,GRID_VOLTAGE_STALE_TICKS


def emit(id,verdict,description,limit,value):
    print('\t'.join(['@@',id,verdict,description,str(limit),str(value)]),flush=True)
    return verdict!='FAIL'


def load(folder):
    """Read controller data and stored electrical samples with their manifest."""
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader((folder/'control.csv').open())]
    with np.load(folder/'waveforms.npz',allow_pickle=False) as raw:data={k:raw[k] for k in raw.files}
    return rows,data,json.loads((folder/'manifest.json').read_text())


def main():
    """Run declared cases and report evidence, including unmet long-run criteria."""
    out=Path(os.environ.get('CORE_TEST_OUT',HERE.parent/'out/l1-switched-development'));out.mkdir(parents=True,exist_ok=True)
    ok=True
    # These are the same raw scenarios already run by test_model.py, with formal
    # criteria evaluated here instead of rerunning nominal grid/source dropout.
    rows,data,meta=load(out/'idle_healthy')
    # @test L1-APP-01.a: Nominal idle leaves both boost gates off.
    ok=emit('L1-APP-01.a','PASS' if all(r['en_leg1']==r['en_leg2']==0 for r in rows) else 'FAIL',
            'IDLE on a healthy switched grid keeps boost legs disabled','both off',len(rows)) and ok
    q=next((i for i,r in enumerate(rows) if r['grid_qualified']),None);dwell=None
    if q is not None:
        start=q
        while start>0 and 207<=rows[start-1]['observer_rms']<=253 and 47.5<=rows[start-1]['observer_hz']<=52.5:start-=1
        dwell=rows[q]['time_s']-rows[start]['time_s']+CONTROL_PERIOD
    # @test L1-APP-01.b: Qualification is timed from continuous eligible grid.
    ok=emit('L1-APP-01.b','PASS' if dwell is not None and abs(dwell-1)<=CONTROL_PERIOD*1.00001 else 'FAIL',
            'Observer qualifies after continuous healthy estimated grid',f'1 s +/- {CONTROL_PERIOD*1e6:g} us',dwell) and ok
    emit('L1-APP-01.c','INFO','Time from electrical start to PLL alignment (one-period averages, inverter OFF)','informational',
         pll_aligned(rows)[0])
    rows,data,meta=load(out/'vac_dropout');trip=next((r for r in rows if r['observer_trip']==1),None)
    fresh=[r for r in rows if r['vac_updated']]
    # @test L1-APP-08.a: Real delivery loss reaches stale threshold exactly.
    valid=trip is not None and trip['vac_age_ticks']==GRID_VOLTAGE_STALE_TICKS and abs(trip['time_s']-fresh[-1]['time_s']-.02)<1e-10
    ok=emit('L1-APP-08.a','PASS' if valid else 'FAIL','Lost VAC samples trigger InvalidMeasurement',f'{GRID_VOLTAGE_STALE_TICKS} control periods (20 ms)',trip['time_s'] if trip else None) and ok
    for case in ['dcdc_startup','dcdc_busreg','dcdc_burst']:
        scenario=json.loads((HERE/'scenarios'/f'{case}.json').read_text())
        print('Formal switched scenario:',case,flush=True)
        result=run(scenario);save(result,out/case);rows,data,meta=load(out/case)
        if not meta['finite_states'] or meta['initial_capacitor_voltage_error_V']>1e-8 or meta['initial_transformer_voltage_mismatch_V']>1e-8 or meta['electrical_observations']<round(scenario['duration_s']/scenario['dt_s'])+1:
            ok=emit('L1-INT-06.'+case,'FAIL','Fixture initialization/finite-state check','consistent capacitor and transformer voltages',meta) and ok
            continue
        if case=='dcdc_startup':
            worst=max(r['i_ref1']+r['i_ref2'] for r in rows)
            # @test L1-APP-02.a: STARTUP clamps the total current reference to app::startup_current_max (ADR 0003).
            ok=emit('L1-APP-02.a','PASS' if worst<=STARTUP_CURRENT_MAX+1e-6 else 'FAIL','STARTUP total current reference over first 50 ms',f'<= {STARTUP_CURRENT_MAX:g} A',worst) and ok
            peak=meta['electrical_step_extrema']['ILow1']['max']
            # @test L1-APP-02.b: Check physical current at every electrical step against the trip level.
            ok=emit('L1-APP-02.b','PASS' if peak<BOOST_TRIP_CURRENT else 'FAIL','STARTUP physical leg-1 current over first 50 ms',f'< {BOOST_TRIP_CURRENT:g} A (trip) at every 50 ns step',peak) and ok
            # @test L1-APP-02.c: Startup never enables converter 2.
            ok=emit('L1-APP-02.c','PASS' if all(r['en_leg2']==0 for r in rows) else 'FAIL','STARTUP keeps converter 2 disabled','always off',max(r['en_leg2'] for r in rows)) and ok
            emit('L1-APP-02.d','BLOCKED','Full zero-to-400 V startup trajectory','complete switched run of about 1 s at the 0.25 A clamp (dcdc_startup_full.json)','Only first 50 ms executed; no full start-up claim')
        elif case=='dcdc_busreg':
            a=scenario['acceptance'];window=(data['time']>=a['settled_window_s'][0]-1e-12)&(data['time']<=a['settled_window_s'][1]+1e-12)
            low=float(data['VDCBus'][window].min());high=float(data['VDCBus'][window].max())
            # @test L1-APP-03.a: Window and load step are fixed before running.
            ok=emit('L1-APP-03.a','PASS' if low>=a['bus_min_V'] and high<=a['bus_max_V'] else 'FAIL',
                    'BUSREG after 50 ms load step, assessed over 80-100 ms','395-405 V',(low,high)) and ok
            emit('L1-APP-03.b','INFO','Maximum bus overshoot above 400 V','informational',float(data['VDCBus'].max()-400))
            emit('L1-APP-03.c','INFO','Physical current extrema including initial conditions','informational',meta['electrical_step_extrema'])
        else:
            # Current-driven burst on leg 1 (ADR 0003): the light-load fixture starts fully skipped.
            # @test L1-APP-04.a: Entering BUSREG from IDLE starts with every PWM period skipped.
            ok=emit('L1-APP-04.a','PASS' if rows[0]['burst_skip']==BURST_PERIOD==rows[0]['burst_period'] else 'FAIL',
                    'Light load: BUSREG entered from IDLE starts fully skipped',f'skip = period = {BURST_PERIOD:g}',(rows[0]['burst_skip'],rows[0]['burst_period'])) and ok
            # @test L1-APP-04.b: Leg 2 never runs at light load, so burst stays allowed.
            ok=emit('L1-APP-04.b','PASS' if all(r['en_leg2']==0 for r in rows) else 'FAIL','Light load keeps leg 2 off','always off',max(r['en_leg2'] for r in rows)) and ok
            # @test L1-APP-04.c: The bus stays within +/-5 % of the reference over the run.
            lo,hi=float(data['VDCBus'].min()),float(data['VDCBus'].max())
            ok=emit('L1-APP-04.c','PASS' if lo>=380 and hi<=420 else 'FAIL','Light load: bus over the whole run','380-420 V',(lo,hi)) and ok
            # @test L1-APP-04.d: Once current is needed the skip count leaves the full value: burst follows the demand.
            left=[r for r in rows if r['burst_skip']<BURST_PERIOD]
            ok=emit('L1-APP-04.d','PASS' if left else 'FAIL','Burst follows the current demand (skip count leaves full skipping)','some tick with skip < period',
                    (left[0]['time_s'],left[0]['VDCBus']) if left else 'never') and ok
            tail=[r for r in rows if r['time_s']>=rows[-1]['time_s']-0.05]
            emit('L1-APP-04.e','INFO','Last 50 ms: skip count range, fraction of ticks bursting, bus range','informational',
                 {'skip':(min(r['burst_skip'] for r in tail),max(r['burst_skip'] for r in tail)),'bursting':sum(r['burst_skip']>0 for r in tail)/len(tail),
                  'bus':(min(r['VDCBus'] for r in tail),max(r['VDCBus'] for r in tail))})
    raise SystemExit(0 if ok else 1)


if __name__=='__main__':main()
