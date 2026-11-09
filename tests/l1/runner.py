#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file runner.py
## @brief Drive scenario commands and record a native Core C-Block against the switched plant.
"""Integration bring-up only; does not change the formal L1 catalogue prerequisites."""
from pathlib import Path
import argparse,csv,hashlib,json,time
import numpy as np
import pulsim as p
from plant.model import Model,SENSORS
from runtime import HERE,LIB,CONTRACT,NAMES,INDEX,attach,Actuators

def run(scenario,project=None,progress=None,cancelled=None):
    """Run one isolated model/block instance, returning waveforms and a manifest."""
    dt=scenario['dt_s'];duration=scenario['duration_s']
    if project is None and scenario.get('plant_variant')=='pv':
        from pulsimgui.models.project import Project
        from plant.pv import apply
        project=apply(Project.load(HERE/'plant/full_model.pulsim'),model=scenario.get('pv_model','diode'),
                      initial_bus_V=scenario.get('pv_initial_bus_V'))
    if project is None and scenario.get('fixture'):
        from pulsimgui.models.project import Project
        from plant.fixtures import apply
        project=apply(Project.load(HERE/'plant/full_model.pulsim'),scenario['fixture'])
    grid=scenario.get('grid')  # [{"t_s", "v_rms", "f_hz"}, ...]: grid steps, phase-continuous
    if grid:
        from pulsimgui.models.project import Project
        if project is None:
            meta=json.loads((HERE/'plant/params.yaml').read_text())
            if hashlib.sha256((HERE/'plant/full_model.pulsim').read_bytes()).hexdigest()!=meta['model_sha256']:
                raise ValueError('Accepted model hash no longer matches params.yaml')
            project=Project.load(HERE/'plant/full_model.pulsim')
        vg=next(a for a in project.get_active_circuit().components.values() if a.name=='Vgrid')
        vg.parameters['grid_profile']=grid;vg.parameters['waveform']={'type':'dc','value':0.0}
        grid=sorted(grid,key=lambda g:g['t_s'])
        if grid[0]['t_s']!=0:raise ValueError('A grid profile starts at t = 0')
        phase=[0.]
        for g0,g1 in zip(grid,grid[1:]):phase.append(phase[-1]+2*np.pi*g0['f_hz']*(g1['t_s']-g0['t_s']))
        def grid_v(t):
            k=max(i for i,g in enumerate(grid) if g['t_s']<=t+1e-12)
            return np.sqrt(2)*grid[k]['v_rms']*np.sin(phase[k]+2*np.pi*grid[k]['f_hz']*(t-grid[k]['t_s']))
    options=scenario.get('plant_options')  # inverter scenarios: output_chokes, grid_series_ohm/_h, ideal_bus_V
    if options and project is None:
        from pulsimgui.models.project import Project
        meta=json.loads((HERE/'plant/params.yaml').read_text())
        if hashlib.sha256((HERE/'plant/full_model.pulsim').read_bytes()).hexdigest()!=meta['model_sha256']:
            raise ValueError('Accepted model hash no longer matches params.yaml')
        project=Project.load(HERE/'plant/full_model.pulsim')
    if options and options.get('ideal_bus_V'):
        cb=next(a for a in project.get_active_circuit().components.values() if a.name=='Cbus')
        cb.parameters['initial_voltage']=options['ideal_bus_V']
    model=Model(project,options);b=model.builder
    if options and options.get('ideal_bus_V'):
        # Ideal bus: the boost is not part of the inverter tests (as plant/open_loop.py).
        b.add_resistor('MOD_bus_r','MOD_bus','Vout',1e-3);b.add_voltage_source('MOD_bus_source','MOD_bus','0',options['ideal_bus_V'])
    commands=sorted(scenario['commands'],key=lambda x:x['time_s'])
    if not commands or commands[0]['time_s']!=0:raise ValueError('Scenario must specify commands at t=0')
    if dt>1e-6 or dt<=0:raise ValueError('Integration step must be positive and no greater than 1 us')
    if any(q['mode']!=0 for q in commands) and dt>50e-9:raise ValueError('Switched boost runs require dt <= 50 ns')
    if any(q.get('sync_request',0)>=1 for q in commands) and dt>50e-9 and scenario.get('bridge_dt_ok')!=True:
        raise ValueError('Runs with the inverter bridge (stages SETTLE and above) require dt <= 50 ns')
    period=CONTRACT['control_period_s']
    for q in commands:
        if abs(q['time_s']/period-round(q['time_s']/period))>1e-7:raise ValueError('Commands must land on control ticks')
    def command(t):
        return next(q for q in reversed(commands) if q['time_s']<=t+1e-12)
    fields=['vac_updated','mode','sync_request','v_bus_ref','inject_current']
    for q in commands:q.setdefault('inject_current',0.0)
    initial=commands[0]
    for name in fields:b.add_voltage_source('scenario_'+name,'cmd_'+name,'0',initial[name])
    wires=[model.probes[SENSORS[name]] for name in ['VLow','ILow1','ILow2','VDCBus','VAC','IAC']]+[('v','cmd_'+name) for name in fields]
    block=attach(b,wires,dt);rows=[];last_count=0;last_report=-1
    duty=scenario.get('open_loop_duty')  # exploratory: both legs held at this duty, firmware not obeyed
    if duty is not None and dt>50e-9:raise ValueError('Open-loop switching runs require dt <= 50 ns')
    if duty is None:hardware=Actuators(scenario.get('bridge'))
    else:
        class Fixed(Actuators):
            def __init__(self):super().__init__();self.enabled=[True,True];self.duties=[float(duty)]*2
            def apply(self,out):
                if out[INDEX['block_error']]:raise RuntimeError('Native C-Block contract/timing failure')
        hardware=Fixed()
    original=block.step_observer;extrema={};peak_indices=[];electrical_observations=0
    pv_v=[0.]
    def observer(t,x):
        nonlocal last_count,last_report,electrical_observations
        if model.pv_lambertw:pv_v[0]=float(x[pv_rows[0]]-(x[pv_rows[1]] if pv_rows[1] is not None else 0.))
        if peak_indices:electrical_observations+=1
        for name,index in peak_indices:
            value=float(x[index]);extrema[name]['min']=min(extrema[name]['min'],value);extrema[name]['max']=max(extrema[name]['max'],value)
        original(t,x)
        if block.n_fires==last_count:return
        if cancelled and cancelled():raise RuntimeError('Cancelled')
        hardware.apply(block.outputs,float(t));last_count=block.n_fires
        row={'time_s':float(t),**dict(zip(CONTRACT['inputs'],map(float,block.inputs))),**dict(zip(NAMES,map(float,block.outputs)))}
        rows.append(row)
        if int(t*5)!=last_report:
            last_report=int(t*5)
            text=f'{t:.3f} s: real Core calls={int(row["invocations"])}, synchronized={int(row["synchronized"])}'
            if progress:progress(100*t/duration,text)
            else:print(text,flush=True)
    block.step_observer=observer
    names=b.state_var_names();ix={name:i for i,name in enumerate(names)};x=model.initial()
    light=scenario.get('light')  # [[t_s, scale], ...] irradiance profile, linear between points
    if light and not model.pv_lambertw:raise ValueError('A light profile needs pv_model lambertw')
    if model.pv_lambertw:
        from plant.pv import current_lambertw
        pv_rows=[None if n=='0' else ix[f'V({n})'] for n in model.nodes['PV_light'][:2]]
        light_t=[q[0] for q in light] if light else [0.];light_s=[q[1] for q in light] if light else [1.]
    if 'DCDC_test_load' in model.parts:
        for logical in ['ILow1','ILow2']:
            probe=model.probes[SENSORS[logical]][1]
            peak_indices.append((logical,ix[f'Is({probe})']))
            extrema[logical]={'min':float('inf'),'max':float('-inf')}
    for name in fields:x[ix['V(cmd_'+name+')']]=initial[name]
    if options and options.get('ideal_bus_V'):
        for n in ['Vout','MOD_bus']:
            if f'V({n})' in ix:x[ix[f'V({n})']]=options['ideal_bus_V']
    observer(0.,x)
    mask=p.SwitchStateMask(b.graph.num_switches);switches={name:b.switch_index_of(name) for name in hardware.gates(0)}
    def switch(t):
        for name,value in hardware.gates(t).items():mask.set(switches[name],bool(value))
        if 'DCDC_load_step' in model.parts:
            mask.set(b.switch_index_of('DCDC_load_step'),t+1e-14>=model.parts['DCDC_load_step'].parameters['fixture_step_time_s'])
        return mask
    extra=[0.]*len(names)
    def source_commands(t):
        # The fixed-step kernel evaluates source RHS at the interval start.
        # Deliver a tick-aligned scenario command for the interval endpoint.
        q=command(t+dt)
        for name in fields:extra[ix['Is(scenario_'+name+')']]=initial[name]-q[name]
        if grid:extra[ix['Is(Vgrid)']]=-grid_v(t+dt)  # source value = static (0) - extra
        if model.pv_lambertw:
            # Panel current from the terminal voltage of the previous step (explicit Lambert W),
            # pushed into the source's positive node: injection enters b_extra with a minus sign.
            i=float(current_lambertw(pv_v[0],model.pv_lambertw,float(np.interp(t,light_t,light_s))))
            if pv_rows[0] is not None:extra[pv_rows[0]]=-i
            if pv_rows[1] is not None:extra[pv_rows[1]]=i
        return extra
    started=time.monotonic()
    try:
        result=p.simulate(b,t_end=duration,dt=dt,engine='pwl',initial_state=x,switch_fn=switch,
            b_extra_fn=source_commands,store_every=max(1,round(scenario['store_period_s']/dt)),
            auto_regularize=False,max_event_iterations=32)
    finally:
        if block.teardown:block.teardown()
    t=np.asarray(result.times);states=np.asarray(result.states)
    signals={name:states[:,i] for i,name in enumerate(names) if not name.startswith(('V(core_','Is(Core_out'))}
    sample_t=np.array([row['time_s'] for row in rows]);index=np.maximum(0,np.searchsorted(sample_t,t,side='right')-1)
    for name in rows[0]:
        if name!='time_s':signals[name]=np.array([row[name] for row in rows])[index]
    for logical,probe in SENSORS.items():
        kind,ref=model.probes[probe]
        col=ix[('V' if kind=='v' else 'Is')+f'({ref})']
        signals[('VP' if kind=='v' else 'IP')+f'({probe})']=states[:,col]
    def mpp_power(times):
        """Available MPP power at each stored time, following the light profile when there is one."""
        if not light:return np.full_like(times,model.pv_reference['Pmp_W'])
        from plant.pv import reference
        scale=np.round(np.interp(times,light_t,light_s),3);cache={}
        for s in np.unique(scale):cache[s]=reference({**model.pv,'IL_A':model.pv['IL_A']*s})['Pmp_W'] if s>0 else 0.
        return np.array([cache[s] for s in scale])
    if model.pv:
        def voltage(node):return np.zeros_like(t) if node=='0' else states[:,ix[f'V({node})']]
        jpos,jneg=model.nodes['PV_light'][:2]
        vj=voltage(jpos)-voltage(jneg)
        r0=model.nodes['Rsource'][0]
        terminal=voltage(r0)-voltage(jneg)
        source_current=(vj-voltage(r0))/model.parts['Rsource'].parameters['resistance']
        signals.update(pv_voltage=terminal,pv_current=source_current,pv_power=terminal*source_current,
                       pv_mpp_power=mpp_power(t),
                       pv_sensor_power=signals['VLow']*(signals['ILow1']+signals['ILow2']))
    metadata={'scenario':scenario,'contract':CONTRACT['version'],'profile':CONTRACT['application_profile'],
        'model_kind':'switched','model_sha256':hashlib.sha256((HERE/'plant/full_model.pulsim').read_bytes()).hexdigest(),
        'model_payload_sha256':hashlib.sha256(json.dumps(project.to_dict(),sort_keys=True).encode()).hexdigest() if project else None,
        'firmware':json.loads((LIB.parent/'manifest.json').read_text()),
        'pulsim':p.__version__,'wall_s':time.monotonic()-started,'actual_cblock_calls':block.n_fires,
        'finite_states':bool(np.isfinite(states).all()),'native_cblock':True,'formal_campaign':False,
        'averaged_model_available':False,'pv_model_available':bool(model.pv),
        'pv_model':('lambertw' if model.pv_lambertw else 'diode') if model.pv else None,
        'open_loop_duty':duty,'grid_profile':grid or None,'plant_options':options,
        'bridge':hardware.br if duty is None else None,'relay':CONTRACT['actuators']['relay']}
    if model.pv:
        metadata.update(pv_parameters=model.pv,pv_reference=model.pv_reference,
                        pv_profile_sha256=hashlib.sha256((HERE/'plant/pv-profile.json').read_bytes()).hexdigest(),
                        pv_bus_load_ohm=model.parts['PV_bus_test_load'].parameters['resistance'],
                        pv_scope='switched integration; not formal tracking-efficiency acceptance')
    if 'DCDC_test_load' in model.parts:
        metadata['fixture']=model.parts['Cbus'].parameters['dcdc_fixture']
        metadata['electrical_step_extrema']=extrema
        metadata['electrical_observations']=electrical_observations
        metadata['initial_transformer_voltage_mismatch_V']=max(abs(
            (x[ix['V('+model.nodes[p+'_T1'][0]+')']]-x[ix['V('+model.nodes[p+'_T1'][1]+')']])-
            model.parts[p+'_T1'].parameters['turns_ratio']*(
                x[ix['V('+model.nodes[p+'_T1'][2]+')']]-x[ix['V('+model.nodes[p+'_T1'][3]+')']])) for p in ['DC1','DC2'])
        metadata['initial_capacitor_voltage_error_V']=max(abs(
            (0 if model.nodes[n][0]=='0' else x[ix['V('+model.nodes[n][0]+')']])-
            (0 if model.nodes[n][1]=='0' else x[ix['V('+model.nodes[n][1]+')']])-
            a.parameters.get('initial_voltage',0))
            for n,a in model.parts.items() if a.type.name=='CAPACITOR' and n.startswith(('DC1_C','DC2_C','Cbus')))
    return t,signals,rows,metadata

def save(data,out):
    """Write raw CSV, GUI-readable traces and exact source/model provenance."""
    t,signals,rows,meta=data;out=Path(out);out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'waveforms.npz',time=t,**signals)
    with (out/'control.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (out/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')

def main():
    """Run an explicit scenario; no automatic application transitions are inserted."""
    a=argparse.ArgumentParser();a.add_argument('--scenario',type=Path,default=HERE/'scenarios/idle_healthy.json');a.add_argument('--out',type=Path,required=True)
    args=a.parse_args();data=run(json.loads(args.scenario.read_text()));save(data,args.out)
    if not data[3]['finite_states']:raise SystemExit('Non-finite electrical states')
if __name__=='__main__':main()
