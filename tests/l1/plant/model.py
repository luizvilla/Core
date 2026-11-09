#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file model.py
## @brief Strict native translation of the accepted switched model and six sensor probes.
"""Only physical translation; controllers and application logic execute in the C-Block."""
from pathlib import Path
import json,hashlib
import numpy as np
import pulsim as p
from pulsimgui.models.project import Project
from pulsimgui.utils.net_utils import build_node_map,build_node_alias_map
HERE=Path(__file__).resolve().parent
SENSORS={'VLow':'DC1_Vin','ILow1':'DC1_Iin','ILow2':'DC2_Iin','VDCBus':'DC_bus','VAC':'Grid_voltage','IAC':'Grid_current'}
class Model:
    """Native electrical elements plus exact physical-probe bindings."""
    def __init__(self,project=None,options=None):
        """@param options inverter-test plant variants, each recorded by the runner:
        output_chokes (False: FL1/FL2 windings shorted, the bench board has none),
        grid_series_ohm / grid_series_h (impedance between the ideal grid source and the PCC,
        where the VAC sensor is: series bulb, reference grid impedance)."""
        options=options or {}
        if project is None:
            meta=json.loads((HERE/'params.yaml').read_text())
            if hashlib.sha256((HERE/'full_model.pulsim').read_bytes()).hexdigest()!=meta['model_sha256']:raise ValueError('Accepted model hash no longer matches params.yaml')
            project=Project.load(HERE/'full_model.pulsim')
        c=project.get_active_circuit();nm=build_node_map(c);aliases=build_node_alias_map(c,nm)
        from plant.pv import parameters, reference
        self.pv=parameters(c);self.pv_reference=reference(self.pv) if self.pv else None
        self.builder=p.CircuitBuilder();b=self.builder;self.parts={};self.nodes={};self.probes={};self.pv_lambertw=None
        for a in c.components.values():
            name=a.name;kind=a.type.name;q=a.parameters
            ns=['0' if n=='0' else aliases.get(n,'N'+n) for n in [nm[(str(a.id),i)] for i in range(len(a.pins))]]
            self.parts[name]=a;self.nodes[name]=ns
            if kind=='RESISTOR':b.add_resistor(name,*ns[:2],q['resistance'])
            elif kind=='CAPACITOR':b.add_capacitor(name,*ns[:2],q['capacitance'])
            elif kind=='INDUCTOR':b.add_inductor(name,*ns[:2],q['inductance'])
            elif kind=='SWITCH':b.add_switch(name,*ns[:2],1/q['ron'],1/q['roff'])
            elif kind=='DIODE':
                if q.get('pv_shockley'):
                    b.add_shockley_diode(name,*ns[:2],I_S=q['is_'],n=q['n'],V_T=q['thermal_voltage'],G_min=q['g_min'])
                else:b.add_diode(name,*ns[:2],1/q['ron'],1/q['roff'],q['v_forward'])
            elif kind=='CURRENT_SOURCE':
                if name!='PV_light' or q['waveform']['type']!='dc':raise ValueError('Unexpected current source '+name)
                # Lambert W panel: value 0 here, the runner injects I(V) at every step.
                if q.get('pv_lambertw'):self.pv_lambertw=dict(q['pv_lambertw'])
                b.add_current_source(name,*ns[:2],q['waveform']['value'])
            elif kind=='TRANSFORMER':b.add_transformer(name,*ns[:4],q['lm'],q['lm']/q['turns_ratio']**2,q['coupling'])
            elif kind=='COUPLED_INDUCTOR' and name in ('FL1','FL2') and options.get('output_chokes',True) is False:
                b.add_resistor(name+'_w1',ns[0],ns[1],1e-3);b.add_resistor(name+'_w2',ns[2],ns[3],1e-3)
            elif kind=='COUPLED_INDUCTOR':
                k=q['coupling_coefficient']
                if abs(q['mutual_inductance']-k*np.sqrt(q['l1']*q['l2']))>1e-12:raise ValueError(f'Inconsistent mutual coupling: {name}')
                b.add_transformer(name,*ns[:4],q['l1'],q['l2'],k)
            elif kind=='VOLTAGE_SOURCE':
                w=q['waveform']
                if name=='Vgrid' and (options.get('grid_series_ohm') or options.get('grid_series_h')):
                    # Ideal source behind the impedance; the PCC keeps the original node (VAC sensor).
                    pcc=ns[0];ns=['Vgrid_src',ns[1]];mid='Vgrid_src'
                    if options.get('grid_series_h'):
                        b.add_inductor('Grid_series_L',mid,'Vgrid_mid',options['grid_series_h']);mid='Vgrid_mid'
                    b.add_resistor('Grid_series_R',mid,pcc,options.get('grid_series_ohm') or 1e-3)
                if w['type']=='dc':b.add_voltage_source(name,*ns[:2],w['value'])
                elif w['type']=='sine':b.add_sine_voltage_source(name,*ns[:2],w.get('offset',0),w['amplitude'],w['frequency'],np.deg2rad(w.get('phase',0)))
                else:raise ValueError(f'Unsupported waveform {name}')
            elif kind=='CURRENT_PROBE':
                branch='probe_'+name;b.add_voltage_source(branch,*ns[:2],0);self.probes[name]=('i',branch)
            elif kind=='VOLTAGE_PROBE':
                out='measurement_'+name;b.add_vcvs('measure_'+name,*ns[:2],out,'0',1);self.probes[name]=('v',out)
            elif kind=='C_BLOCK':
                if name!='Core_controlStep' or q['n_inputs']!=len(json.loads((HERE.parent/'contract.json').read_text())['inputs']) or q['n_outputs']!=len(json.loads((HERE.parent/'contract.json').read_text())['outputs']) or q.get('contract')!=json.loads((HERE.parent/'contract.json').read_text())['version']:raise ValueError('Unexpected firmware block contract '+name)
                expected=HERE.parent/'firmware_block/build/libcore_control.so'
                if Path(q['lib_path']).resolve()!=expected.resolve():raise ValueError('GUI block library is not the verified Core build')
            elif kind=='CONSTANT':
                if not name.startswith('Scenario_'):raise ValueError('Unexpected scenario source '+name)
            elif kind=='PWM_GENERATOR':
                if not name.startswith('Core_PWM_DC') or not q.get('core_adapter') or q['frequency']!=200000. or q['deadtime_s']!=1e-7:
                    raise ValueError('PWM adapter differs from the versioned hardware contract: '+name)
            elif kind not in {'GROUND','GOTO_LABEL','FROM_LABEL','SUBCIRCUIT_PORT','ELECTRICAL_SCOPE'}:raise ValueError(f'Unsupported element {kind}: {name}')
        for name in SENSORS.values():
            if name not in self.probes:raise ValueError('Missing physical sensor '+name)
    def initial(self):
        """Honor declared capacitor currents/voltages and energized input conditions."""
        b=self.builder;names=b.state_var_names();ix={n:i for i,n in enumerate(names)};x=np.zeros(len(names))
        def setv(n,v):
            if n!='0':x[ix[f'V({n})']]=v
        if self.pv:
            v=self.pv_reference['Voc_V'];setv(self.nodes['PV_light'][0],v)
        else:
            v=self.parts['Vsource'].parameters['waveform']['value'];setv(self.nodes['Vsource'][0],v)
        setv(self.nodes['DC1_Vin'][0],v)
        for prefix in ['DC1','DC2']:
            for part in ['L3','RL3','Iin','T1']:
                for n in self.nodes[prefix+'_'+part][:2]:setv(n,v)
            setv(self.nodes[prefix+'_C1'][0],v)
        for _ in range(3):
            for name,a in self.parts.items():
                if a.type.name=='CAPACITOR' and name.endswith(('_C1','_C2','_C7')):
                    ns=self.nodes[name];neg=0 if ns[1]=='0' else x[ix[f'V({ns[1]})']]
                    setv(ns[0],neg+a.parameters.get('initial_voltage',0))
        for name,a in self.parts.items():
            if a.type.name=='INDUCTOR':x[ix[f'I({name})']]=a.parameters.get('initial_current',0)
        if self.pv or 'DCDC_test_load' in self.parts:
            setv(self.nodes['Cbus'][0],self.parts['Cbus'].parameters['initial_voltage'])
            for prefix in ['DC1','DC2']:
                setv(self.nodes[prefix+'_C6'][0],self.parts[prefix+'_C6'].parameters['initial_voltage'])
        if 'DCDC_test_load' in self.parts:
            # An initially unenergized transformer must not begin with an
            # artificial secondary voltage merely because floating rectifier
            # nodes were left at the zero-vector default during precharge.
            for prefix in ['DC1','DC2']:
                mid=x[ix['V('+self.nodes[prefix+'_T1'][2]+')']]
                for n in self.nodes[prefix+'_Rs'][:2]:setv(n,mid)
        for name,(kind,ref) in self.probes.items():
            if kind=='v':
                ns=self.nodes[name];value=sum((1 if i==0 else -1)*(0 if n=='0' else x[ix[f'V({n})']]) for i,n in enumerate(ns[:2]))
                setv(ref,value)
        return x
