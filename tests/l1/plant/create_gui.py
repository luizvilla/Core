#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file create_gui.py
## @brief Derive a GUI view with the actually executed Core firmware block and timer adapters.
"""Preserve the accepted physical model; append signal-only blocks below it."""
from pathlib import Path
import json,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime import HERE,CONTRACT,LIB,NAMES
from plant.model import SENSORS
from pulsimgui.models.project import Project
from pulsimgui.utils.net_utils import build_node_map
from pulsimgui.models.component import Component,ComponentType as T,Pin
from pulsimgui.models.wire import Wire,WireSegment,WireConnection

def create(pv=False,scenario=None,dest=None):
    """Write a disposable derived view; never change the accepted model snapshot.
    @param dest Folder for the derived .pulsim (default plant/derived/)."""
    project=Project.load(HERE/'plant/full_model.pulsim')
    if pv:
        from plant.pv import apply
        sc=scenario or {}
        apply(project,model=sc.get('pv_model','diode'),initial_bus_V=sc.get('pv_initial_bus_V'))
    if scenario and scenario.get('fixture'):
        if pv:raise ValueError('DCDC ideal-source fixture and PV variant are separate')
        from plant.fixtures import apply
        apply(project,scenario['fixture'])
    c=project.get_active_circuit()
    bottom=max(p.y for p in c.components.values())+600
    center=min(p.x for p in c.components.values())+1500
    probes={p.name:p for p in c.components.values()};mapping=build_node_map(c)
    input_labels=[]
    for logical in CONTRACT['inputs']:
        if logical in SENSORS:
            p=probes[SENSORS[logical]]
            label=next(a.parameters['net_label'] for a in c.components.values() if a.type==T.GOTO_LABEL and mapping[(str(a.id),0)]==mapping[(str(p.id),2)])
        else:label='command_'+logical
        input_labels.append(label)
    p=Component(type=T.C_BLOCK,name='Core_controlStep',x=center,y=bottom,
        parameters={'n_inputs':len(CONTRACT['inputs']),'n_outputs':len(NAMES),'implementation':'library','lib_path':str(LIB),
            'sample_time':.0001,'inputs':input_labels,'contract':CONTRACT['version'],
            'scenario_timeline':scenario['commands'] if scenario else [],
            'scenario_name':scenario['name'] if scenario else 'gui-core-app',
            'source_code':(HERE/'firmware_block/wrapper.cpp').read_text()},
        pins=[Pin(i,'IN'+str(i),-80,(i-4.5)*36) for i in range(10)]+[Pin(i+10,'OUT'+str(i),80,(i-(len(NAMES)-1)/2)*28) for i in range(len(NAMES))])
    c.add_component(p)
    def port(component,pin,label,incoming):
        x,y=component.get_pin_position(pin)
        marker=Component(type=T.FROM_LABEL if incoming else T.GOTO_LABEL,name='',x=x-120 if incoming else x+120,y=y,parameters={'net_label':label})
        c.add_component(marker);mx,my=marker.get_pin_position(0)
        c.add_wire(Wire(segments=[WireSegment(x,y,mx,my)],start_connection=WireConnection(component.id,pin),end_connection=WireConnection(marker.id,0),alias=label))
    for i,label in enumerate(input_labels):port(p,i,label,True)
    for i,label in enumerate(NAMES):port(p,i+10,label,False)
    defaults={'vac_updated':1,'mode':3 if pv else 0,'sync_request':1,'v_bus_ref':400}
    if scenario:defaults.update({k:scenario['commands'][0][k] for k in defaults})
    for i,(name,value) in enumerate(defaults.items()):
        const=Component(type=T.CONSTANT,name='Scenario_'+name,x=center-850,y=bottom-160+i*100,parameters={'value':value})
        c.add_component(const);port(const,0,'command_'+name,False)
    # These are plant/hardware adapters, not a second implementation of controlStep.
    for channel in [1,2]:
        for index,role in enumerate(['low','high']):
            timer=Component(type=T.PWM_GENERATOR,name=f'Core_PWM_DC{channel}_{role}',x=center+900,y=bottom-240+(channel-1)*320+index*140,
                parameters={'frequency':200000.,'duty_cycle':.5,'enable_duty_input':True,'amplitude':1.,
                    'carrier':'triangle','core_adapter':True,'complementary':role=='high','deadtime_s':1e-7,
                    'enable_channel':f'en_leg{channel}','duty_channel':f'duty_leg{channel}'})
            c.add_component(timer)
            for pin in timer.pins:
                if pin.name=='OUT':port(timer,pin.index,f'DC{channel}_pwm_{role}',False)
                elif pin.name=='DUTY_IN':port(timer,pin.index,f'duty_leg{channel}',True)
    dest=Path(dest) if dest else HERE/'plant/derived';dest.mkdir(parents=True,exist_ok=True)
    project.name='Core controlStep — L1 integration (sync only)';project.simulation_settings.tstop=1.3
    project.simulation_settings.dt=1e-6;project.simulation_settings.max_step=1e-6
    if pv:
        project.name='Core MPPT — synthetic PV with DC bus load'
        project.simulation_settings.tstop=.02
        project.simulation_settings.dt=50e-9;project.simulation_settings.max_step=50e-9
    filename='core_pv_model.pulsim' if pv else 'core_full_model.pulsim'
    if scenario:
        project.name=scenario['name'];filename=scenario['name']+'.pulsim'
        project.simulation_settings.tstop=scenario['duration_s']
        project.simulation_settings.dt=scenario['dt_s'];project.simulation_settings.max_step=scenario['dt_s']
    project.save(dest/filename)
    print(project.path)
    return project.path
if __name__=='__main__':create()
