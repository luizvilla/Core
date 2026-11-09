#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file test_rc.py
## @brief L1-INT-02: actual Pulsim C-Block execution, closed RC coupling and reset.
"""The same firmware library used by the full plant drives an RC stand-in here."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import pulsim as p
from runtime import attach,CONTRACT,NAMES,INDEX

def emit(id,ok,description,measured):
    """Emit the procedure's criterion line and return success."""
    print('\t'.join(['@@',id,'PASS' if ok else 'FAIL',description,'all observations hold',str(measured)]));return ok

def experiment():
    """Run actual ABI outputs through a source, R and C and back to VDCBus."""
    b=p.CircuitBuilder();values=[40,0,0,None,0,0,1,1,0,400,0];wires=[]  # app-v3: inject_current 0
    for i,value in enumerate(values):
        if value is None:wires.append(('v','bus'));continue
        node='input'+str(i);b.add_voltage_source('input_source'+str(i),node,'0',value);wires.append(('v',node))
    b.add_resistor('R','core_i_ref1','bus',1000)  # leg-1 current reference: nonzero from the first STARTUP tick (ADR 0003 starts fully skipped, duty 0);b.add_capacitor('C','bus','0',1e-6)
    cb=attach(b,wires,20e-6);rows=[];previous=0;native=cb.step_observer
    def observer(t,x):
        nonlocal previous
        native(t,x)
        if cb.n_fires!=previous:
            rows.append((float(t),cb.inputs.copy(),cb.outputs.copy()));previous=cb.n_fires
    cb.step_observer=observer
    names=b.state_var_names();x=np.zeros(len(names))
    for i,value in enumerate(values):
        if value is not None:x[names.index(f'V(input{i})')]=value
    observer(0.,x)
    try:r=p.simulate(b,t_end=.003,dt=20e-6,initial_state=x,store_every=1)
    finally:
        if cb.teardown:cb.teardown()
    return rows,np.asarray(r.states)[:,names.index('V(bus)')]

def main():
    """Verify the sampled native block and fresh repeated runs."""
    rows,bus=experiment();again,other=experiment();times=np.array([r[0] for r in rows]);out=np.array([r[2] for r in rows]);ok=True
    # @test L1-INT-02.a: Pulsim invokes the native ABI exactly once per control tick.
    ok=emit('L1-INT-02.a',len(rows)==31 and np.allclose(np.diff(times),.0001,atol=1e-12,rtol=0) and np.array_equal(out[:,INDEX['invocations']],np.arange(1,32)),'Pulsim schedules real native controlStep once per 100 us',len(rows)) and ok
    # @test L1-INT-02.b: Firmware source output drives the RC and its measurement returns.
    sampled=np.array([r[1][3] for r in rows])
    ok=emit('L1-INT-02.b',np.max(bus)>0 and np.max(sampled)>0 and np.all(out[:,INDEX['block_error']]==0),'Native C-Block drives RC and reads its changing bus voltage',float(bus.max())) and ok
    # @test L1-INT-02.c: New contexts/runs do not leak application state.
    ok=emit('L1-INT-02.c',np.array_equal(bus,other) and np.array_equal(out,np.array([r[2] for r in again])),'Repeated new model/block starts identically',len(again)) and ok
    raise SystemExit(0 if ok else 1)
if __name__=='__main__':main()
