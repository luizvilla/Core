#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file safety_allocation.py
## @brief Hardware protection allocation (ADR 0009, issue luizvilla/Core#17).
##
## Given a shield's sensors, limits, legs and safe outputs, assigns each limit bound the strongest protection path
## the STM32G474 offers on that board:
##
##   comparator path    COMPx (non-inverting input on the sensor pin) -> HRTIM FLTy        one bound, ~31 ns
##   ADC-watchdog path  ADCn analog watchdog k (window on the channel) -> EEV -> FLTy      both bounds, 1 conversion
##   software path      the Safety API's polling, only where the shield states an exception
##
## The six HRTIM fault inputs are shared: FLTy takes a pin, a comparator or an external event (RM0440 Table 82),
## and only ADC1/ADC2 watchdogs reach the external events behind FLT1-6 (RM0440 Table 220). A limit whose bounds
## cannot all get a hardware path is an error unless the shield gives an exception.
##
## Standard library only. Used at build time (protection_manifest.py, from Zephyr's edt.pickle), by the host tests
## (tests/l0/safety_allocation) and by tools/safety_check.py.

# ---- STM32G474 routing (RM0440 Rev 8) ----------------------------------------------------------------------------

# Comparator non-inverting inputs: MCU pin -> (comparator, INPSEL) (Table 196)
COMP_INP = {'PA1': (1, 0), 'PB1': (1, 1), 'PA7': (2, 0), 'PA3': (2, 1), 'PA0': (3, 0), 'PC1': (3, 1),
            'PB0': (4, 0), 'PE7': (4, 1), 'PB13': (5, 0), 'PD12': (5, 1), 'PB11': (6, 0), 'PD11': (6, 1),
            'PB14': (7, 0), 'PD14': (7, 1)}
# Comparator -> HRTIM fault input (Table 82, FLTxSRC = 01). COMP7 reaches no fault input.
COMP_FLT = {1: 4, 2: 1, 3: 5, 4: 2, 5: 6, 6: 3}
# Comparator inverting-input DACs (Table 197, INMSEL 100 and 101)
COMP_DACS = {1: ('DAC3_CH1', 'DAC1_CH1'), 2: ('DAC3_CH2', 'DAC1_CH2'), 3: ('DAC3_CH1', 'DAC1_CH1'),
             4: ('DAC3_CH2', 'DAC1_CH1'), 5: ('DAC4_CH1', 'DAC1_CH2'), 6: ('DAC4_CH2', 'DAC2_CH1'),
             7: ('DAC4_CH1', 'DAC2_CH1')}
# ADC analog watchdog -> external event -> HRTIM fault input (Table 220, EExSRC = 11; Table 82, FLTxSRC = 10)
AWD_FLT = {('adc1', 1): 1, ('adc1', 2): 2, ('adc1', 3): 3, ('adc2', 1): 4, ('adc2', 2): 5, ('adc2', 3): 6}
# AWD1 compares 12 bits on one channel; AWD2/AWD3 compare the 8 most significant bits (RM0440 §21.4.28)
AWD_BITS = {1: 12, 2: 8, 3: 8}
ADC_MAX = 4095


def counts(value, gain, offset):
    """@brief ADC counts (12 bit) at which the sensor reads value: value = counts * gain + offset. None if unknown."""
    if gain in (None, 0):
        return None
    raw = (value - offset) / gain
    return max(0, min(ADC_MAX, int(round(raw))))


# EEV4/EEV5 are taken by peak-current mode (COMP1/COMP3 -> EEV4/EEV5, owntech_hrtim_driver _CM_init_EEV): the
# ADC2 watchdogs 1 and 2, which reach FLT4/FLT5 only through them, are never assigned.
AWD_RESERVED = {('adc2', 1), ('adc2', 2)}


def hardware_bounds(limit):
    """Bounds that need a hardware path: 'hardware-bounds' if set, else both for a current (name starting with I)
    and the high bound only otherwise (a low voltage is a range check, not a hazard: a boost output starts at 0 V)."""
    hb = limit.get('hardware_bounds')
    if hb is None:
        hb = 'both' if limit['sensor'].upper().startswith('I') else 'high'
    return ['high', 'low'] if hb == 'both' else [hb]


def _bound_order(limit):
    """Bounds needing hardware, in protection order: the comparator bound first."""
    need = hardware_bounds(limit)
    first = limit.get('comparator_bound', 'high')
    return sorted(need, key=lambda b: 0 if b == first else 1)


def allocate(model):
    """
    @brief Assigns protection paths. Deterministic: by 'priority' (lower first; default 10 for currents, 20
           otherwise), then devicetree order;
           per limit, the comparator on its comparator bound, then an ADC-watchdog window for the bounds left.
    @param model dict with
           'sensors': {name: {'gain', 'offset', 'conv', 'channels': [{'adc': 'adc1', 'channel': 1, 'spin_pin': 30,
                                                                          'mcu_pin': 'PA1'}]}},
           'limits':  [{'sensor', 'high', 'low', 'protection': 'hardware'|'software', 'exception': str|None,
                        'comparator_bound': 'high'|'low'}],
           'legs':    [{'name', 'timer', 'safe_state'}],
           'safe_outputs': [{'name', 'spin_pin', 'mcu_pin', 'safe_level', 'reset_state_safe'}],
           'reserved': {'comparators': [...], 'flt': [...]}  (resources an application already uses)
    @return dict: 'limits' (with 'paths' per bound and 'covered'), 'errors', 'warnings', 'resources'
    """
    reserved = model.get('reserved', {})
    used_comp = {c: 'reserved' for c in reserved.get('comparators', [])}
    used_flt = {f: 'reserved' for f in reserved.get('flt', [])}
    used_awd = {}
    used_dac = set(reserved.get('dacs', []))
    errors, warnings, out_limits = [], [], []

    def rank(i):
        lim = model['limits'][i]
        default = 10 if lim['sensor'].upper().startswith('I') else 20   # currents first
        return (lim.get('priority') if lim.get('priority') is not None else default, i)

    order = sorted(range(len(model.get('limits', []))), key=rank)
    results = [None] * len(order)
    for i in order:
        lim = model['limits'][i]
        name = lim['sensor']
        sensor = model.get('sensors', {}).get(name)
        entry = {'sensor': name, 'high': lim.get('high'), 'low': lim.get('low'),
                 'protection': lim.get('protection', 'hardware'), 'exception': lim.get('exception'),
                 'paths': {'high': None, 'low': None}, 'covered': False}
        if sensor is None:
            errors.append(f'{name}: limit on a sensor the shield does not declare')
            results[i] = entry
            continue
        gain, offset = sensor.get('gain'), sensor.get('offset')
        linear = sensor.get('conv', 'LINEAR') in (None, 'LINEAR')
        if entry['protection'] == 'software':
            entry['paths'] = {'high': {'kind': 'software'}, 'low': {'kind': 'software'}}
            entry['covered'] = True
            results[i] = entry
            continue
        need = _bound_order(lim)
        soft = [x for x in ('high', 'low') if x not in need]
        for x in soft:
            entry['paths'][x] = {'kind': 'software'}
        # 1. Comparator on the comparator bound, from any channel's pin.
        b = need[0] if need else None
        for ch in (sensor['channels'] if b == lim.get('comparator_bound', 'high') else []):
            comp = COMP_INP.get(ch.get('mcu_pin'), (None,))[0]
            flt = COMP_FLT.get(comp)
            if comp is None or flt is None or comp in used_comp or flt in used_flt or not linear:
                continue
            dac = next((d for d in COMP_DACS[comp] if d not in used_dac), None)
            if dac is None:
                continue
            used_dac.add(dac)
            used_comp[comp] = name
            used_flt[flt] = f'{name} {b} (COMP{comp})'
            entry['paths'][b] = {'kind': 'comparator', 'comp': comp, 'inpsel': COMP_INP[ch['mcu_pin']][1],
                                 'flt': flt, 'mcu_pin': ch['mcu_pin'], 'spin_pin': ch['spin_pin'],
                                 'polarity': 'non-inverted' if b == 'high' else 'inverted',
                                 'dac_options': list(COMP_DACS[comp]), 'dac': dac,
                                 'dac_code': counts(lim.get(b), gain, offset) if linear else None}
            break
        # 2. ADC-watchdog window for the bounds still open (it covers both).
        left = [x for x in need if entry['paths'][x] is None]
        if left and linear:
            done = False
            # The watchdog only sees what data acquisition converts: the sensor's default ADC (any, if it has none).
            dflt = sensor.get('default_adc')
            chans = [c for c in sensor['channels'] if dflt is None or c['adc'] == dflt]
            for ch in chans:
                for k in (1, 2, 3):
                    key = (ch['adc'], k)
                    flt = AWD_FLT.get(key)
                    if flt is None or key in used_awd or key in AWD_RESERVED or flt in used_flt:
                        continue
                    used_awd[key] = name
                    used_flt[flt] = f'{name} {"/".join(left)} ({ch["adc"].upper()} AWD{k})'
                    shift = 12 - AWD_BITS[k]
                    hi, lo = counts(lim.get('high'), gain, offset), counts(lim.get('low'), gain, offset)
                    # A bound without hardware protection leaves that side of the window open.
                    c_hi = (hi >> shift) if 'high' in need and hi is not None else (ADC_MAX >> shift)
                    c_lo = (lo >> shift) if 'low' in need and lo is not None else 0
                    for x in left:
                        entry['paths'][x] = {'kind': 'adc_watchdog', 'adc': ch['adc'], 'awd': k, 'eev': flt,
                                             'flt': flt, 'channel': ch['channel'], 'bits': AWD_BITS[k],
                                             'counts_high': c_hi, 'counts_low': c_lo}
                    done = True
                    break
                if done:
                    break
        missing = [x for x in need if entry['paths'][x] is None]
        if missing:
            if entry['exception']:
                for x in missing:
                    entry['paths'][x] = {'kind': 'software', 'exception': entry['exception']}
                entry['covered'] = True
            else:
                why = ('not a linear sensor' if not linear else
                       'no free comparator, and no free ADC watchdog on its default ADC '
                       f'({(sensor.get("default_adc") or "any").upper()})')
                errors.append(f'{name}: no hardware path for the {" and ".join(missing)} bound ({why}). '
                              f'Route the sensor to a free comparator input or ADC1/ADC2 channel, or state why '
                              f'software protection is acceptable with exception = "..." on its safety-thresholds node.')
        else:
            entry['covered'] = True
        results[i] = entry
    out_limits = [r for r in results if r is not None]

    for leg in model.get('legs', []):
        s = leg.get('safe_state', 'open')
        if s not in ('open', 'short-low', 'short-high'):
            errors.append(f'leg {leg["name"]}: safe-state "{s}" is not open, short-low or short-high')
        if s.startswith('short'):
            # a short-* leg must trip on its own low-side current (ADR 0009)
            cur = leg.get('current_sensor')
            ok = any(l['sensor'] == cur and l['paths'].get('high') and l['paths']['high']['kind'] != 'software'
                     for l in out_limits)
            if not ok and not leg.get('short_risk_accepted'):
                errors.append(f'leg {leg["name"]}: safe-state "{s}" needs a hardware trip on its own current '
                              f'({cur or "no current sensor declared"}), or short-risk-accepted with a reason')

    for so in model.get('safe_outputs', []):
        if not so.get('reset_state_safe'):
            warnings.append(
                f'safe output {so["name"]} (Spin pin {so.get("spin_pin")}, {so.get("mcu_pin")}) is not declared '
                f'safe at reset. Why it matters: after a watchdog reset the MCU pins float; if nothing on the board '
                f'pulls this pin to its safe level ({so.get("safe_level")}), it may stay active while the firmware '
                f'reboots. Fix: check the schematic for a pull resistor on this net, then add reset-state-safe; to '
                f'its node in the shield devicetree. See docs/safety_new_board.md.')

    return {'limits': out_limits, 'legs': model.get('legs', []), 'safe_outputs': model.get('safe_outputs', []),
            'errors': errors, 'warnings': warnings,
            'resources': {'comparators': {f'COMP{c}': v for c, v in sorted(used_comp.items())},
                          'flt': {f'FLT{f}': v for f, v in sorted(used_flt.items())},
                          'adc_watchdogs': {f'{a.upper()} AWD{k}': v for (a, k), v in sorted(used_awd.items())},
                          'free_flt': [f'FLT{f}' for f in range(1, 7) if f not in used_flt]}}


def path_text(p):
    """@brief Short text of one bound's path (boot line, tables)."""
    if p is None:
        return 'none'
    if p['kind'] == 'comparator':
        return f'COMP{p["comp"]}->FLT{p["flt"]}'
    if p['kind'] == 'adc_watchdog':
        return f'{p["adc"].upper()} AWD{p["awd"]}->FLT{p["flt"]}'
    return 'software' + (' (exception)' if p.get('exception') else '')


def markdown_table(shield, alloc):
    """@brief The board's protection table (docs), one row per limit."""
    rows = [f'# Protection table: {shield}', '',
            'Generated from the shield devicetree by the build (ADR 0009). Do not edit by hand.', '',
            '| Limit | Low | High | Low bound path | High bound path | Note |', '|---|---|---|---|---|---|']
    for l in alloc['limits']:
        note = l['exception'] or ''
        rows.append(f'| {l["sensor"]} | {l["low"]:g} | {l["high"]:g} | {path_text(l["paths"]["low"])} | '
                    f'{path_text(l["paths"]["high"])} | {note} |')
    rows += ['', '| Leg | Safe state |', '|---|---|']
    rows += [f'| {g["name"]} | {g.get("safe_state", "open")} |' for g in alloc['legs']]
    if alloc['safe_outputs']:
        rows += ['', '| Safe output | Spin pin | Safe level | Safe at reset |', '|---|---|---|---|']
        rows += [f'| {s["name"]} | {s.get("spin_pin")} | {s.get("safe_level")} | '
                 f'{"yes" if s.get("reset_state_safe") else "**not declared**"} |' for s in alloc['safe_outputs']]
    r = alloc['resources']
    rows += ['', f'Fault inputs used: {", ".join(f"{k} ({v})" for k, v in r["flt"].items()) or "none"}. '
                 f'Free: {", ".join(r["free_flt"]) or "none"}.']
    return '\n'.join(rows) + '\n'
