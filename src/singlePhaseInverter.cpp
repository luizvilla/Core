/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file is a power control class for single phase inverters
 *
 * @author Regis Ruelland <rruelland@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */


#include "singlePhaseInverter.h"

// Constructor for singlePhaseInverter
singlePhaseInverter::singlePhaseInverter() : _w(0.0F) {}

// Initialization function for singlePhaseInverter
int8_t singlePhaseInverter::init(inverter_mode mode, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts) {

    // parameters of the SOGI filter
    float32_t rise_time = 1.0F * 2.0F * PI / grid_w0;
    float32_t wn = 3.0F / rise_time;
    float32_t xsi = 0.7F;
    float32_t Kp = 2 * wn * xsi / grid_Vpk;
    float32_t Ki = (wn * wn) / grid_Vpk;
    float32_t Kr = 500.0;

    _mode = mode;

    _grid_Vpk = grid_Vpk;
    _w = grid_w0;
    _w_ref = grid_w0;
    _Ts = Ts;
    _theta = 0;
    _next_theta = 0;

    _VN_ref = 0;

    _sogi_v.init(Kr, _Ts);
    _sogi_i.init(Kr, _Ts);

    _pll_pi_params.Ts = Ts;
    _pll_pi_params.Td = 0.0;
    _pll_pi_params.N = 1;
    _pll_pi_params.Ti = Kp / Ki;
    _pll_pi_params.Kp = Kp;
    _pll_pi_params.lower_bound = -0.1F * _w_ref;
    _pll_pi_params.upper_bound = 0.1F * _w_ref;

    _pll_pi.init(_pll_pi_params);
    _pll_pi.reset(0);

    _Idq_ref.d = 0.0;
    _Idq_ref.q = 0.0;
    _Vdq_ref.d = 0.0;
    _Vdq_ref.q = 0.0;

    _Idq_ref_max.d = 8.0;
    _Idq_ref_max.q = 1.0;
    _Idq_ref_min.d = -0.1;
    _Idq_ref_min.q = -0.1;

    _Vdq_ref_max.d = 30.0;
    _Vdq_ref_max.q = 30.0;
    _Vdq_ref_min.d = -0.1;
    _Vdq_ref_min.q = -0.1;

    _Idq_ref_delta.d = 0.0;
    _Idq_ref_delta.q = 0.0;

    _current_pi_params.Ts = Ts;
    _current_pi_params.Kp = 0.010;  // Kp_i is L/Tcm L=66uH, Tc_i=6.6ms
    _current_pi_params.Ti = 3.3e-6; // Ti_i is L/R, R=20


    _current_pi_params.Td = 0.0;
    _current_pi_params.N = 1.0;
    _current_pi_params.upper_bound = 30;
    _current_pi_params.lower_bound = -30;

    _current_d_pi.init(_current_pi_params);
    _current_q_pi.init(_current_pi_params);


    _voltage_pi_params.Ts = Ts;
    _voltage_pi_params.Kp = 0.1;      // kp_v = Tc_i / Tv_i = 1/A (where A is a ratio between the two Tc. A must be greater than 1 or 10)
    _voltage_pi_params.Ti = 66e-3;      // Ti_v = 10*Tc_i (too much oscillation, we multiplied Ti_v by 10 )
    _voltage_pi_params.Td = 0.0;
    _voltage_pi_params.N = 1.0;
    _voltage_pi_params.upper_bound = 30;
    _voltage_pi_params.lower_bound = -30;

    _voltage_d_pi.init(_voltage_pi_params);
    _voltage_q_pi.init(_voltage_pi_params);



    return 0;  // Return 0 to indicate success
}

void singlePhaseInverter::inputProcessing(float32_t vgrid_meas, float32_t igrid_meas)
{
    _theta = ot_modulo_2pi(_theta + _w * _Ts);

    if(_mode == FORMING){
        _w = _w_ref;
    }else if(_mode == FOLLOWING){
        _w = _w_ref + _pll_pi.calculateWithReturn(0, -1.0*_Vdq.q);
    }

    _Vab = _sogi_v.calc(vgrid_meas,_w);
    _Iab = _sogi_i.calc(igrid_meas,_w);

    _Vdq = Transform::rotation_to_dqo(_Vab, _theta);
    _Idq = Transform::rotation_to_dqo(_Iab, _theta);

}

// Calculate function for singlePhaseInverter
float32_t singlePhaseInverter::calculateDuty() {


    if(_mode == FORMING){
        /* This is the outer voltage loop of the grid forming */
        _Idq_ref_delta.d = _voltage_d_pi.calculateWithReturn(_Vdq_ref.d, _Vdq.d); 
        _Idq_ref_delta.q = _voltage_q_pi.calculateWithReturn(_Vdq_ref.q, _Vdq.q); 

        /* This is the inner current loop of the grid forming */
        _Vdq_output.d = _current_d_pi.calculateWithReturn(_Idq_ref_delta.d + _Idq_ref.d, _Idq.d); 
        _Vdq_output.q = _current_q_pi.calculateWithReturn(_Idq_ref_delta.q + _Idq_ref.q, _Idq.q); 

    }else if(_mode == FOLLOWING){
        /* This is the current control loop of the grid following */
        _Vdq_output_pid.d = _current_d_pi.calculateWithReturn(_Idq_ref.d, _Idq.d); 
        _Vdq_output_pid.q = _current_q_pi.calculateWithReturn(_Idq_ref.q, _Idq.q); 
    }


    if(_mode == FORMING){
        // _Vdq_output.d = _Vdq_output.d + _Vdq_ref.d; 
        // _Vdq_output.q = _Vdq_output.q + _Vdq_ref.q;
        _Vdq_output.d =  _Vdq_ref.d; 
        _Vdq_output.q =  _Vdq_ref.q;
    }else if(_mode == FOLLOWING){
        _Vdq_output.d = _Vdq_output_pid.d + _Vdq.d; 
        _Vdq_output.q = _Vdq_output_pid.q + _Vdq.q;
    }

    _Vab_output = Transform::rotation_to_clarke(_Vdq_output, _theta);

    _Vond = _Vab_output.alpha;

    _duty_cycle = _Vond /(2.0F * _V_bus );

    // // // Calculate active and reactive power
    _power.d = 0.5F * (_Vdq.d * _Idq.d + _Vdq.q * _Idq.q);
    _power.q = 0.5F * (_Idq.d * _Vdq.q - _Idq.q * _Vdq.d);

    return _duty_cycle;

}

dqo_t singlePhaseInverter::getVdqIn(){
    return _Vdq;
}

dqo_t singlePhaseInverter::getVdqOut(){
    return _Vdq_output_pid;
}


dqo_t singlePhaseInverter::getIdq(){
    return _Idq;
}

dqo_t singlePhaseInverter::getIdqRefDelta(){
    return _Idq_ref_delta;
}


dqo_t singlePhaseInverter::getPower(){
    return _power;
}

clarke_t singlePhaseInverter::getIab(){
    return _Iab;
}

clarke_t singlePhaseInverter::getVab(){
    return _Vab;
}

clarke_t singlePhaseInverter::getVabOutput(){
    return _Vab_output;
}


float32_t singlePhaseInverter::getTheta(){
    return _theta;
}


float32_t singlePhaseInverter::getw(){
    return _w;
}

void singlePhaseInverter::setVBus(float32_t V_bus){
    _V_bus = V_bus;
}

void singlePhaseInverter::setIdqRef(dqo_t Idq_ref){
    _Idq_ref = Idq_ref;
}

void singlePhaseInverter::setVNRef(float32_t VN_ref){
    _VN_ref = VN_ref;
}


void singlePhaseInverter::setVdqRef(dqo_t Vdq_ref){
    _Vdq_ref = Vdq_ref;
}




