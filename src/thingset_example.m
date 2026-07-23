%
% Copyright (c) 2021-present LAAS-CNRS
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 2 of the License, or
%   (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program.  If not, see <https://www.gnu.org/licenses/>.
%
% SPDX-License-Identifier: GPL-2.0-or-later
%
% @author Luiz Villa <luiz.villa@laas.fr>
%

% thingset_example.m
%
% Usage example for ThingSetTools: discovers the device's ThingSet
% objects, auto-builds a {short_name: path} map for the Measurements
% group, reads a single measurement and the whole group, and writes a
% Config value.

MEAS = "Measurements";

ts = ThingSetTools("", 115200, 1.0, "2FE3", "", true); 
%ts = ThingSetTools();
ts.discover();

% Auto-build {short_name: full_path} for every measurement, e.g.
% "rV1Low_V" -> name "V1Low" (between the leading "r" and the "_V" unit).
measurements = containers.Map("KeyType", "char", "ValueType", "any");
for name = ts.fetchChildren(MEAS)
    tok = regexp(char(name), "^r(.+)_(\w+)$", "tokens", "once");
    if ~isempty(tok)
        measurements(tok{1}) = char(MEAS + "/" + name);
    end
end

disp(measurements.keys);
disp(measurements.values);

disp(ts.read(measurements("V1Low")));

% Flush all measurements and their current values at once.
disp(ts.read(MEAS));

ts.write("Config", struct("wBlinkPeriod_s", 0.5));

ts.close();
