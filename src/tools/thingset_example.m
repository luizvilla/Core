%
% Copyright (c) 2021-present LAAS-CNRS
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 2 of the License, or
%   (at your option) any later version.
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

function thingset_example(options)
%THINGSET_EXAMPLE Safe-by-default ThingSet power-test-bench example.
%
%   thingset_example()
%   thingset_example(Port="/dev/ttyACM1", Leg=2, DutyCycle=0.2)
%   thingset_example(EnablePower=true, ConnectDriver=true, Duration=1.0)

arguments
    options.Port (1,1) string = ""
    options.BaudRate (1,1) double {mustBePositive} = 115200
    options.Leg (1,1) double {mustBeMember(options.Leg, [1, 2])} = 1
    options.DutyCycle (1,1) double {mustBeFinite, mustBeInRange( ...
        options.DutyCycle, 0, 1)} = 0.1
    options.Duration (1,1) double {mustBeNonnegative, mustBeFinite} = 1.0
    options.EnablePower (1,1) logical = false
    options.ConnectDriver (1,1) logical = false
    options.ConnectCapacitor (1,1) logical = false
end

if ~options.EnablePower && ...
        (options.ConnectDriver || options.ConnectCapacitor)
    error("thingset_example:safety", ...
        "ConnectDriver and ConnectCapacitor require EnablePower=true");
end

ts = ThingSetTools(options.Port, options.BaudRate);
transportCleanup = onCleanup(@() ts.close());
ts.discover();
bench = PowerTestBench(ts);

% Establish the safe baseline before applying any requested duty.
bench.shutdown();
bench.configureLeg( ...
    options.Leg, struct("dutyCycle", options.DutyCycle));

disp("Converter metadata:");
disp(bench.readMetadata());
disp("Measurements:");
disp(bench.readMeasurements());

if ~options.EnablePower
    disp("Power remains OFF; pass EnablePower=true to energize one leg.");
    return
end

disconnectAfterward = ...
    options.ConnectDriver || options.ConnectCapacitor;
powerCleanup = onCleanup( ...
    @() safeShutdown(bench, disconnectAfterward));
bench.powerOn( ...
    options.Leg, ...
    struct("dutyCycle", options.DutyCycle), ...
    ConnectDriver=options.ConnectDriver, ...
    ConnectCapacitor=options.ConnectCapacitor);
fprintf("Leg %d is powered for %.3f seconds.\n", ...
    options.Leg, options.Duration);
pause(options.Duration);
disp(bench.readMeasurements());
end

function safeShutdown(bench, disconnectHardware)
try
    bench.shutdown(DisconnectHardware=disconnectHardware);
catch ME
    warning("thingset_example:shutdown", ...
        "Shutdown completed with errors: %s", ME.message);
end
end
