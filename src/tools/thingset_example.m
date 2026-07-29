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
%   thingset_example(Port="...-if02", ScopePort="...-if00", Capture=true)

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
    options.ScopePort (1,1) string = ""
    options.Capture (1,1) logical = false
    options.PretriggerRatio (1,1) double {mustBeFinite, mustBeInRange( ...
        options.PretriggerRatio, 0, 0.9)} = 0.2
    options.Decimation (1,1) double {mustBeInteger, mustBeInRange( ...
        options.Decimation, 1, 100)} = 1
    options.ScopeCsv (1,1) string = ""
end

if ~options.EnablePower && ...
        (options.ConnectDriver || options.ConnectCapacitor)
    error("thingset_example:safety", ...
        "ConnectDriver and ConnectCapacitor require EnablePower=true");
end
if options.Capture && strlength(options.ScopePort) == 0
    error("thingset_example:scope", ...
        "Capture=true requires an explicit ScopePort");
end
if ~options.Capture && strlength(options.ScopeCsv) > 0
    error("thingset_example:scope", ...
        "ScopeCsv requires Capture=true");
end

ts = ThingSetTools(options.Port, options.BaudRate);
transportCleanup = onCleanup(@() ts.close());
scope = [];
if options.Capture
    scope = ScopeSerial(options.ScopePort);
    scopeCleanup = onCleanup(@() scope.close());
end
ts.discover();
bench = PowerTestBench(ts, scope);

% Establish the safe baseline before applying any requested duty.
bench.shutdown();
bench.configureLeg( ...
    options.Leg, struct("dutyCycle", options.DutyCycle));

disp("Converter metadata:");
disp(bench.readMetadata());
disp("Measurements:");
disp(bench.readMeasurements());

if options.Capture
    bench.armScope( ...
        PretriggerRatio=options.PretriggerRatio, ...
        Decimation=options.Decimation);
    bench.triggerScope();
    bench.waitScopeReady();
    capture = bench.downloadScope();
    fprintf( ...
        "Scope capture: %d samples, %d us period, %g ms window.\n", ...
        size(capture.samples, 1), capture.samplePeriodUs, ...
        capture.durationMs);
    if strlength(options.ScopeCsv) > 0
        names = ["time_s", capture.channelNames];
        output = array2table( ...
            [capture.timeAxisS, capture.samples], ...
            VariableNames=cellstr(names));
        writetable(output, options.ScopeCsv);
        fprintf("Scope CSV written to %s.\n", options.ScopeCsv);
    end
end

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
