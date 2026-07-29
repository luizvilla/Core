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

classdef PowerTestBench < handle
    % PowerTestBench  Safety-oriented API over a ThingSet transport.
    %
    % The supplied client must provide read(path) and write(path, values)
    % methods, as ThingSetTools does.

    properties (SetAccess = private)
        Client
    end

    properties (Constant)
        ModeIdle = 0
        ModePowerOn = 1
        ModePowerOff = 2
    end

    methods
        function obj = PowerTestBench(client)
            if nargin ~= 1 || ~ismethod(client, "read") || ~ismethod(client, "write")
                error("PowerTestBench:invalidClient", ...
                    "client must provide read(path) and write(path, values)");
            end
            obj.Client = client;
        end

        function modeValue = setMode(obj, mode)
            % Set and verify Config/Mode.
            modeValue = PowerTestBench.normalizeMode(mode);
            obj.writeAndVerify("Config", struct("Mode", modeValue));
        end

        function frequencyHz = setFrequency(obj, frequencyHz)
            % Set and verify Config/Frequency_Hz.
            frequencyHz = PowerTestBench.validateInteger( ...
                "frequencyHz", frequencyHz, 1, Inf);
            obj.writeAndVerify( ...
                "Config", struct("Frequency_Hz", frequencyHz));
        end

        function readback = configureLeg(obj, leg, settings)
            % Configure a leg from a scalar settings struct.
            %
            % Supported fields: enable, capacitor, driver, buck, boost,
            % dutyCycle, referenceValue, trackingVar, phaseShift,
            % deadTimeRisingNs, and deadTimeFallingNs.
            arguments
                obj
                leg
                settings (1,1) struct
            end

            legNumber = PowerTestBench.normalizeLeg(leg);
            allowed = [ ...
                "enable", "capacitor", "driver", "buck", "boost", ...
                "dutyCycle", "referenceValue", "trackingVar", ...
                "phaseShift", "deadTimeRisingNs", "deadTimeFallingNs"];
            PowerTestBench.rejectUnknownFields(settings, allowed, "leg setting");

            values = struct();
            boolMappings = {
                "enable", "wEnable";
                "capacitor", "wCapa";
                "driver", "wDriver";
                "buck", "wBuck";
                "boost", "wBoost"
            };
            for i = 1:size(boolMappings, 1)
                source = boolMappings{i, 1};
                target = boolMappings{i, 2};
                if isfield(settings, source)
                    values.(target) = PowerTestBench.validateLogical( ...
                        source, settings.(source));
                end
            end

            if isfield(settings, "buck") && isfield(settings, "boost") && ...
                    settings.buck && settings.boost
                error("PowerTestBench:validation", ...
                    "buck and boost cannot both be true");
            end
            if isfield(settings, "buck") && settings.buck
                values.wBoost = false;
            end
            if isfield(settings, "boost") && settings.boost
                values.wBuck = false;
            end

            if isfield(settings, "dutyCycle")
                duty = PowerTestBench.validateFinite( ...
                    "dutyCycle", settings.dutyCycle);
                if duty < 0 || duty > 1
                    error("PowerTestBench:validation", ...
                        "dutyCycle must be between 0 and 1");
                end
                values.wDutyCycle = duty;
            end
            if isfield(settings, "referenceValue")
                values.wReferenceValue = PowerTestBench.validateFinite( ...
                    "referenceValue", settings.referenceValue);
            end
            if isfield(settings, "trackingVar")
                values.wTrackingVar = char( ...
                    PowerTestBench.normalizeChannel(settings.trackingVar));
            end
            if isfield(settings, "phaseShift")
                values.wPhaseShift = PowerTestBench.validateInteger( ...
                    "phaseShift", settings.phaseShift, -360, 360);
            end
            if isfield(settings, "deadTimeRisingNs")
                values.wDeadTimeRising_ns = PowerTestBench.validateInteger( ...
                    "deadTimeRisingNs", settings.deadTimeRisingNs, 0, 65535);
            end
            if isfield(settings, "deadTimeFallingNs")
                values.wDeadTimeFalling_ns = PowerTestBench.validateInteger( ...
                    "deadTimeFallingNs", settings.deadTimeFallingNs, 0, 65535);
            end

            if isempty(fieldnames(values))
                error("PowerTestBench:validation", ...
                    "at least one leg setting must be supplied");
            end
            path = "Config/Leg" + legNumber;
            readback = obj.writeAndVerify(path, values, 5e-4);
        end

        function value = readLeg(obj, leg)
            % Read all configuration fields for one leg.
            legNumber = PowerTestBench.normalizeLeg(leg);
            value = obj.readObject("Config/Leg" + legNumber, "leg");
        end

        function value = readMeasurements(obj)
            % Read all live measurements.
            value = obj.readObject("Measurements", "measurement");
        end

        function value = readCalibration(obj, channel)
            % Read one sensor channel's calibration object.
            normalized = PowerTestBench.normalizeChannel(channel);
            value = obj.readObject("Calibration/" + normalized, "calibration");
        end

        function readback = setCalibration(obj, channel, settings)
            % Set gain/offset and optionally persist them with store=true.
            arguments
                obj
                channel
                settings (1,1) struct
            end
            normalized = PowerTestBench.normalizeChannel(channel);
            allowed = ["gain", "offset", "store"];
            PowerTestBench.rejectUnknownFields( ...
                settings, allowed, "calibration setting");

            values = struct();
            if isfield(settings, "gain")
                values.wGain = PowerTestBench.validateFinite( ...
                    "gain", settings.gain);
            end
            if isfield(settings, "offset")
                values.wOffset = PowerTestBench.validateFinite( ...
                    "offset", settings.offset);
            end
            if isfield(settings, "store")
                store = PowerTestBench.validateLogical("store", settings.store);
                if store
                    values.wStore = true;
                end
            end
            if isempty(fieldnames(values))
                error("PowerTestBench:validation", ...
                    "gain, offset, or store=true must be supplied");
            end

            expected = values;
            if isfield(values, "wStore")
                expected.wStore = false;
            end
            readback = obj.writeAndVerify( ...
                "Calibration/" + normalized, values, 5e-7, expected);
        end

        function metadata = readMetadata(obj)
            % Read converter metadata using host-friendly field names.
            raw = obj.readObject("Converter", "metadata");
            metadata = PowerTestBench.metadataFromReadback(raw);
        end

        function metadata = setMetadata(obj, settings)
            % Set and verify one or more persistent converter identity fields.
            arguments
                obj
                settings (1,1) struct
            end
            allowed = [ ...
                "boardName", "boardVersion", ...
                "serialNumber", "firmwareVersion"];
            PowerTestBench.rejectUnknownFields( ...
                settings, allowed, "metadata field");

            mappings = {
                "boardName", "wBoardName", 23;
                "boardVersion", "wBoardVersion", 15;
                "serialNumber", "wSerialNumber", 47;
                "firmwareVersion", "wFirmwareVersion", 31
            };
            values = struct();
            for i = 1:size(mappings, 1)
                source = mappings{i, 1};
                target = mappings{i, 2};
                maximumLength = mappings{i, 3};
                if isfield(settings, source)
                    values.(target) = PowerTestBench.validateMetadata( ...
                        source, settings.(source), maximumLength);
                end
            end
            if isempty(fieldnames(values))
                error("PowerTestBench:validation", ...
                    "at least one metadata field must be supplied");
            end

            raw = obj.writeAndVerify("Converter", values);
            metadata = PowerTestBench.metadataFromReadback(raw);
        end

        function powerOn(obj, leg, settings, options)
            % Safely configure and energize exactly one selected leg.
            arguments
                obj
                leg
                settings (1,1) struct = struct()
                options.ConnectDriver (1,1) logical = false
                options.ConnectCapacitor (1,1) logical = false
            end
            legNumber = PowerTestBench.normalizeLeg(leg);
            forbidden = intersect( ...
                fieldnames(settings), {'enable', 'driver', 'capacitor'});
            if ~isempty(forbidden)
                error("PowerTestBench:validation", ...
                    "powerOn controls these settings directly: %s", ...
                    strjoin(forbidden, ", "));
            end

            try
                obj.setMode(PowerTestBench.ModePowerOff);
                obj.configureLeg(1, struct("enable", false));
                obj.configureLeg(2, struct("enable", false));
                settings.enable = true;
                settings.driver = options.ConnectDriver;
                settings.capacitor = options.ConnectCapacitor;
                obj.configureLeg(legNumber, settings);
                obj.setMode(PowerTestBench.ModePowerOn);
            catch ME
                try
                    obj.shutdown();
                catch
                    % Keep the original power-on error.
                end
                rethrow(ME);
            end
        end

        function shutdown(obj, options)
            % Request POWER_OFF and disable both legs, attempting every step.
            arguments
                obj
                options.DisconnectHardware (1,1) logical = false
            end
            failures = strings(0, 1);

            try
                obj.setMode(PowerTestBench.ModePowerOff);
            catch ME
                failures(end+1) = string(ME.message);
            end

            for leg = 1:2
                settings = struct("enable", false);
                if options.DisconnectHardware
                    settings.driver = false;
                    settings.capacitor = false;
                end
                try
                    obj.configureLeg(leg, settings);
                catch ME
                    failures(end+1) = string(ME.message); %#ok<AGROW>
                end
            end

            if ~isempty(failures)
                error("PowerTestBench:shutdown", ...
                    "shutdown completed with errors: %s", ...
                    strjoin(failures, "; "));
            end
        end
    end

    methods (Access = private)
        function value = readObject(obj, path, label)
            try
                value = obj.Client.read(path);
            catch ME
                error("PowerTestBench:communication", ...
                    "ThingSet read failed at %s: %s", path, ME.message);
            end
            if ~isstruct(value) || ~isscalar(value)
                error("PowerTestBench:readback", ...
                    "%s readback is not a ThingSet object", label);
            end
        end

        function readback = writeAndVerify( ...
                obj, path, values, absoluteTolerance, expected)
            if nargin < 4
                absoluteTolerance = 0;
            end
            if nargin < 5
                expected = values;
            end

            try
                obj.Client.write(path, values);
                readback = obj.Client.read(path);
            catch ME
                error("PowerTestBench:communication", ...
                    "ThingSet operation failed at %s: %s", path, ME.message);
            end
            if ~isstruct(readback) || ~isscalar(readback)
                error("PowerTestBench:readback", ...
                    "expected object readback from %s", path);
            end

            names = fieldnames(expected);
            for i = 1:numel(names)
                name = names{i};
                if ~isfield(readback, name)
                    error("PowerTestBench:readback", ...
                        "%s/%s is missing from the firmware readback", ...
                        path, name);
                end
                if ~PowerTestBench.valuesMatch( ...
                        readback.(name), expected.(name), absoluteTolerance)
                    error("PowerTestBench:readback", ...
                        "%s/%s readback does not match the requested value", ...
                        path, name);
                end
            end
        end
    end

    methods (Static, Access = private)
        function legNumber = normalizeLeg(leg)
            if isnumeric(leg) && isscalar(leg) && isreal(leg) && ...
                    isfinite(leg) && fix(double(leg)) == double(leg) && ...
                    any(double(leg) == [1, 2])
                legNumber = double(leg);
                return
            end
            if ischar(leg) || (isstring(leg) && isscalar(leg))
                normalized = upper(string(leg));
                if normalized == "1" || normalized == "LEG1"
                    legNumber = 1;
                    return
                elseif normalized == "2" || normalized == "LEG2"
                    legNumber = 2;
                    return
                end
            end
            error("PowerTestBench:validation", ...
                "leg must be 1, 2, 'Leg1', or 'Leg2'");
        end

        function channel = normalizeChannel(channel)
            if ~(ischar(channel) || (isstring(channel) && isscalar(channel)))
                error("PowerTestBench:validation", ...
                    "channel must be V1, V2, VH, I1, I2, or IH");
            end
            channel = upper(string(channel));
            if ~any(channel == ["V1", "V2", "VH", "I1", "I2", "IH"])
                error("PowerTestBench:validation", ...
                    "channel must be V1, V2, VH, I1, I2, or IH");
            end
        end

        function modeValue = normalizeMode(mode)
            if ischar(mode) || (isstring(mode) && isscalar(mode))
                switch upper(string(mode))
                    case "IDLE"
                        modeValue = PowerTestBench.ModeIdle;
                    case "POWER_ON"
                        modeValue = PowerTestBench.ModePowerOn;
                    case "POWER_OFF"
                        modeValue = PowerTestBench.ModePowerOff;
                    otherwise
                        error("PowerTestBench:validation", ...
                            "mode must be IDLE, POWER_ON, POWER_OFF, 0, 1, or 2");
                end
                return
            end
            if isnumeric(mode) && isscalar(mode) && isreal(mode) && ...
                    isfinite(mode) && any(double(mode) == [0, 1, 2])
                modeValue = double(mode);
                return
            end
            error("PowerTestBench:validation", ...
                "mode must be IDLE, POWER_ON, POWER_OFF, 0, 1, or 2");
        end

        function value = validateLogical(name, value)
            if ~islogical(value) || ~isscalar(value)
                error("PowerTestBench:validation", "%s must be logical", name);
            end
        end

        function value = validateFinite(name, value)
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
                    ~isfinite(value)
                error("PowerTestBench:validation", ...
                    "%s must be a finite number", name);
            end
            value = double(value);
        end

        function value = validateInteger(name, value, minimum, maximum)
            value = PowerTestBench.validateFinite(name, value);
            if fix(value) ~= value || value < minimum || value > maximum
                error("PowerTestBench:validation", ...
                    "%s must be an integer between %g and %g", ...
                    name, minimum, maximum);
            end
        end

        function value = validateMetadata(name, value, maximumLength)
            if ~(ischar(value) || (isstring(value) && isscalar(value)))
                error("PowerTestBench:validation", "%s must be text", name);
            end
            value = char(string(value));
            codes = double(value);
            if isempty(value) || numel(value) > maximumLength || ...
                    any(codes < 32 | codes > 126)
                error("PowerTestBench:validation", ...
                    "%s must contain 1 to %d printable ASCII characters", ...
                    name, maximumLength);
            end
        end

        function rejectUnknownFields(settings, allowed, description)
            unknown = setdiff(string(fieldnames(settings)), string(allowed));
            if ~isempty(unknown)
                error("PowerTestBench:validation", ...
                    "unknown %s: %s", description, strjoin(unknown, ", "));
            end
        end

        function matches = valuesMatch(actual, expected, absoluteTolerance)
            if isnumeric(expected) && ~islogical(expected)
                matches = isnumeric(actual) && isscalar(actual) && ...
                    isreal(actual) && isfinite(actual) && ...
                    abs(double(actual) - double(expected)) <= ...
                    max(absoluteTolerance, abs(double(expected)) * 1e-6);
            elseif ischar(expected) || isstring(expected)
                matches = (ischar(actual) || isstring(actual)) && ...
                    isscalar(string(actual)) && ...
                    string(actual) == string(expected);
            else
                matches = isequal(actual, expected);
            end
        end

        function metadata = metadataFromReadback(raw)
            required = [ ...
                "wBoardName", "wBoardVersion", ...
                "wSerialNumber", "wFirmwareVersion"];
            missing = required(~isfield(raw, cellstr(required)));
            if ~isempty(missing)
                error("PowerTestBench:readback", ...
                    "Converter metadata is missing: %s", ...
                    strjoin(missing, ", "));
            end
            metadata = struct( ...
                "boardName", string(raw.wBoardName), ...
                "boardVersion", string(raw.wBoardVersion), ...
                "serialNumber", string(raw.wSerialNumber), ...
                "firmwareVersion", string(raw.wFirmwareVersion));
        end
    end
end
