%
% Copyright (c) 2021-present LAAS-CNRS
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

classdef FakeThingSetClient < handle
    properties
        State
        Operations = {}
        FailWrites
        RestoreValues
    end

    methods
        function obj = FakeThingSetClient()
            obj.State = containers.Map( ...
                "KeyType", "char", "ValueType", "any");
            obj.FailWrites = containers.Map( ...
                "KeyType", "char", "ValueType", "double");
            obj.RestoreValues = containers.Map( ...
                "KeyType", "char", "ValueType", "any");

            leg = struct( ...
                "wEnable", false, "wCapa", false, "wDriver", false, ...
                "wBuck", false, "wBoost", false, "wDutyCycle", 0.1, ...
                "wReferenceValue", 0.0, "wTrackingVar", "V1", ...
                "wPhaseShift", 0, "wDeadTimeRising_ns", 100, ...
                "wDeadTimeFalling_ns", 100);
            calibration = struct( ...
                "wGain", 1.0, "wOffset", 0.0, "wStore", false);

            obj.State('Config') = struct( ...
                "Mode", 0, "Frequency_Hz", 200000);
            obj.State('Config/Leg1') = leg;
            obj.State('Config/Leg2') = leg;
            obj.State('Measurements') = struct( ...
                "rV1Low_V", 12.0, "rDuty1", 0.25);
            channels = ["V1", "V2", "VH", "I1", "I2", "IH"];
            for channel = channels
                obj.State(char("Calibration/" + channel)) = calibration;
            end
            obj.State('Converter') = struct( ...
                "wBoardName", "TWIST", ...
                "wBoardVersion", "v1.4.2", ...
                "wSerialNumber", "UNSET", ...
                "wFirmwareVersion", "1.0.0");
        end

        function value = read(obj, path)
            key = char(path);
            obj.Operations{end+1} = struct( ...
                "Method", "read", "Path", string(path));
            value = obj.State(key);
        end

        function write(obj, path, values)
            key = char(path);
            obj.Operations{end+1} = struct( ...
                "Method", "write", "Path", string(path), ...
                "Values", values);

            if isKey(obj.FailWrites, key) && obj.FailWrites(key) > 0
                obj.FailWrites(key) = obj.FailWrites(key) - 1;
                error("FakeThingSetClient:injected", ...
                    "injected write failure at %s", path);
            end

            current = obj.State(key);
            names = fieldnames(values);
            for i = 1:numel(names)
                name = names{i};
                current.(name) = values.(name);
            end
            if isfield(values, "wStore") && values.wStore
                current.wStore = false;
            end
            for i = 1:numel(names)
                name = names{i};
                restoreKey = [key '|' name];
                if isKey(obj.RestoreValues, restoreKey)
                    current.(name) = obj.RestoreValues(restoreKey);
                end
            end
            obj.State(key) = current;
        end

        function setWriteFailure(obj, path, count)
            obj.FailWrites(char(path)) = count;
        end

        function setRestoreValue(obj, path, name, value)
            obj.RestoreValues([char(path) '|' char(name)]) = value;
        end

        function value = getState(obj, path)
            value = obj.State(char(path));
        end
    end
end
