%
% Copyright (c) 2021-present LAAS-CNRS
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

classdef ScopeSerial < handle
    % ScopeSerial  Bounded parser for the dedicated scope-data interface.

    properties (SetAccess = private)
        Port
    end

    properties (Access = private)
        Serial
    end

    properties (Constant)
        BaudRate = 115200
        SampleCount = 1024
        ChannelNames = [ ...
            "V1Low_V", "V2Low_V", "VHigh_V", "I1Low_A", ...
            "I2Low_A", "IHigh_A", "Duty1", "Duty2"]
    end

    methods
        function obj = ScopeSerial(port, options)
            arguments
                port (1,1) string
                options.Timeout (1,1) double = 1.0
                options.ProbeTimeout (1,1) double = 3.0
                options.Transport = []
            end
            if strlength(port) == 0
                error("ScopeSerial:validation", ...
                    "scope port must be an explicit non-empty string");
            end
            ScopeSerial.validatePositiveFinite("Timeout", options.Timeout);
            ScopeSerial.validatePositiveFinite( ...
                "ProbeTimeout", options.ProbeTimeout);

            obj.Port = port;
            try
                if isempty(options.Transport)
                    obj.Serial = serialport( ...
                        port, ScopeSerial.BaudRate, ...
                        "Timeout", options.Timeout);
                else
                    if ~ismethod(options.Transport, "read") || ...
                            ~ismethod(options.Transport, "write")
                        error("ScopeSerial:validation", ...
                            "Transport must provide read and write methods");
                    end
                    obj.Serial = options.Transport;
                end
                obj.probe(options.ProbeTimeout);
            catch ME
                obj.close();
                if startsWith(string(ME.identifier), "ScopeSerial:")
                    rethrow(ME);
                end
                error("ScopeSerial:transport", ...
                    "failed to open scope data port %s: %s", ...
                    port, ME.message);
            end
        end

        function close(obj)
            if ~isempty(obj.Serial) && ismethod(obj.Serial, "close")
                obj.Serial.close();
            end
            obj.Serial = [];
        end

        function delete(obj)
            obj.close();
        end

        function capture = download(obj, metadata, options)
            % Request, validate, decode, and rotate a frozen capture.
            arguments
                obj
                metadata (1,1) struct
                options.Timeout (1,1) double = 15.0
            end
            ScopeSerial.validatePositiveFinite("Timeout", options.Timeout);
            metadata = ScopeSerial.validateMetadata(metadata);

            obj.writeCommand(uint8('D'));
            started = tic;
            foundStart = false;
            for i = 1:16
                line = obj.readLine(started, options.Timeout, 256);
                if line == "begin record"
                    foundStart = true;
                    break
                end
                if startsWith(line, "SCOPE-DATA/1 ERROR")
                    error("ScopeSerial:protocol", "%s", line);
                end
            end
            if ~foundStart
                error("ScopeSerial:protocol", ...
                    "scope record start marker was not received");
            end

            channelLine = obj.readLine(started, options.Timeout, 256);
            if ~startsWith(channelLine, "#")
                error("ScopeSerial:protocol", ...
                    "scope channel header is malformed");
            end
            channelNames = split(extractAfter(channelLine, 1), ",").';
            channelNames(channelNames == "") = [];
            if ~isequal(channelNames, ScopeSerial.ChannelNames)
                error("ScopeSerial:protocol", ...
                    "scope channel header mismatch");
            end

            finalLine = obj.readLine(started, options.Timeout, 256);
            token = regexp(finalLine, '^# ([0-9]+)$', 'tokens', 'once');
            if isempty(token)
                error("ScopeSerial:protocol", ...
                    "scope final-index line is malformed");
            end
            finalIndex = str2double(token{1});
            if finalIndex < 0 || finalIndex >= ScopeSerial.SampleCount
                error("ScopeSerial:protocol", ...
                    "scope final index is out of range");
            end

            channelCount = numel(ScopeSerial.ChannelNames);
            valueCount = ScopeSerial.SampleCount * channelCount;
            values = zeros(valueCount, 1);
            for i = 1:valueCount
                line = obj.readLine(started, options.Timeout, 16);
                if line == "end record"
                    error("ScopeSerial:protocol", ...
                        "scope payload is truncated at %d/%d values", ...
                        i - 1, valueCount);
                end
                if isempty(regexp(line, '^[0-9A-Fa-f]{8}$', 'once'))
                    error("ScopeSerial:protocol", ...
                        "scope payload value %d is not 8-digit hexadecimal", ...
                        i - 1);
                end
                bits = uint32(hex2dec(char(line)));
                values(i) = double(typecast(bits, 'single'));
            end

            terminator = obj.readLine(started, options.Timeout, 16);
            if terminator ~= "end record"
                if ~isempty(regexp(terminator, ...
                        '^[0-9A-Fa-f]{8}$', 'once'))
                    error("ScopeSerial:protocol", ...
                        "scope payload contains more than %d values", ...
                        valueCount);
                end
                error("ScopeSerial:protocol", ...
                    "scope record end marker is malformed");
            end

            rows = reshape(values, channelCount, ScopeSerial.SampleCount).';
            firstIndex = mod(finalIndex + 1, ScopeSerial.SampleCount);
            order = [ ...
                (firstIndex + 1):ScopeSerial.SampleCount, ...
                1:firstIndex];
            samples = rows(order, :);

            triggerOffset = ...
                metadata.pretriggerRatio * ScopeSerial.SampleCount;
            timeAxisS = ( ...
                (0:(ScopeSerial.SampleCount - 1)).' - triggerOffset) * ...
                metadata.samplePeriodUs * 1e-6;

            capture = struct( ...
                "decimation", metadata.decimation, ...
                "samplePeriodUs", metadata.samplePeriodUs, ...
                "durationMs", metadata.durationMs, ...
                "channelNames", channelNames, ...
                "samples", samples, ...
                "finalIndex", finalIndex, ...
                "pretriggerRatio", metadata.pretriggerRatio, ...
                "timeAxisS", timeAxisS);
        end
    end

    methods (Access = private)
        function writeCommand(obj, command)
            try
                if ismethod(obj.Serial, "flush")
                    flush(obj.Serial, "input");
                end
                write(obj.Serial, command, "uint8");
            catch ME
                if startsWith(string(ME.identifier), "ScopeSerial:")
                    rethrow(ME);
                end
                error("ScopeSerial:transport", ...
                    "scope data-port write failed: %s", ME.message);
            end
        end

        function line = readLine(obj, started, timeout, maximumLength)
            bytes = uint8.empty(1, 0);
            while toc(started) < timeout
                try
                    if obj.Serial.NumBytesAvailable == 0
                        pause(0.001);
                        continue
                    end
                    value = read(obj.Serial, 1, "uint8");
                catch ME
                    error("ScopeSerial:transport", ...
                        "scope data-port read failed: %s", ME.message);
                end
                if isempty(value)
                    continue
                end
                bytes(end+1) = uint8(value); %#ok<AGROW>
                if numel(bytes) > maximumLength
                    error("ScopeSerial:protocol", ...
                        "scope line exceeds %d bytes", maximumLength);
                end
                if bytes(end) == 10
                    while ~isempty(bytes) && ...
                            any(bytes(end) == uint8([10, 13]))
                        bytes(end) = [];
                    end
                    if any(bytes > 127)
                        error("ScopeSerial:protocol", ...
                            "scope response is not ASCII");
                    end
                    line = string(char(bytes));
                    return
                end
            end
            error("ScopeSerial:timeout", ...
                "scope data-port response timed out");
        end

        function probe(obj, timeout)
            obj.writeCommand(uint8('?'));
            started = tic;
            for i = 1:32
                line = obj.readLine(started, timeout, 256);
                if line == "SCOPE-DATA/1 OK"
                    return
                end
                if startsWith(line, "SCOPE-DATA/1 ERROR")
                    error("ScopeSerial:probe", ...
                        "scope probe rejected: %s", line);
                end
            end
            error("ScopeSerial:probe", ...
                "scope probe response was not received");
        end
    end

    methods (Static, Access = private)
        function validatePositiveFinite(name, value)
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
                    ~isfinite(value) || value <= 0
                error("ScopeSerial:validation", ...
                    "%s must be a positive finite number", name);
            end
        end

        function metadata = validateMetadata(metadata)
            required = [ ...
                "decimation", "samplePeriodUs", ...
                "durationMs", "pretriggerRatio"];
            if ~all(isfield(metadata, cellstr(required)))
                error("ScopeSerial:validation", ...
                    "capture metadata is incomplete");
            end

            decimation = metadata.decimation;
            if ~isnumeric(decimation) || islogical(decimation) || ...
                    ~isscalar(decimation) || ~isreal(decimation) || ...
                    ~isfinite(decimation) || fix(decimation) ~= decimation || ...
                    decimation < 1 || decimation > 100
                error("ScopeSerial:validation", ...
                    "decimation must be an integer between 1 and 100");
            end
            samplePeriodUs = metadata.samplePeriodUs;
            if ~isnumeric(samplePeriodUs) || islogical(samplePeriodUs) || ...
                    ~isscalar(samplePeriodUs) || ...
                    samplePeriodUs ~= 100 * decimation
                error("ScopeSerial:validation", ...
                    "samplePeriodUs must equal 100 times decimation");
            end
            durationMs = metadata.durationMs;
            expectedDuration = ...
                ScopeSerial.SampleCount * samplePeriodUs / 1000;
            if ~isnumeric(durationMs) || islogical(durationMs) || ...
                    ~isscalar(durationMs) || ~isreal(durationMs) || ...
                    ~isfinite(durationMs) || ...
                    abs(durationMs - expectedDuration) > 0.05
                error("ScopeSerial:validation", ...
                    "durationMs does not match the capture timing");
            end
            pretriggerRatio = metadata.pretriggerRatio;
            if ~isnumeric(pretriggerRatio) || islogical(pretriggerRatio) || ...
                    ~isscalar(pretriggerRatio) || ...
                    ~isreal(pretriggerRatio) || ...
                    ~isfinite(pretriggerRatio) || ...
                    pretriggerRatio < 0 || pretriggerRatio > 0.9
                error("ScopeSerial:validation", ...
                    "pretriggerRatio must be between 0.0 and 0.9");
            end

            metadata = struct( ...
                "decimation", double(decimation), ...
                "samplePeriodUs", double(samplePeriodUs), ...
                "durationMs", double(durationMs), ...
                "pretriggerRatio", double(pretriggerRatio));
        end
    end
end
