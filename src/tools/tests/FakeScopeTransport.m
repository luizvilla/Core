%
% Copyright (c) 2021-present LAAS-CNRS
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

classdef FakeScopeTransport < handle
    properties
        ProbeResponse = uint8(sprintf('SCOPE-DATA/1 OK\n'))
        Record = uint8.empty(1, 0)
        Input = uint8.empty(1, 0)
        Writes = {}
        Closed = false
    end

    properties (Dependent)
        NumBytesAvailable
    end

    methods
        function obj = FakeScopeTransport(record, probeResponse)
            if nargin >= 1
                obj.Record = reshape(uint8(record), 1, []);
            end
            if nargin >= 2
                obj.ProbeResponse = reshape(uint8(probeResponse), 1, []);
            end
        end

        function count = get.NumBytesAvailable(obj)
            count = numel(obj.Input);
        end

        function flush(obj, direction)
            if nargin < 2 || string(direction) == "input"
                obj.Input = uint8.empty(1, 0);
            end
        end

        function write(obj, value, ~)
            value = reshape(uint8(value), 1, []);
            obj.Writes{end+1} = value;
            if isequal(value, uint8('?'))
                obj.Input = [obj.Input, obj.ProbeResponse];
            elseif isequal(value, uint8('D'))
                obj.Input = [obj.Input, obj.Record];
            end
        end

        function value = read(obj, count, ~)
            count = min(count, numel(obj.Input));
            value = obj.Input(1:count);
            obj.Input(1:count) = [];
        end

        function close(obj)
            obj.Closed = true;
        end
    end
end
