classdef ShieldDevice < handle
    % Serial protocol client for a Twist/Ownverter shield, mirroring
    % Shield_Class.py from the python_twist_comm_protocol library.

    properties
        SerialObj
        MessageIndex
        MessageLength = 16
    end

    methods
        function obj = ShieldDevice(port, varargin)
            p = inputParser;
            addRequired(p, 'port');
            addParameter(p, 'BaudRate', 115200);
            addParameter(p, 'DataBits', 8);
            addParameter(p, 'Parity', 'none');
            addParameter(p, 'StopBits', 1);
            addParameter(p, 'Timeout', 2);
            parse(p, port, varargin{:});

            obj.SerialObj = serialport(port, p.Results.BaudRate, ...
                'DataBits', p.Results.DataBits, ...
                'Parity', p.Results.Parity, ...
                'StopBits', p.Results.StopBits, ...
                'Timeout', p.Results.Timeout);
            configureTerminator(obj.SerialObj, "LF");

            obj.MessageIndex = containers.Map( ...
                {'D1','V1','I1','M1','T1','D2','I2','V2','M2','T2','VH','IH','AN','CE','CR','RS'}, ...
                {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15});
        end

        function sendMessage(obj, message)
            chunkSize = 10;
            numChunks = ceil(length(message) / chunkSize);
            for i = 1:numChunks
                startIdx = (i - 1) * chunkSize + 1;
                endIdx = min(i * chunkSize, length(message));
                write(obj.SerialObj, message(startIdx:endIdx), "char");
                pause(0.1);
            end
            write(obj.SerialObj, sprintf('\r\n'), "char");
        end

        function message = sendCommand(obj, action, varargin)
            delay = 0.2;
            if numel(varargin) >= 2 && (ischar(varargin{end - 1}) || isstring(varargin{end - 1})) ...
                    && strcmpi(varargin{end - 1}, 'Delay')
                delay = varargin{end};
                varargin(end - 1:end) = [];
            end

            switch action
                case 'IDLE'
                    message = 'd_i';
                case 'POWER_OFF'
                    message = 'd_f';
                case 'POWER_ON'
                    message = 'd_o';
                case 'LEG'
                    message = sprintf('s_%s_l_%s', upper(varargin{1}), lower(varargin{2}));
                case 'CAPA'
                    message = sprintf('s_%s_c_%s', upper(varargin{1}), lower(varargin{2}));
                case 'DRIVER'
                    message = sprintf('s_%s_v_%s', upper(varargin{1}), lower(varargin{2}));
                case 'BUCK'
                    message = sprintf('s_%s_b_%s', upper(varargin{1}), lower(varargin{2}));
                case 'BOOST'
                    message = sprintf('s_%s_t_%s', upper(varargin{1}), lower(varargin{2}));
                case 'REFERENCE'
                    message = sprintf('s_%s_r_%s_%.5f', upper(varargin{1}), upper(varargin{2}), varargin{3});
                case 'DUTY'
                    message = sprintf('s_%s_d_%.5f', upper(varargin{1}), varargin{2});
                case 'CALIBRATE'
                    message = sprintf('k_%s_g_%.8f_o_%.8f', upper(varargin{1}), varargin{2}, varargin{3});
                otherwise
                    error('ShieldDevice:InvalidAction', 'Invalid action: %s', action);
            end

            obj.sendMessage(message);
            pause(delay);
        end

        function fields = getLine(obj)
            fields = split(readline(obj.SerialObj), ':');
        end

        function value = getMeasurement(obj, measurementType)
            if ~isKey(obj.MessageIndex, measurementType)
                error('ShieldDevice:InvalidMeasurement', ...
                    'Invalid measurement type: %s', measurementType);
            end
            index = obj.MessageIndex(measurementType) + 1;

            flush(obj.SerialObj, "input");

            sizeOk = false;
            while ~sizeOk
                fields = obj.getLine();
                fields = erase(fields, {'{', '}'});
                if numel(fields) == obj.MessageLength
                    sizeOk = true;
                end
            end

            value = str2double(fields(index));
        end
    end
end
