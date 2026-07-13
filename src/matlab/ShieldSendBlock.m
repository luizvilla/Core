classdef ShieldSendBlock < matlab.System
    % Simulink System object that sends REFERENCE commands to both legs of
    % the shield each sample-time step. Shares one underlying connection
    % with ShieldGetBlock via getShieldConnection/releaseShieldConnection.

    properties (Nontunable)
        SampleTime = 1
        VendorID = '2fe3'
        ProductID = '0101'
        Interactive (1, 1) logical = false
        ForcedPort = ''
    end

    properties (Access = private)
        Device
    end

    methods
        function obj = ShieldSendBlock(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Device = getShieldConnection('VendorID', obj.VendorID, ...
                'ProductID', obj.ProductID, 'Interactive', obj.Interactive, ...
                'ForcedPort', obj.ForcedPort);
        end

        function stepImpl(obj, ref1, ref2)
            obj.Device.sendCommand('REFERENCE', 'LEG1', 'V1', ref1);
            obj.Device.sendCommand('REFERENCE', 'LEG2', 'V2', ref2);
        end

        function releaseImpl(~)
            releaseShieldConnection();
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function [name, varargout] = getInputNamesImpl(~)
            name = 'Ref1';
            varargout{1} = 'Ref2';
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end

        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end
end
