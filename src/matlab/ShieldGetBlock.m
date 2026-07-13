classdef ShieldGetBlock < matlab.System
    % Simulink System object that reads V1/V2 measurements from the shield
    % each sample-time step. Shares one underlying connection with
    % ShieldSendBlock via getShieldConnection/releaseShieldConnection.

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
        function obj = ShieldGetBlock(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Device = getShieldConnection('VendorID', obj.VendorID, ...
                'ProductID', obj.ProductID, 'Interactive', obj.Interactive, ...
                'ForcedPort', obj.ForcedPort);
        end

        function [v1, v2] = stepImpl(obj)
            v1 = obj.Device.getMeasurement('V1');
            v2 = obj.Device.getMeasurement('V2');
        end

        function releaseImpl(~)
            releaseShieldConnection();
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function [name, varargout] = getOutputNamesImpl(~)
            name = 'V1';
            varargout{1} = 'V2';
        end

        % Simulink's automatic output-property inference tries to
        % statically analyze stepImpl via code generation regardless of
        % the "Simulate using" setting, and fails because
        % getShieldConnection uses inputParser (not codegen-compatible).
        % Declaring these explicitly skips that inference entirely.
        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end

        function [cplx1, cplx2] = isOutputComplexImpl(~)
            cplx1 = false;
            cplx2 = false;
        end

        function [fixed1, fixed2] = isOutputFixedSizeImpl(~)
            fixed1 = true;
            fixed2 = true;
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
