%
% Copyright (c) 2021-present LAAS-CNRS
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

classdef test_PowerTestBench < matlab.unittest.TestCase
    properties
        Client
        Bench
    end

    methods (TestMethodSetup)
        function createBench(testCase)
            testCase.Client = FakeThingSetClient();
            testCase.Bench = PowerTestBench(testCase.Client);
        end
    end

    methods (Test)
        function modeAndFrequencyAreWrittenAndVerified(testCase)
            testCase.verifyEqual( ...
                testCase.Bench.setMode("POWER_OFF"), ...
                PowerTestBench.ModePowerOff);
            testCase.verifyEqual( ...
                testCase.Bench.setFrequency(100000), 100000);
            config = testCase.Client.getState("Config");
            testCase.verifyEqual(config.Mode, 2);
            testCase.verifyEqual(config.Frequency_Hz, 100000);
        end

        function configureLegMapsAndValidatesFields(testCase)
            settings = struct( ...
                "boost", true, "dutyCycle", 0.25, ...
                "referenceValue", 24, "trackingVar", "vh", ...
                "phaseShift", -90, "deadTimeRisingNs", 120, ...
                "deadTimeFallingNs", 130);
            result = testCase.Bench.configureLeg("Leg2", settings);
            testCase.verifyTrue(result.wBoost);
            testCase.verifyFalse(result.wBuck);
            testCase.verifyEqual(string(result.wTrackingVar), "VH");

            testCase.verifyError( ...
                @() testCase.Bench.configureLeg( ...
                    1, struct("dutyCycle", 1.01)), ...
                "PowerTestBench:validation");
            testCase.verifyError( ...
                @() testCase.Bench.configureLeg( ...
                    1, struct("buck", true, "boost", true)), ...
                "PowerTestBench:validation");
        end

        function restoredFirmwareValueRaisesReadbackError(testCase)
            testCase.Client.setRestoreValue( ...
                "Config/Leg1", "wDutyCycle", 0.1);
            testCase.verifyError( ...
                @() testCase.Bench.configureLeg( ...
                    1, struct("dutyCycle", 0.5)), ...
                "PowerTestBench:readback");
        end

        function calibrationAndMetadataRoundTrip(testCase)
            calibration = testCase.Bench.setCalibration( ...
                "i1", struct( ...
                    "gain", 2.0, "offset", -0.25, "store", true));
            testCase.verifyEqual(calibration.wGain, 2.0);
            testCase.verifyFalse(calibration.wStore);

            metadata = testCase.Bench.setMetadata( ...
                struct("boardName", "OWNVERTER", ...
                       "serialNumber", "SN-123"));
            testCase.verifyEqual(metadata.boardName, "OWNVERTER");
            testCase.verifyEqual(metadata.serialNumber, "SN-123");
            testCase.verifyEqual(testCase.Bench.readMetadata(), metadata);

            testCase.verifyError( ...
                @() testCase.Bench.setMetadata( ...
                    struct("boardVersion", "")), ...
                "PowerTestBench:validation");
        end

        function powerOnUsesSafeSequence(testCase)
            testCase.Bench.powerOn( ...
                2, struct("dutyCycle", 0.2), ConnectDriver=true);
            leg1 = testCase.Client.getState("Config/Leg1");
            leg2 = testCase.Client.getState("Config/Leg2");
            config = testCase.Client.getState("Config");
            testCase.verifyFalse(leg1.wEnable);
            testCase.verifyTrue(leg2.wEnable);
            testCase.verifyTrue(leg2.wDriver);
            testCase.verifyEqual(config.Mode, PowerTestBench.ModePowerOn);

            writes = testCase.Client.Operations(cellfun( ...
                @(operation) operation.Method == "write", ...
                testCase.Client.Operations));
            testCase.verifyEqual(writes{1}.Values.Mode, ...
                PowerTestBench.ModePowerOff);
            testCase.verifyEqual(writes{end}.Values.Mode, ...
                PowerTestBench.ModePowerOn);
        end

        function powerOnFailureAttemptsShutdown(testCase)
            testCase.Client.setWriteFailure("Config/Leg2", 1);
            testCase.verifyError( ...
                @() testCase.Bench.powerOn( ...
                    1, struct(), ConnectDriver=true), ...
                "PowerTestBench:communication");
            config = testCase.Client.getState("Config");
            leg1 = testCase.Client.getState("Config/Leg1");
            leg2 = testCase.Client.getState("Config/Leg2");
            testCase.verifyEqual(config.Mode, PowerTestBench.ModePowerOff);
            testCase.verifyFalse(leg1.wEnable);
            testCase.verifyFalse(leg2.wEnable);
        end

        function shutdownAttemptsAllLegs(testCase)
            testCase.Client.setWriteFailure("Config", 1);
            testCase.Client.setWriteFailure("Config/Leg1", 1);
            testCase.verifyError( ...
                @() testCase.Bench.shutdown(DisconnectHardware=true), ...
                "PowerTestBench:shutdown");
            leg2 = testCase.Client.getState("Config/Leg2");
            testCase.verifyFalse(leg2.wEnable);
            testCase.verifyFalse(leg2.wDriver);
            testCase.verifyFalse(leg2.wCapa);
        end

        function scopeControlSequenceAndDownload(testCase)
            scopeClient = FakeScopeClient();
            bench = PowerTestBench(testCase.Client, scopeClient);

            armed = bench.armScope( ...
                PretriggerRatio=0.25, Decimation=10);
            testCase.verifyEqual(armed.state, "ARMED");
            testCase.verifyEqual(armed.samplePeriodUs, 1000);
            testCase.verifyEqual(armed.durationMs, 1024);

            triggered = bench.triggerScope();
            testCase.verifyEqual(triggered.state, "TRIGGERED");
            raw = testCase.Client.getState("Debug/Scope");
            raw.rState = 3;
            raw.rFinalIndex = 17;
            testCase.Client.setState("Debug/Scope", raw);

            ready = bench.waitScopeReady( ...
                Timeout=0.1, PollInterval=0.001);
            testCase.verifyEqual(ready.state, "READY");
            capture = bench.downloadScope(Timeout=4);
            testCase.verifyEqual(capture.decimation, 10);
            testCase.verifySize(capture.samples, [1024, 8]);
            testCase.verifyEqual( ...
                scopeClient.Calls{1}.metadata.pretriggerRatio, 0.25);
            testCase.verifyEqual(scopeClient.Calls{1}.timeout, 4);
        end

        function scopeValidationAndRestoredWrite(testCase)
            invalid = {false, 1.5, 0, 101, NaN, Inf};
            for i = 1:numel(invalid)
                value = invalid{i};
                testCase.verifyError( ...
                    @() testCase.Bench.armScope(Decimation=value), ...
                    "PowerTestBench:validation");
            end
            for value = [1, 10, 100]
                status = testCase.Bench.armScope(Decimation=value);
                testCase.verifyEqual(status.activeDecimation, value);
            end
            invalidPretrigger = {-0.1, 0.91, NaN, Inf};
            for i = 1:numel(invalidPretrigger)
                value = invalidPretrigger{i};
                testCase.verifyError( ...
                    @() testCase.Bench.armScope( ...
                        PretriggerRatio=value), ...
                    "PowerTestBench:validation");
            end

            testCase.Client.setRestoreValue( ...
                "Debug/Scope", "wDecimation", 1);
            testCase.verifyError( ...
                @() testCase.Bench.armScope(Decimation=10), ...
                "PowerTestBench:readback");
        end

        function scopeRequiresTransportAndReadyState(testCase)
            testCase.verifyError( ...
                @() testCase.Bench.downloadScope(), ...
                "PowerTestBench:scopeTransport");

            bench = PowerTestBench(testCase.Client, FakeScopeClient());
            testCase.verifyError( ...
                @() bench.downloadScope(), ...
                "PowerTestBench:scopeState");
        end

        function scopeStatusRejectsMetadataMismatch(testCase)
            raw = testCase.Client.getState("Debug/Scope");
            raw.rChannelCount = 7;
            testCase.Client.setState("Debug/Scope", raw);
            testCase.verifyError( ...
                @() testCase.Bench.readScopeStatus(), ...
                "PowerTestBench:readback");
        end
    end
end
