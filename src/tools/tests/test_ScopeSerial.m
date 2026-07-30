%
% Copyright (c) 2021-present LAAS-CNRS
%
% SPDX-License-Identifier: GPL-2.0-or-later
%

classdef test_ScopeSerial < matlab.unittest.TestCase
    methods (Test)
        function explicitProbeAndCleanup(testCase)
            testCase.verifyError( ...
                @() ScopeSerial(""), "ScopeSerial:validation");

            transport = FakeScopeTransport( ...
                uint8.empty(1, 0), uint8(sprintf('WRONG PORT\n')));
            testCase.verifyError( ...
                @() ScopeSerial( ...
                    "fake", Transport=transport, ProbeTimeout=0.01), ...
                "ScopeSerial:timeout");
            testCase.verifyTrue(transport.Closed);
        end

        function validCaptureIsDecodedAndRotated(testCase)
            transport = FakeScopeTransport( ...
                test_ScopeSerial.makeRecord());
            scope = ScopeSerial("fake", Transport=transport);
            capture = scope.download(test_ScopeSerial.metadata(10, 0.2));

            testCase.verifyEqual(transport.Writes, ...
                {uint8('?'), uint8('D')});
            testCase.verifyEqual( ...
                capture.channelNames, ScopeSerial.ChannelNames);
            testCase.verifyEqual(capture.finalIndex, 1022);
            testCase.verifyEqual(capture.samples(1, 1), 10230);
            testCase.verifyEqual(capture.samples(2, 1), 0);
            testCase.verifySize(capture.samples, [1024, 8]);
            testCase.verifyEqual( ...
                capture.timeAxisS(1), -0.2048, AbsTol=1e-12);
            scope.close();
            testCase.verifyTrue(transport.Closed);
        end

        function malformedPayloadsAreRejected(testCase)
            records = {
                test_ScopeSerial.makeRecord(ValueCount=8191), ...
                test_ScopeSerial.makeRecord(CorruptValue=123), ...
                test_ScopeSerial.makeRecord(ValueCount=8193)
            };
            for i = 1:numel(records)
                transport = FakeScopeTransport(records{i});
                scope = ScopeSerial("fake", Transport=transport);
                testCase.verifyError( ...
                    @() scope.download(test_ScopeSerial.metadata(1, 0)), ...
                    "ScopeSerial:protocol");
                scope.close();
            end
        end

        function wrongChannelAndFinalIndexAreRejected(testCase)
            channels = ScopeSerial.ChannelNames;
            channels(1) = "wrong";
            transport = FakeScopeTransport( ...
                test_ScopeSerial.makeRecord(Channels=channels));
            scope = ScopeSerial("fake", Transport=transport);
            testCase.verifyError( ...
                @() scope.download(test_ScopeSerial.metadata(1, 0)), ...
                "ScopeSerial:protocol");
            scope.close();

            record = test_ScopeSerial.makeRecord();
            recordText = string(char(record));
            recordText = replace(recordText, "# 1022" + newline, ...
                "## 1022" + newline);
            transport = FakeScopeTransport(uint8(char(recordText)));
            scope = ScopeSerial("fake", Transport=transport);
            testCase.verifyError( ...
                @() scope.download(test_ScopeSerial.metadata(1, 0)), ...
                "ScopeSerial:protocol");
            scope.close();
        end

        function notReadyTimeoutAndLineBoundAreRejected(testCase)
            transport = FakeScopeTransport(uint8(sprintf( ...
                'SCOPE-DATA/1 ERROR NOT_READY ARMED\n')));
            scope = ScopeSerial("fake", Transport=transport);
            testCase.verifyError( ...
                @() scope.download(test_ScopeSerial.metadata(1, 0)), ...
                "ScopeSerial:protocol");
            scope.close();

            transport = FakeScopeTransport(uint8.empty(1, 0));
            scope = ScopeSerial("fake", Transport=transport);
            testCase.verifyError( ...
                @() scope.download( ...
                    test_ScopeSerial.metadata(1, 0), Timeout=0.01), ...
                "ScopeSerial:timeout");
            scope.close();

            transport = FakeScopeTransport( ...
                uint8([repmat('x', 1, 257), newline]));
            scope = ScopeSerial("fake", Transport=transport);
            testCase.verifyError( ...
                @() scope.download(test_ScopeSerial.metadata(1, 0)), ...
                "ScopeSerial:protocol");
            scope.close();
        end

        function metadataValidation(testCase)
            transport = FakeScopeTransport( ...
                test_ScopeSerial.makeRecord());
            scope = ScopeSerial("fake", Transport=transport);
            invalid = {
                struct("decimation", false, "samplePeriodUs", 100, ...
                    "durationMs", 102.4, "pretriggerRatio", 0), ...
                test_ScopeSerial.metadata(0, 0), ...
                test_ScopeSerial.metadata(101, 0), ...
                struct("decimation", 10, "samplePeriodUs", 100, ...
                    "durationMs", 102.4, "pretriggerRatio", 0), ...
                struct("decimation", 1, "samplePeriodUs", 100, ...
                    "durationMs", NaN, "pretriggerRatio", 0), ...
                struct("decimation", 1, "samplePeriodUs", 100, ...
                    "durationMs", 102.4, "pretriggerRatio", Inf)
            };
            for i = 1:numel(invalid)
                metadata = invalid{i};
                testCase.verifyError( ...
                    @() scope.download(metadata), ...
                    "ScopeSerial:validation");
            end
            scope.close();
        end
    end

    methods (Static, Access = private)
        function metadata = metadata(decimation, pretriggerRatio)
            metadata = struct( ...
                "decimation", decimation, ...
                "samplePeriodUs", 100 * decimation, ...
                "durationMs", 102.4 * decimation, ...
                "pretriggerRatio", pretriggerRatio);
        end

        function record = makeRecord(options)
            arguments
                options.FinalIndex = 1022
                options.Channels = ScopeSerial.ChannelNames
                options.ValueCount = 8192
                options.CorruptValue = -1
                options.Terminator = "end record"
            end
            lines = cell(options.ValueCount + 4, 1);
            lines{1} = 'begin record';
            lines{2} = ['#' char(strjoin(options.Channels, ',')) ','];
            lines{3} = sprintf('# %d', options.FinalIndex);
            for index = 0:(options.ValueCount - 1)
                if index == options.CorruptValue
                    lines{index + 4} = 'not-hex!';
                else
                    sample = floor(index / 8);
                    channel = mod(index, 8);
                    bits = typecast(single(sample * 10 + channel), 'uint32');
                    lines{index + 4} = sprintf('%08x', bits);
                end
            end
            lines{end} = char(options.Terminator);
            record = uint8(char(strjoin(string(lines), newline) + newline));
        end
    end
end
