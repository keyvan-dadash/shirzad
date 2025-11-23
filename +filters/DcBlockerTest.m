classdef DcBlockerTest < matlab.unittest.TestCase
    % Tests for filters.DcBlocker

    methods (Test)
        function testDefaultConstructor(testCase)
            dc = filters.DcBlocker();
            testCase.verifyEqual(dc.Length, 64, ...
                'Default Length should be 64.');
        end

        function testConstantRealSignalDcRemoval(testCase)
            % A constant signal should be driven close to zero after
            % the transient, i.e., DC is removed.
            Lavg = 64;
            dc   = filters.DcBlocker('Length', Lavg);

            x = 5 * ones(2000,1);    % constant DC of 5
            y = dc.process(x);

            % Ignore initial transient, look at tail
            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(tail)), 1e-2, ...
                'DC blocker did not sufficiently remove constant real DC.');
        end

        function testConstantComplexSignalDcRemoval(testCase)
            % Same as above but with complex DC.
            Lavg = 64;
            dc   = filters.DcBlocker('Length', Lavg);

            x = (2 + 3i) * ones(2000,1);   % complex DC
            y = dc.process(x);

            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(real(tail))), 1e-2, ...
                'Real part DC not sufficiently removed for complex input.');
            testCase.verifyLessThan(max(abs(imag(tail))), 1e-2, ...
                'Imag part DC not sufficiently removed for complex input.');
        end

        function testStreamingEquivalence(testCase)
            % Because the filter keeps its internal state correctly,
            % processing in one shot or in several chunks should produce
            % identical output.
            betaLen = 64; %#ok<NASGU> % just to mirror style
            dc1 = filters.DcBlocker('Length', 64);
            dc2 = filters.DcBlocker('Length', 64);

            x = randn(1000,1) + 1j*randn(1000,1);

            % One-shot processing
            yFull = dc1.process(x);

            % Chunked processing (simulate streaming)
            y1 = dc2.process(x(1:300));
            y2 = dc2.process(x(301:700));
            y3 = dc2.process(x(701:end));
            yCat = [y1; y2; y3];

            testCase.verifyEqual(size(yCat), size(yFull), ...
                'Output sizes differ between streaming and one-shot.');

            testCase.verifyLessThan(max(abs(yFull - yCat)), 1e-12, ...
                'Streaming vs one-shot results differ for DcBlocker.');
        end

        function testRowVectorPreserved(testCase)
            % Input row vector should produce row-vector output.
            dc = filters.DcBlocker('Length', 32);
            x  = randn(1,50);
            y  = dc.process(x);

            testCase.verifyTrue(isrow(y), ...
                'DcBlocker should preserve row-vector shape.');
        end

        function testEmptyInput(testCase)
            % Empty input should be passed through unchanged.
            dc = filters.DcBlocker('Length', 32);
            x  = [];
            y  = dc.process(x);

            testCase.verifyEqual(y, x, ...
                'Empty input should result in empty output.');
        end
    end
end
