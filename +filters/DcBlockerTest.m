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
            % some time.
            Lavg = 64;
            dc   = filters.DcBlocker('Length', Lavg);

            x = 5 * ones(2000,1);    % constant DC of 5
            y = dc.process(x);

            % Ignore initial state
            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(tail)), 1e-2, ...
                'DC blocker did not sufficiently remove constant real DC.');
        end

        function testConstantComplexSignalDcRemoval(testCase)
            % Same as the above but with complex DC.
            Lavg = 64;
            dc   = filters.DcBlocker('Length', Lavg);

            x = (2 + 3i) * ones(2000,1);
            y = dc.process(x);

            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(real(tail))), 1e-2, ...
                'Real part DC not sufficiently removed for complex input.');
            testCase.verifyLessThan(max(abs(imag(tail))), 1e-2, ...
                'Imag part DC not sufficiently removed for complex input.');
        end
    end
end
