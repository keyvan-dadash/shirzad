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
            fdc   = filters.FastDcBlocker('Length', Lavg);


            % Simple dc blocker test
            x = 5 * ones(2000,1);    % constant DC of 5
            y = dc.process(x);

            % Ignore initial state
            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(tail)), 1e-2, ...
                'DC blocker did not sufficiently remove constant real DC.');


            % Fast dc blocker test
            x = 5 * ones(2000,1);    % constant DC of 5
            y = fdc.process(x);

            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(tail)), 1e-2, ...
                'Fast DC blocker did not sufficiently remove constant real DC.');

        end

        function testConstantComplexSignalDcRemoval(testCase)
            % Same as the above but with complex DC.
            Lavg = 64;
            dc   = filters.DcBlocker('Length', Lavg);
            fdc   = filters.FastDcBlocker('Length', Lavg);

            % Simple Dcblocker
            x = (2 + 3i) * ones(2000,1);
            y = dc.process(x);

            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(real(tail))), 1e-2, ...
                'Real part DC not sufficiently removed for complex input.');
            testCase.verifyLessThan(max(abs(imag(tail))), 1e-2, ...
                'Imag part DC not sufficiently removed for complex input.');

            % Fast Dcblocker
            x = (2 + 3i) * ones(2000,1);
            y = fdc.process(x);

            tail = y(end-200:end);
            testCase.verifyLessThan(max(abs(real(tail))), 1e-2, ...
                'Real part DC not sufficiently removed for complex input by fast dcblocker.');
            testCase.verifyLessThan(max(abs(imag(tail))), 1e-2, ...
                'Imag part DC not sufficiently removed for complex input by fast dcblocker.');
        end

        function testPerformanceLargeBlock(testCase)
            dc = filters.DcBlocker('Length', 2048);
            fdc = filters.FastDcBlocker('Length', 2048);
    
            N = 30000;
            x = randn(N,1) + 1j*randn(N,1);
    
            Niter = 200;
            t0 = tic;
            for k = 1:Niter
                y = dc.process(x);
            end
            tAvg = toc(t0)/Niter;

            t1 = tic;
            for k = 1:Niter
                y = fdc.process(x);
            end
            tAvg1 = toc(t1)/Niter;

            fprintf('DcBlocker: N=%d, avg time = %.3f µs\n', ...
                    N, 1e6*tAvg);
            fprintf('Fast DcBlocker: N=%d, avg time = %.3f µs\n', ...
                    N, 1e6*tAvg1);
        end
    end
end
