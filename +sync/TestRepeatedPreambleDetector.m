classdef TestRepeatedPreambleDetector < matlab.unittest.TestCase
    % Unit tests for sync.RepeatedPreambleDetector

    methods (Test)
        function testPerfectPreamble_sps1(testCase)
            sps = 1;
            Lh  = 8;     % half-preamble length
            Lpre = 2*Lh;

            det = sync.RepeatedPreambleDetector( ...
                'SamplesPerSymbol', sps, ...
                'PreambleHalfLen',  Lh, ...
                'MetricThreshold',  0.2, ...
                'MinWindowPower',   1e-6);

            rng(1234);
            preHalf = exp(1j*2*pi*rand(Lh,1));  % arbitrary complex a
            pre     = [preHalf; preHalf];       % [a, a]

            % Put preamble at a known symbol index
            preStartSym = 5;

            % Build symbol-rate sequence with zeros before/after
            NsymBefore = preStartSym - 1;
            NsymAfter  = 20;
            ySym = [zeros(NsymBefore,1); pre; zeros(NsymAfter,1)];

            % sps = 1 => sample-rate sequence is same as symbol-rate
            y = ySym;

            res = det.detect(y);

            testCase.verifyTrue(res.Found, ...
                'Detector failed to find a perfect repeated preamble.');
            testCase.verifyEqual(res.SampleOffset, 0, ...
                'SampleOffset must be 0 for sps=1.');
            testCase.verifyEqual(res.PreambleStartSym, preStartSym, ...
                'PreambleStartSym does not match known location.');

            testCase.verifyGreaterThan(res.Metric, 0.2);
            testCase.verifyLessThan(abs(res.Metric - 0.25), 1e-3, ...
                sprintf('Metric for ideal preamble should be ~0.25, got %.4f.', res.Metric));

            testCase.verifyGreaterThan(res.WindowPower, 0, ...
                'WindowPower for ideal preamble should be > 0.');
        end

        function testPerfectPreambleWithSampleOffset(testCase)
            sps = 4;
            Lh  = 8;
            Lpre = 2*Lh;

            det = sync.RepeatedPreambleDetector( ...
                'SamplesPerSymbol', sps, ...
                'PreambleHalfLen',  Lh, ...
                'MetricThreshold',  0.2, ...
                'MinWindowPower',   1e-8);

            rng(1234);
            preHalf = exp(1j*2*pi*rand(Lh,1));
            pre     = [preHalf; preHalf];

            preStartSym = 7;   % symbol index at 1-sps
            NsymBefore  = preStartSym - 1;
            NsymAfter   = 15;

            ySymAll = [zeros(NsymBefore,1); pre; zeros(NsymAfter,1)];
            Ns      = numel(ySymAll);

            % True sample offset (0 .. sps-1)
            offTrue = 2;

            Nsamples = offTrue + (Ns-1)*sps + 1;
            y = zeros(Nsamples,1);
            for k = 1:Ns
                idx = offTrue + (k-1)*sps + 1;
                y(idx) = ySymAll(k);
            end

            res = det.detect(y);

            testCase.verifyTrue(res.Found, ...
                'Detector did not find preamble with non-zero sample offset.');
            testCase.verifyEqual(res.SampleOffset, offTrue, ...
                'Recovered SampleOffset does not match true offset.');
            testCase.verifyEqual(res.PreambleStartSym, preStartSym, ...
                'Recovered PreambleStartSym does not match the know start.');

            testCase.verifyGreaterThan(res.Metric, 0.2, ...
                'Metric too low for a clean repeated preamble.');
        end

        function testNoiseOnlyNoDetection(testCase)
            sps = 4;
            Lh  = 16;

            det = sync.RepeatedPreambleDetector( ...
                'SamplesPerSymbol', sps, ...
                'PreambleHalfLen',  Lh, ...
                'MetricThreshold',  1.1, ...  % impossible to reach
                'MinWindowPower',   1e-6);

            rng(42);
            N = 5000;
            noise = (randn(N,1) + 1j*randn(N,1))/sqrt(2);

            res = det.detect(noise);

            testCase.verifyFalse(res.Found, ...
                'Detector should not declare a preamble in pure noise with impossible threshold.');
        end

        function testVectorizedDetectorLongSequence(testCase)
            % Compare original RepeatedPreambleDetector with the
            % vectorized schmidlCoxDetectFast on a long sequence

            sps  = 10;
            Lh   = 128;
            Lpre = 2*Lh;
            metricThresh = 0.2;
            minPow       = 1e-7;

            det = sync.RepeatedPreambleDetector( ...
                'SamplesPerSymbol', sps, ...
                'PreambleHalfLen',  Lh, ...
                'MetricThreshold',  metricThresh, ...
                'MinWindowPower',   minPow);

            detf = sync.RepeatedPreambleDetector( ...
                'SamplesPerSymbol', sps, ...
                'PreambleHalfLen',  Lh, ...
                'MetricThreshold',  metricThresh, ...
                'MinWindowPower',   minPow);

            rng(2025);
            preHalf = exp(1j*2*pi*rand(Lh,1));
            pre     = [preHalf; preHalf];

            preStartSym = 100;
            NsymBefore  = preStartSym - 1;
            NsymAfter   = 300;

            ySym = [zeros(NsymBefore,1); pre; zeros(NsymAfter,1)];
            NsSym = numel(ySym);

            offTrue = 3;

            Nsamples = offTrue + (NsSym-1)*sps + 1;
            y = zeros(Nsamples,1);

            for k = 1:NsSym
                idx = offTrue + (k-1)*sps + 1;
                y(idx) = ySym(k);
            end

            tTotal = 0;
            for k = 1:100
                t1 = tic;
                resOrig = det.detect(y);
                tOrig = toc(t1);
                tTotal = tTotal + tOrig;
            end

            tOrig = tTotal / 100;

            tTotal = 0;
            for k = 1:100
                t1 = tic;
                resFast = det.detectFast(y);
                tFast = toc(t1);
                tTotal = tTotal + tFast;
            end

            tFast = tTotal / 100;

            fprintf('\nLong-sequence Schmidl-Cox detector timing (Nsamples=%d):\n', numel(y));
            fprintf('  RepeatedPreambleDetector : %.6f s\n', tOrig);
            fprintf('  schmidlCoxDetectFast     : %.6f s\n', tFast);

            % --- Correctness checks ---
            testCase.verifyTrue(resFast.Found, ...
                'Vectorized detector failed to find preamble.');

            testCase.verifyEqual(resFast.SampleOffset, offTrue, ...
                'Vectorized detector SampleOffset mismatch.');

            testCase.verifyEqual(resFast.PreambleStartSym, preStartSym, ...
                'Vectorized detector PreambleStartSym mismatch.');

            % If original detector also found something, check consistency
            if resOrig.Found
                testCase.verifyEqual(resOrig.SampleOffset, offTrue, ...
                    'Original detector SampleOffset mismatch on long sequence.');
                testCase.verifyEqual(resOrig.PreambleStartSym, preStartSym, ...
                    'Original detector PreambleStartSym mismatch on long sequence.');

                % CFO estimates should be close (here CFO is 0)
                testCase.verifyLessThan( ...
                    abs(resFast.CfoRadPerSym - resOrig.CfoRadPerSym), 1e-3, ...
                    'CFO estimates differ too much between detectors.');
            end
        end
    end
end
