classdef TestDecisionDirectedCarrierSync < matlab.unittest.TestCase
    % Tests for sync.DecisionDirectedCarrierSync

    methods (Test)
        function testQpskPhaseOffsetConvergence(testCase)
            % PLL should remove a constant phase offset on QPSK.

            rng(42);
            N  = 2000;  % number of symbols
            M  = 4;

            % Generate random QPSK symbols
            bits = randi([0 1], 2*N, 1);
            b1   = bits(1:2:end);   % MSB -> Q
            b0   = bits(2:2:end);   % LSB -> I

            I = 1 - 2*b0;
            Q = 1 - 2*b1;
            s = (I + 1j*Q) / sqrt(2);

            % Apply a constant phase offset
            phi0 = pi/10;  % 18 degrees
            x = s .* exp(1j*phi0);

            % Create synchronizer (coarse-ish loop bandwidth)
            syncObj = sync.DecisionDirectedCarrierSync( ...
                'ModulationOrder', 4, ...
                'SamplesPerSymbol', 1, ...
                'DampingFactor', 0.707, ...
                'NormalizedLoopBandwidth', 0.01);

            y = syncObj.process(x);

            % Compare only after some symbols (warm up loop gains)
            skip = 200;
            s_use = s(skip+1:end);
            y_use = y(skip+1:end);

            % Phase error per symbol
            phaseErr = angle(y_use .* conj(s_use));  % should be around 0

            % RMS phase error should be small
            rmsErr = sqrt(mean(phaseErr.^2));

            testCase.verifyLessThan(rmsErr, 0.1, ...
                'PLL did not remove constant phase offset.');
        end

        function testQpskSmallFrequencyOffsetConvergence(testCase)
            % PLL should track a small residual frequency offset.

            rng(43);
            N  = 4000;
            M  = 4;

            % Random QPSK symbols
            bits = randi([0 1], 2*N, 1);
            b1   = bits(1:2:end);
            b0   = bits(2:2:end);

            I = 1 - 2*b0;
            Q = 1 - 2*b1;
            s = (I + 1j*Q) / sqrt(2);

            % Apply small frequency offset + phase offset
            w0   = 0.02*pi;          % rad/sym (~3.6 deg/sym)
            phi0 = pi/4;
            n    = (0:N-1).';
            x    = s .* exp(1j*(w0*n + phi0));

            % Synchronizer with tighter loop
            syncObj = sync.DecisionDirectedCarrierSync( ...
                'ModulationOrder', 4, ...
                'SamplesPerSymbol', 1, ...
                'DampingFactor', 0.707, ...
                'NormalizedLoopBandwidth', 0.05);

            y = syncObj.process(x);

            % Ignore initial symbols (for warming up the loop gains)
            skip = 500;
            s_use = s(skip+1:end);
            y_use = y(skip+1:end);

            phaseErr = angle(y_use .* conj(s_use));
            rmsErr   = sqrt(mean(phaseErr.^2));

            testCase.verifyLessThan(rmsErr, 0.2, ...
                'PLL did not track small frequency offset.');
        end

        function testResetReproducibleOutput(testCase)
            % After reset, processing the same input should give the same output.

            rng(44);
            N  = 1000;
            M  = 4;

            % Random QPSK symbols
            bits = randi([0 1], 2*N, 1);
            b1   = bits(1:2:end);
            b0   = bits(2:2:end);

            I = 1 - 2*b0;
            Q = 1 - 2*b1;
            s = (I + 1j*Q) / sqrt(2);

            % Apply some phase+freq offset
            w0   = 0.01*pi;
            phi0 = -pi/5;
            n    = (0:N-1).';
            x    = s .* exp(1j*(w0*n + phi0));

            syncObj = sync.DecisionDirectedCarrierSync( ...
                'ModulationOrder', 4, ...
                'SamplesPerSymbol', 1, ...
                'DampingFactor', 0.707, ...
                'NormalizedLoopBandwidth', 0.01);

            % First run
            syncObj.reset();
            y1 = syncObj.process(x);

            % Second run after reset
            syncObj.reset();
            y2 = syncObj.process(x);

            testCase.verifyLessThan(max(abs(y1 - y2)), 1e-12, ...
                'Outputs differ between runs with reset and same input.');
        end
    end
end
