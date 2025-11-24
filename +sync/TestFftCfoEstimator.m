classdef TestFftCfoEstimator < matlab.unittest.TestCase
    % Unit tests for sync.FftCfoEstimator

    methods (Test)
        function testNoHistoryExactBin(testCase)
            % No history, no noise, CFO exactly on an FFT bin.
            % The estimator should recover CFO within a tiny tolerance.

            rng(1);

            Rsym = 100e3; % symbol rate
            Nfft = 1024; % FFT lenght
            Lpre = 128; % Preamble lenght

            % Building qpsk like data
            bits = randi([0 1], 2*Lpre, 1);
            b0 = bits(1:2:end);
            b1 = bits(2:2:end);
            I  = 1 - 2*b0;
            Q  = 1 - 2*b1;
            preSyms = (I + 1j*Q) / sqrt(2);

            kBin    = 12;
            df      = Rsym / Nfft;
            fTrueHz = kBin * df;
            wTrue   = 2*pi * fTrueHz / Rsym;

            n = (0:Lpre-1).'; % symbols
            rPreamble = preSyms .* exp(1j*wTrue*n);

            % Lets put some garbage symbols
            leadSyms    = 5;
            ySym        = [zeros(leadSyms,1); rPreamble; zeros(10,1)];
            preStartSym = leadSyms + 1;

            est = sync.FftCfoEstimator( ...
                'SampleRateSym', Rsym, ...
                'PreambleSyms',  preSyms, ...
                'Nfft',          Nfft);

            [wHat, fHatHz, peakVal] = est.estimate(ySym, preStartSym); %#ok<ASGLU>

            % Peak must be non-zero
            testCase.verifyGreaterThan(peakVal, 0, ...
                'FFT peak magnitude should be > 0.');

            % CFO should be very close to the true CFO
            tolHz = df * 1e-3;
            testCase.verifyLessThan(abs(fHatHz - fTrueHz), tolHz, ...
                sprintf('Estimated CFO %g Hz differs from true %g Hz.', ...
                        fHatHz, fTrueHz));
        end
    end
end
