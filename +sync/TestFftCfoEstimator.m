classdef TestFftCfoEstimator < matlab.unittest.TestCase
    % Unit tests for sync.FftCfoEstimator

    methods (Test)
        function testNoHistoryExactBin(testCase)
            % No history, no noise, CFO exactly on an FFT bin.
            % The estimator should recover CFO within a tiny tolerance.

            rng(1);  %#ok<RAND> make deterministic

            Rsym = 100e3;      % symbol rate [Hz]
            Nfft = 1024;       % FFT length
            Lpre = 128;        % preamble length [symbols]

            % Build a random QPSK-like preamble (unit-ish magnitude)
            bits = randi([0 1], 2*Lpre, 1);
            b0 = bits(1:2:end);       % LSB -> I
            b1 = bits(2:2:end);       % MSB -> Q
            I  = 1 - 2*b0;
            Q  = 1 - 2*b1;
            preSyms = (I + 1j*Q) / sqrt(2);

            % Choose CFO exactly on an FFT bin (positive frequency)
            kBin    = 12;                         % bin index (0-centered)
            df      = Rsym / Nfft;                % Hz per bin
            fTrueHz = kBin * df;                  % true CFO in Hz
            wTrue   = 2*pi * fTrueHz / Rsym;      % rad/sym

            n = (0:Lpre-1).';                     % symbol indices
            rPreamble = preSyms .* exp(1j*wTrue*n);

            % Put preamble after some leading symbols to test preStartSym
            leadSyms    = 5;
            ySym        = [zeros(leadSyms,1); rPreamble; zeros(10,1)];
            preStartSym = leadSyms + 1;

            est = sync.FftCfoEstimator( ...
                'SampleRateSym', Rsym, ...
                'PreambleSyms',  preSyms, ...
                'Nfft',          Nfft, ...
                'UseHistory',    false);

            [wHat, fHatHz, peakVal] = est.estimate(ySym, preStartSym); %#ok<ASGLU>

            % Peak must be non-zero
            testCase.verifyGreaterThan(peakVal, 0, ...
                'FFT peak magnitude should be > 0.');

            % CFO should be very close to the true CFO
            tolHz = df * 1e-3;  % much smaller than a bin width
            testCase.verifyLessThan(abs(fHatHz - fTrueHz), tolHz, ...
                sprintf('Estimated CFO %g Hz differs from true %g Hz.', ...
                        fHatHz, fTrueHz));
        end

        function testInsufficientSymbolsReturnsZero(testCase)
            % If there are not enough symbols after preStartSym to cover
            % the preamble, estimator should return zeros.

            Rsym = 100e3;
            Nfft = 512;
            Lpre = 64;

            % Simple preamble (doesn't matter much here)
            preSyms = ones(Lpre,1);

            est = sync.FftCfoEstimator( ...
                'SampleRateSym', Rsym, ...
                'PreambleSyms',  preSyms, ...
                'Nfft',          Nfft, ...
                'UseHistory',    false);

            % ySym shorter than needed
            ySym = randn(40,1) + 1j*randn(40,1); % 40 < Lpre

            [wHat, fHatHz, peakVal] = est.estimate(ySym, 1);

            testCase.verifyEqual(wHat, 0, ...
                'wSym should be zero when there are not enough symbols.');
            testCase.verifyEqual(fHatHz, 0, ...
                'fCfoHz should be zero when there are not enough symbols.');
            testCase.verifyEqual(peakVal, 0, ...
                'peakVal should be zero when there are not enough symbols.');
        end

        function testHistoryJumpLimiting(testCase)
            % In history mode with Alpha=1, a large CFO jump between frames
            % should be limited by MaxJumpHz.

            rng(2); %#ok<RAND>

            Rsym = 100e3;
            Nfft = 1024;
            Lpre = 128;

            % Build a random QPSK-like preamble
            bits = randi([0 1], 2*Lpre, 1);
            b0 = bits(1:2:end);
            b1 = bits(2:2:end);
            I  = 1 - 2*b0;
            Q  = 1 - 2*b1;
            preSyms = (I + 1j*Q) / sqrt(2);

            % Choose two CFOs on exact FFT bins
            df      = Rsym / Nfft;
            k1      = 4;                 % first frame CFO bin
            k2      = 20;                % second frame CFO bin
            f1True  = k1 * df;
            f2True  = k2 * df;

            MaxJumpHz = 1000;           % allowed jump
            Alpha     = 1.0;            % no smoothing, pure jump-limit

            est = sync.FftCfoEstimator( ...
                'SampleRateSym', Rsym, ...
                'PreambleSyms',  preSyms, ...
                'Nfft',          Nfft, ...
                'UseHistory',    true, ...
                'NumCandidates', 1, ...
                'Alpha',         Alpha, ...
                'MaxJumpHz',     MaxJumpHz);

            est.resetHistory();

            n = (0:Lpre-1).';

            % ----- Frame 1: CFO = f1True -----
            w1 = 2*pi * f1True / Rsym;
            y1 = preSyms .* exp(1j*w1*n);

            [~, f1Hat, ~] = est.estimate(y1, 1);

            % f1Hat should be close to f1True
            tol1 = df * 1e-3;
            testCase.verifyLessThan(abs(f1Hat - f1True), tol1, ...
                'First-frame CFO estimate is not close to true value.');

            % ----- Frame 2: CFO = f2True (big jump) -----
            w2 = 2*pi * f2True / Rsym;
            y2 = preSyms .* exp(1j*w2*n);

            [~, f2Hat, ~] = est.estimate(y2, 1);

            % The instantaneous jump is |f2True - f1True| > MaxJumpHz,
            % so with Alpha=1 we expect f2Hat ≈ f1Hat + sign*MaxJumpHz.
            expectedF2 = f1Hat + sign(f2True - f1True) * MaxJumpHz;

            tol2 = 1e-3;  % Hz (tight, logic is purely arithmetic)
            testCase.verifyLessThan(abs(f2Hat - expectedF2), tol2, ...
                sprintf(['History/jump limiting failed: f2Hat=%g, ' ...
                         'expected about %g (Hz).'], f2Hat, expectedF2));
        end
    end
end
