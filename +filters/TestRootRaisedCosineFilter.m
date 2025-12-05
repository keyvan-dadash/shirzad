classdef TestRootRaisedCosineFilter < matlab.unittest.TestCase
    % Tests for filters.RootRaisedCosineFilter

    methods (Test)
        function testLengthAndSymmetry(testCase)
            beta = 0.35;
            span = 10;
            sps  = 8;

            L = span * sps + 1;

            rrc = filters.RootRaisedCosineFilter(beta, span, sps, false);

            % Impulse response
            x = [1; zeros(L-1,1)];
            y = rrc.process(x);
            h = y(:).';   % row

            % length matches spec
            testCase.verifyEqual(numel(h), L, ...
                'RRC length != span*sps+1');

            % symmetry (real, even)
            testCase.verifyLessThan( ...
                max(abs(h - fliplr(h))), 1e-10, ...
                'RRC impulse response is not symmetric.');
        end

        function testEnergyNormalization(testCase)
            beta = 0.25;
            span = 8;
            sps  = 4;
            L    = span * sps + 1;

            rrc = filters.RootRaisedCosineFilter(beta, span, sps, true);

            x = [1; zeros(L-1,1)];
            y = rrc.process(x);
            h = y(:).';

            E = sum(abs(h).^2);
            testCase.verifyLessThan(abs(E - 1), 1e-10, ...
                'Energy-normalized RRC does not have unit energy.');
        end

        function testMatchesReferenceFormula(testCase)
            % Checking the implementation of RRC
            beta = 0.35;
            span = 6;
            sps  = 5;
            L    = span * sps + 1;

            rrc = filters.RootRaisedCosineFilter(beta, span, sps, false);

            % Impulse response
            x = [1; zeros(L-1,1)];
            y = rrc.process(x);
            h = y(:).';

            % Refrences
            n  = -(L-1)/2 : (L-1)/2;
            t  = n / sps;
            Ts = 1;

            hRef = zeros(size(t));
            tol  = 1e-8;

            % t == 0
            idx0 = abs(t) < tol;
            if any(idx0)
                hRef(idx0) = (1 + beta*(4/pi - 1));
            end

            % t == (+-) Ts/(4*beta)
            t1 = Ts / (4*beta);
            idx1 = abs(abs(t) - t1) < tol;
            if any(idx1)
                hRef(idx1) = (beta / sqrt(2)) * ...
                    ( (1 + 2/pi) * sin(pi/(4*beta)) + ...
                      (1 - 2/pi) * cos(pi/(4*beta)) );
            end

            idxGen = ~(idx0 | idx1);
            tg = t(idxGen);
            if ~isempty(tg)
                num = sin(pi * tg * (1 - beta)) + ...
                      4 * beta .* tg .* cos(pi * tg * (1 + beta));
                den = pi * tg .* (1 - (4*beta.*tg).^2);
                hRef(idxGen) = num ./ den;
            end

            % Normalize for fair comparison
            hNorm    = h / norm(h);
            hRefNorm = hRef / norm(hRef);

            testCase.verifyLessThan( ...
                max(abs(hNorm - hRefNorm)), 1e-10, ...
                'RRC impulse shape does not match reference formula.');
        end

        function testStreamingEquivalence(testCase)
            % Due to statefull nature of the RRC filter, feeding data in
            % an one-shot should be equal to giving it in a multiple shots.
            beta = 0.35;
            span = 10;
            sps  = 4;
            L    = span * sps + 1;

            xFull = [1; zeros(L-1,1)];

            % One-shot
            rrc1 = filters.RootRaisedCosineFilter(beta, span, sps, false);
            yFull = rrc1.process(xFull);

            % Chunked (simulate streaming)
            rrc2 = filters.RootRaisedCosineFilter(beta, span, sps, false);
            yPart1 = rrc2.process(xFull(1:10));
            yPart2 = rrc2.process(xFull(11:end));
            yCat   = [yPart1; yPart2];

            % Compare first L samples
            yFullL = yFull(1:L);
            yCatL  = yCat(1:L);

            testCase.verifyLessThan( ...
                max(abs(yFullL - yCatL)), 1e-12, ...
                'Streaming vs one-shot processing differs for RRC filter.');
        end

        function testPerformanceLargeInput(testCase)
            % Measure runtime of RRC filter for large inputs.
            % We test sps = 4 and 8 with 30000 samples and print
            % the elapsed time in microseconds.

            beta = 0.35;
            span = 10;
            spsList = [4 8];
            N = 30000;

            x = randn(N, 1);

            % Make sure it is single
            x = single(x);

            for sps = spsList
                rrc = filters.RootRaisedCosineFilter(beta, span, sps, false);

                % Make sure state is clean
                rrc.reset();

                tStart = tic;
                y = rrc.process(x);
                elapsed = toc(tStart);           % seconds
                us = elapsed * 1e6;              % microseconds

                fprintf(['RootRaisedCosineFilter: sps = %d, N = %d -> ' ...
                         '%.3f us (%.3f ms)\n'], ...
                        sps, N, us, elapsed * 1e3);

                testCase.verifyEqual(numel(y), N, ...
                    sprintf('Unexpected output length for sps = %d', sps));
            end
        end
    end
end
