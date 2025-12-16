classdef testQam64ModDemod < matlab.unittest.TestCase
    methods (Test)
        function testSymbolMappingTable(testCase)
            mod = modulators.Qam64Modulator();
            dem = demodulators.CPPQam64Demodulator();

            B = mod.BitsPerSymbol;

            bitsAll = de2bi(0:63, B, 'left-msb');
            bitsIn  = bitsAll.';
            bitsIn  = bitsIn(:);

            syms = mod.modulate(bitsIn);

            symsUnique = unique(syms);
            testCase.verifyEqual(numel(symsUnique), 64, ...
                'Expected 64 unique constellation points for 64-QAM.');

            syms = single(syms);
            bitsOut = dem.demodulateHard(syms);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Modulator/Demodulator mapping is inconsistent for 64-QAM.');
        end

        function testIndividualSymbolKnownMapping(testCase)
            mod = modulators.Qam64Modulator();

            function checkPattern(b5b4b3b2b1b0, expectedSym)
                bits = b5b4b3b2b1b0(:);
                sym  = mod.modulate(bits);
                testCase.verifyEqual(sym, expectedSym, 'AbsTol', 1e-12, ...
                    sprintf('Mapping mismatch for bits [%d %d %d %d %d %d].', ...
                            b5b4b3b2b1b0(1), b5b4b3b2b1b0(2), ...
                            b5b4b3b2b1b0(3), b5b4b3b2b1b0(4), ...
                            b5b4b3b2b1b0(5), b5b4b3b2b1b0(6)));
            end

            nf = sqrt(42);  % normalization for 64-QAM

            % Bits: [b5 b4 b3 b2 b1 b0]
            % Q uses [b5 b4 b3], I uses [b2 b1 b0]
            % 3-bit → 8-PAM:
            %   000 -> -7
            %   001 -> -5
            %   010 -> -3
            %   011 -> -1
            %   100 -> +7
            %   101 -> +5
            %   110 -> +3
            %   111 -> +1

            % Corner: (-7 - 7j)
            checkPattern([0;0;0;0;0;0], (-7 - 7j)/nf);

            % Next point in I: (-5 - 7j)
            checkPattern([0;0;0;0;0;1], (-5 - 7j)/nf);

            % Center-ish: (1 + 1j)
            checkPattern([1;1;1;1;1;1], (1 + 1j)/nf);

            % Corner: (7 + 7j)
            checkPattern([1;0;0;1;0;0], (7 + 7j)/nf);
        end

        function testRandomRoundTripNoNoise(testCase)
            mod = modulators.Qam64Modulator();
            dem = demodulators.CPPQam64Demodulator();

            B    = mod.BitsPerSymbol;
            Nsym = 1000;

            bitsIn = randi([0 1], Nsym * B, 1);

            syms    = mod.modulate(bitsIn);
            syms = single(syms);
            bitsOut = dem.demodulateHard(syms);

            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Random round-trip bits mismatch with no noise for 64-QAM.');
        end

        function testResolvePhaseAmbiguityNoNoise(testCase)
            mod = modulators.Qam64Modulator();
            dem = demodulators.CPPQam64Demodulator();

            B    = mod.BitsPerSymbol;
            Nsym = 200;  % number of symbols in this "frame"

            bitsIn = randi([0 1], Nsym * B, 1);
            syms   = mod.modulate(bitsIn);

            % Use first part of bits as pilot bits
            pilotBitsLen = 60;
            pilotBitsLen = min(pilotBitsLen, numel(bitsIn));
            pilotBitsLen = B * floor(pilotBitsLen / B);
            pilotBits    = bitsIn(1:pilotBitsLen);

            G = dem.getAmbiguityRotations();
            G = G(:).';
            nRot = numel(G);
            testCase.verifyGreaterThanOrEqual(nRot, 1, ...
                'getAmbiguityRotations must return at least one rotation (64-QAM).');

            kRot   = randi(nRot);
            rotRef = G(kRot);

            rxSymsEq = syms * rotRef;

            rxSymsEq = single(rxSymsEq);
            [rxSymsFixed, bestIdx, errs] = dem.resolvePhaseAmbiguity(rxSymsEq, pilotBits);

            netRot = rotRef * G(bestIdx);
            testCase.verifyLessThan(abs(angle(netRot)), 1e-6, ...
                'Net rotation after ambiguity resolution is not close to zero phase (64-QAM).');

            testCase.verifyLessThan(errs(bestIdx), 1e-6, ...
                'Pilot BER for best rotation should be essentially zero (64-QAM).');

            bitsOut = dem.demodulateHard(rxSymsFixed);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Bits do not match after phase-ambiguity resolution for 64-QAM.');
        end
    end
end
