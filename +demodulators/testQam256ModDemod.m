classdef testQam256ModDemod < matlab.unittest.TestCase
    methods (Test)
        function testSymbolMappingTable(testCase)
            mod = modulators.Qam256Modulator();
            dem = demodulators.CPPQam256Demodulator();

            B = mod.BitsPerSymbol;

            bitsAll = de2bi(0:255, B, 'left-msb');  % [256 x 8]
            bitsIn  = bitsAll.';                    % [8 x 256]
            bitsIn  = bitsIn(:);                    % [2048 x 1]

            syms = mod.modulate(bitsIn);

            symsUnique = unique(syms);
            testCase.verifyEqual(numel(symsUnique), 256, ...
                'Expected 256 unique constellation points for 256-QAM.');

            syms = single(syms);
            bitsOut = dem.demodulateHard(syms);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Modulator/Demodulator mapping is inconsistent for 256-QAM.');
        end

        function testIndividualSymbolKnownMapping(testCase)
            mod = modulators.Qam256Modulator();

            function checkPattern(b7b6b5b4b3b2b1b0, expectedSym)
                bits = b7b6b5b4b3b2b1b0(:);
                sym  = mod.modulate(bits);
                testCase.verifyEqual(sym, expectedSym, 'AbsTol', 1e-12, ...
                    sprintf('Mapping mismatch for bits [%d %d %d %d %d %d %d %d].', ...
                            b7b6b5b4b3b2b1b0(1), b7b6b5b4b3b2b1b0(2), ...
                            b7b6b5b4b3b2b1b0(3), b7b6b5b4b3b2b1b0(4), ...
                            b7b6b5b4b3b2b1b0(5), b7b6b5b4b3b2b1b0(6), ...
                            b7b6b5b4b3b2b1b0(7), b7b6b5b4b3b2b1b0(8)));
            end

            nf = sqrt(170);  % normalization for 256-QAM

            % Bits: [b7 b6 b5 b4 b3 b2 b1 b0]
            % Q uses [b7 b6 b5 b4], I uses [b3 b2 b1 b0]
            % 4-bit → 16-PAM:
            %   0000 -> -15
            %   0001 -> -13
            %   0010 -> -11
            %   0011 -> -9
            %   0100 -> -7
            %   0101 -> -5
            %   0110 -> -3
            %   0111 -> -1
            %   1000 -> +15
            %   1001 -> +13
            %   1010 -> +11
            %   1011 -> +9
            %   1100 -> +7
            %   1101 -> +5
            %   1110 -> +3
            %   1111 -> +1

            % Corner: (-15 - 15j)
            checkPattern([0;0;0;0;0;0;0;0], (-15 - 15j)/nf);

            % Next point in I: (-13 - 15j)
            checkPattern([0;0;0;0;0;0;0;1], (-13 - 15j)/nf);

            % Center-ish: (1 + 1j)
            checkPattern([1;1;1;1;1;1;1;1], (1 + 1j)/nf);

            % Corner: (15 + 15j)
            checkPattern([1;0;0;0;1;0;0;0], (15 + 15j)/nf);
        end

        function testRandomRoundTripNoNoise(testCase)
            mod = modulators.Qam256Modulator();
            dem = demodulators.CPPQam256Demodulator();

            B    = mod.BitsPerSymbol;
            Nsym = 1000;

            bitsIn = randi([0 1], Nsym * B, 1);

            syms    = mod.modulate(bitsIn);
            syms = single(syms);
            bitsOut = dem.demodulateHard(syms);

            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Random round-trip bits mismatch with no noise for 256-QAM.');
        end

        function testResolvePhaseAmbiguityNoNoise(testCase)
            mod = modulators.Qam256Modulator();
            dem = demodulators.CPPQam256Demodulator();

            B    = mod.BitsPerSymbol;
            Nsym = 200;  % number of symbols in this "frame"

            bitsIn = randi([0 1], Nsym * B, 1);
            syms   = mod.modulate(bitsIn);
            syms = single(syms);

            % Use first part of bits as pilot bits
            pilotBitsLen = 60;
            pilotBitsLen = min(pilotBitsLen, numel(bitsIn));
            pilotBitsLen = B * floor(pilotBitsLen / B);
            pilotBits    = bitsIn(1:pilotBitsLen);

            G = dem.getAmbiguityRotations();
            G = G(:).';
            nRot = numel(G);
            testCase.verifyGreaterThanOrEqual(nRot, 1, ...
                'getAmbiguityRotations must return at least one rotation (256-QAM).');

            kRot   = randi(nRot);
            rotRef = G(kRot);

            rxSymsEq = syms * rotRef;

            [rxSymsFixed, bestIdx, errs] = dem.resolvePhaseAmbiguity(rxSymsEq, pilotBits);

            netRot = rotRef * G(bestIdx);
            testCase.verifyLessThan(abs(angle(netRot)), 1e-6, ...
                'Net rotation after ambiguity resolution is not close to zero phase (256-QAM).');

            testCase.verifyLessThan(errs(bestIdx), 1e-6, ...
                'Pilot BER for best rotation should be essentially zero (256-QAM).');

            bitsOut = dem.demodulateHard(rxSymsFixed);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Bits do not match after phase-ambiguity resolution for 256-QAM.');
        end
    end
end
