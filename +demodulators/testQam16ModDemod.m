classdef testQam16ModDemod < matlab.unittest.TestCase
    methods (Test)
        function testSymbolMappingTable(testCase)
            mod = modulators.Qam16Modulator();
            dem = demodulators.Qam16Demodulator();

            B = mod.BitsPerSymbol;

            bitsAll = de2bi(0:15, B, 'left-msb');
            bitsIn  = bitsAll.';
            bitsIn  = bitsIn(:);

            syms = mod.modulate(bitsIn);

            symsUnique = unique(syms);
            testCase.verifyEqual(numel(symsUnique), 16, ...
                'Expected 16 unique constellation points for 16-QAM.');

            bitsOut = dem.demodulateHard(syms);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Modulator/Demodulator mapping is inconsistent.');
        end

        function testIndividualSymbolKnownMapping(testCase)
            mod = modulators.Qam16Modulator();

            function checkPattern(b3b2b1b0, expectedSym)
                bits = b3b2b1b0(:);
                sym  = mod.modulate(bits);
                testCase.verifyEqual(sym, expectedSym, 'AbsTol', 1e-12, ...
                    sprintf('Mapping mismatch for bits [%d %d %d %d].', ...
                            b3b2b1b0(1), b3b2b1b0(2), ...
                            b3b2b1b0(3), b3b2b1b0(4)));
            end

            nf = sqrt(10);  % normalization

            checkPattern([0;0;0;0], (-3 - 3j)/nf);

            checkPattern([0;1;0;0], (-3 - 1j)/nf);

            checkPattern([1;1;1;1], (1 + 1j)/nf);

            checkPattern([1;0;1;0], (3 + 3j)/nf);
        end

        function testRandomRoundTripNoNoise(testCase)
            mod = modulators.Qam16Modulator();
            dem = demodulators.Qam16Demodulator();

            B = mod.BitsPerSymbol;
            Nsym = 1000;

            bitsIn = randi([0 1], Nsym * B, 1);

            syms   = mod.modulate(bitsIn);
            bitsOut = dem.demodulateHard(syms);

            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Random round-trip bits mismatch with no noise.');
        end

        function testResolvePhaseAmbiguityNoNoise(testCase)
            mod = modulators.Qam16Modulator();
            dem = demodulators.Qam16Demodulator();

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
                'getAmbiguityRotations must return at least one rotation.');

            kRot   = randi(nRot);
            rotRef = G(kRot);

            rxSymsEq = syms * rotRef; 

            [rxSymsFixed, bestIdx, errs] = dem.resolvePhaseAmbiguity(rxSymsEq, pilotBits);

            netRot = rotRef * G(bestIdx);
            testCase.verifyLessThan(abs(angle(netRot)), 1e-6, ...
                'Net rotation after ambiguity resolution is not close to zero phase.');

            testCase.verifyLessThan(errs(bestIdx), 1e-6, ...
                'Pilot BER for best rotation should be essentially zero.');

            bitsOut = dem.demodulateHard(rxSymsFixed);
            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Bits do not match after phase-ambiguity resolution.');
        end
    end
end
