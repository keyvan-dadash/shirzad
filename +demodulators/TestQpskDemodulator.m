classdef TestQpskDemodulator < matlab.unittest.TestCase

    methods (Test)
        function testHardDecisionIdealConstellation(testCase)
            dem = demodulators.QpskDemodulator();

            const = (1/sqrt(2)) * [ ...
                1 + 1j;   % 00
               -1 + 1j;   % 01
               -1 - 1j;   % 11
                1 - 1j];  % 10

            bitsExpected = [ ...
                0; 0;  ... % 00
                0; 1;  ... % 01
                1; 1;  ... % 11
                1; 0];     % 10

            bits = dem.demodulateHard(const);
            bits = double(bits);

            testCase.verifyEqual(bits, bitsExpected, ...
                'Hard decisions are wrong.');
        end

        function testHardDecisionMappingVectorRandom(testCase)
            dem = demodulators.QpskDemodulator();

            % Random sequence bits
            N = 200;
            bitsIn = randi([0 1], 2*N, 1);

            % Map each bits to different cells
            b1 = bitsIn(1:2:end);
            b0 = bitsIn(2:2:end);

            % flip real if the bit is equal to 1
            I = ones(N,1);
            I(b0 == 1) = -1;

            % flip imagainry if the bit is equal to 1
            Q = ones(N,1);
            Q(b1 == 1) = -1;

            syms = (I + 1j*Q) / sqrt(2);

            bitsOut = dem.demodulateHard(syms);
            bitsOut = double(bitsOut);

            testCase.verifyEqual(bitsOut, bitsIn, ...
                'Random mapping is wrong.');
        end

        function testHardDecisionScaleInvariance(testCase)
            dem = demodulators.QpskDemodulator();

            const = (1/sqrt(2)) * [ ...
                1 + 1j;   % 00
               -1 + 1j;   % 01
               -1 - 1j;   % 11
                1 - 1j];  % 10

            bitsIdeal = dem.demodulateHard(const);

            scaleFactors = [0.5, 1, 2, 10];
            for a = scaleFactors
                symsScaled = a * const;
                bitsScaled = dem.demodulateHard(symsScaled);
                testCase.verifyEqual(bitsScaled, bitsIdeal, ...
                    sprintf('Hard decisions shouldnt change under scaling a=%.2f', a));
            end
        end
    end
end
