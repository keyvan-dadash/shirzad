classdef TestQpskModulator < matlab.unittest.TestCase
    % Unit tests for modulators.QpskModulator
    methods (Test)
        function testBitLengthError(testCase)
            mod = modulators.QpskModulator();

            % 3 bits is not a multiple of 2 -> we should get error
            badBits = [0; 1; 0];

            testCase.verifyError( ...
                @() mod.modulate(badBits), ...
                'QpskModulator:BitLength', ...
                'Modulator did not throw expected length error.');
        end

        function testSingleSymbolMapping(testCase)
            mod = modulators.QpskModulator();

            % Each row is [b1 b0] (vector order)
            bitPairs = [ ...
                0 0;  % -> (+1 + 1j)/sqrt(2)
                0 1;  % -> (-1 + 1j)/sqrt(2)
                1 1;  % -> (-1 - 1j)/sqrt(2)
                1 0]; % -> (+1 - 1j)/sqrt(2)

            bits = bitPairs.';
            bits = bits(:);

            sym = mod.modulate(bits);

            expected = [ ...
                ( +1 + 1i ) / sqrt(2);  % 0 0
                ( -1 + 1i ) / sqrt(2);  % 0 1
                ( -1 - 1i ) / sqrt(2);  % 1 1
                ( +1 - 1i ) / sqrt(2)]; % 1 0

            testCase.verifySize(sym, size(expected), ...
                'Unexpected symbol vector size.');
            testCase.verifyLessThan(max(abs(sym - expected)), 1e-12, ...
                'QPSK mapping does not match expected mapping.');
        end

        function testEnergyNormalization(testCase)
            mod = modulators.QpskModulator();

            Nsym = 2000;
            bits = randi([0 1], 2*Nsym, 1);

            sym = mod.modulate(bits);

            EsHat = mean(abs(sym).^2);
            testCase.verifyLessThan(abs(EsHat - 1), 0.05, ...
                sprintf('Average symbol energy (%.3f) not close to 1.', EsHat));
        end
    end
end
