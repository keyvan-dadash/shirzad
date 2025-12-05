classdef TestQam16Modulator < matlab.unittest.TestCase
    % Unit tests for modulators.Qam16Modulator

    methods (Test)
        function testFullConstellationMapping(testCase)
            mod = modulators.Qam16Modulator();

            bitQuads = de2bi(0:15, 4, 'left-msb');   % [16 x 4]

            bits = bitQuads.';
            bits = bits(:);

            sym = mod.modulate(bits);

            qBits = bitQuads(:, 1:2);   % [b3 b2]
            iBits = bitQuads(:, 3:4);   % [b1 b0]

            I = modulators.TestQam16Modulator.bits2pamGrayLocal(iBits);
            Q = modulators.TestQam16Modulator.bits2pamGrayLocal(qBits);

            normFactor = sqrt(10);
            expected   = (I + 1i*Q) / normFactor;

            testCase.verifySize(sym, size(expected), ...
                'Unexpected symbol vector size.');
            testCase.verifyLessThan(max(abs(sym - expected)), 1e-12, ...
                '16-QAM mapping does not match expected mapping.');
        end
    end

    methods (Static, Access = private)
        function pam = bits2pamGrayLocal(b2)
            if size(b2,2) ~= 2
                error('TestQam16Modulator:bits2pamGrayLocal', ...
                      'Input must have 2 columns of bits.');
            end
            msb = b2(:,1);
            lsb = b2(:,2);
            val = msb*2 + lsb;      % 0..3

            map = [-3; -1; +3; +1];
            pam = map(val+1);
        end
    end
end
