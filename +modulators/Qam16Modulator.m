classdef Qam16Modulator < modulators.AbstractModulator
    % 16-QAM modulator.
    %
    % Bits per symbol = 4:
    %   [b3 b2 b1 b0]
    %   Q uses [b3 b2], I uses [b1 b0]
    %
    %   2-bit to 4-PAM mapping (per axis):
    %       00 -> -3
    %       01 -> -1
    %       11 -> +1
    %       10 -> +3

    methods
        function obj = Qam16Modulator()
            obj@modulators.AbstractModulator(16, '16-QAM Modulator');
        end

        function symbols = modulate(obj, bits)
            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:));
            B = obj.BitsPerSymbol;   % 4

            if mod(numel(b), B) ~= 0
                error('Qam16Modulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), B);
            end

            bitsMat = reshape(b, B, []).';   % [Nsym x 4]

            % Q uses [b3 b2], I uses [b1 b0]
            qBits = bitsMat(:,1:2);
            iBits = bitsMat(:,3:4);

            I = modulators.Qam16Modulator.bits2pamGray(iBits);
            Q = modulators.Qam16Modulator.bits2pamGray(qBits);

            % nomral factor for 16-qam
            normFactor = sqrt(10);
            symbols    = (I + 1j*Q) / normFactor;
        end
    end

    methods (Static, Access = private)
        function pam = bits2pamGray(b2)
            if size(b2,2) ~= 2
                error('Qam16Modulator:bits2pamGray', ...
                      'Input must have 2 columns of bits.');
            end
            msb = b2(:,1);
            lsb = b2(:,2);
            val = msb*2 + lsb;    % 0..3

            % 00->-3, 01->-1, 11->+1, 10->+3
            map = [-3; -1; +3; +1];   % index = val+1
            pam = map(val+1);
        end
    end
end
