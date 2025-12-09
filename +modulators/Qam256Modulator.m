classdef Qam256Modulator < modulators.AbstractModulator
    % 256-QAM modulator.
    %
    % Bits per symbol = 8:
    %   [b7 b6 b5 b4 b3 b2 b1 b0]
    %   Q uses [b7 b6 b5 b4], I uses [b3 b2 b1 b0]
    %
    % 4-bit to 16-PAM Gray mapping per axis:
    %   0000 -> -15
    %   0001 -> -13
    %   0011 -> -11
    %   0010 -> -9
    %   0110 -> -7
    %   0111 -> -5
    %   0101 -> -3
    %   0100 -> -1
    %   1100 -> +1
    %   1101 -> +3
    %   1111 -> +5
    %   1110 -> +7
    %   1010 -> +9
    %   1011 -> +11
    %   1001 -> +13
    %   1000 -> +15

    methods
        function obj = Qam256Modulator()
            obj@modulators.AbstractModulator(256, '256-QAM Modulator');
        end

        function symbols = modulate(obj, bits)
            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:));
            B = obj.BitsPerSymbol; % 8

            if mod(numel(b), B) ~= 0
                error('Qam256Modulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), B);
            end

            bitsMat = reshape(b, B, []).'; % [Nsym x 8]

            % Q uses [b7 b6 b5 b4], I uses [b3 b2 b1 b0]
            qBits = bitsMat(:,1:4);
            iBits = bitsMat(:,5:8);

            I = modulators.Qam256Modulator.bits2pamGray(iBits);
            Q = modulators.Qam256Modulator.bits2pamGray(qBits);

            % Normalize for unit average power
            normFactor = sqrt(170);
            symbols = (I + 1j*Q) / normFactor;
        end
    end

    methods (Static, Access = private)
        function pam = bits2pamGray(b4)
            if size(b4,2) ~= 4
                error('Qam256Modulator:bits2pamGray', ...
                      'Input must have 4 columns of bits.');
            end
            val = b4(:,1)*8 + b4(:,2)*4 + b4(:,3)*2 + b4(:,4);
            % 16-PAM Gray-coded mapping
            map = [-15; -13; -11; -9; -7; -5; -3; -1; ...
                    +15; +13; +11; +9; +7; +5; +3; +1];
            pam = map(val+1);
        end
    end
end
