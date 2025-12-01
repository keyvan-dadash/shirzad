classdef Qam32Modulator < modulators.AbstractModulator
    % 32-QAM rectangular (8x4).
    %
    % Bits per symbol = 5:
    %   [b4 b3 b2 b1 b0]
    %   Q uses [b4 b3] (4-PAM)
    %   I uses [b2 b1 b0] (8-PAM)
    %
    % I-levels: -7 -5 -3 -1 1 3 5 7
    % Q-levels: -3 -1 1 3

    methods
        function obj = Qam32Modulator()
            obj@modulators.AbstractModulator(32, '32-QAM Modulator');
        end

        function symbols = modulate(obj, bits)
            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:));
            B = obj.BitsPerSymbol;   % 5

            if mod(numel(b), B) ~= 0
                error('Qam32Modulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), B);
            end

            bitsMat = reshape(b, B, []).';   % [Nsym x 5]

            % Q: [b4 b3]  (cols 1:2)
            % I: [b2 b1 b0] (cols 3:5)
            qBits = bitsMat(:,1:2);
            iBits = bitsMat(:,3:5);

            I = modulators.Qam32Modulator.bits3to8pam(iBits);
            Q = modulators.Qam32Modulator.bits2to4pamGray(qBits);

            normFactor = sqrt(26); % normal factor for 32-qam
            symbols    = (I + 1j*Q) / normFactor;
        end
    end

    methods (Static, Access = private)
        function pam = bits3to8pam(b3)
            % 8-PAM mapping:
            % val = 0..7 -> -7 + 2*val
            if size(b3,2) ~= 3
                error('Qam32Modulator:bits3to8pam', ...
                      'Input must have 3 columns of bits.');
            end
            val = b3(:,1)*4 + b3(:,2)*2 + b3(:,3);   % 0..7
            pam = -7 + 2*val;
        end

        function pam = bits2to4pamGray(b2)
            if size(b2,2) ~= 2
                error('Qam32Modulator:bits2to4pamGray', ...
                      'Input must have 2 columns of bits.');
            end
            msb = b2(:,1);
            lsb = b2(:,2);
            val = msb*2 + lsb;
            map = [-3; -1; +3; +1];   % 00->-3, 01->-1, 10->+3, 11->+1
            pam = map(val+1);
        end
    end
end
