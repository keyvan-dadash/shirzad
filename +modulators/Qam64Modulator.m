classdef Qam64Modulator < modulators.AbstractModulator
    % 64-QAM modulator.
    %
    % Bits per symbol = 6:
    %   [b5 b4 b3 b2 b1 b0]
    %   Q uses [b5 b4 b3], I uses [b2 b1 b0]
    %
    % 3-bit to 8-PAM Gray mapping per axis:
    %   000 -> -7
    %   001 -> -5
    %   011 -> -3
    %   010 -> -1
    %   110 -> +1
    %   111 -> +3
    %   101 -> +5
    %   100 -> +7

    methods
        function obj = Qam64Modulator()
            obj@modulators.AbstractModulator(64, '64-QAM Modulator');
        end

        function symbols = modulate(obj, bits)
            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:));
            B = obj.BitsPerSymbol; % 6

            if mod(numel(b), B) ~= 0
                error('Qam64Modulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), B);
            end

            bitsMat = reshape(b, B, []).'; % [Nsym x 6]

            % Q uses [b5 b4 b3], I uses [b2 b1 b0]
            qBits = bitsMat(:,1:3);
            iBits = bitsMat(:,4:6);

            I = modulators.Qam64Modulator.bits2pamGray(iBits);
            Q = modulators.Qam64Modulator.bits2pamGray(qBits);

            % Normalize for unit average power
            normFactor = sqrt(42);
            symbols = (I + 1j*Q) / normFactor;
        end
    end

    methods (Static, Access = private)
        function pam = bits2pamGray(b3)
            if size(b3,2) ~= 3
                error('Qam64Modulator:bits2pamGray', ...
                      'Input must have 3 columns of bits.');
            end
            val = b3(:,1)*4 + b3(:,2)*2 + b3(:,3);
            % Gray-coded mapping
            map = [-7; -5; -3; -1; +7; +5; +3; +1];
            pam = map(val+1);
        end
    end
end
