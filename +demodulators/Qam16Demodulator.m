classdef Qam16Demodulator < demodulators.AbstractDemodulator
    methods
        function obj = Qam16Demodulator()
            obj@demodulators.AbstractDemodulator(16, '16-QAM Demodulator');
        end

        function G = getAmbiguityRotations(obj)
            % 4-fold symmetry for rectangular QAM
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(obj, symbols)
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            z = symbols(:);
            B = obj.BitsPerSymbol;      % 4

            % Undo normalization
            normFactor = sqrt(10);
            s = z * normFactor;
        
            P = mean(abs(s).^2);
            if P <= 0
                bits = zeros(0,1);
                return;
            end
            gainMag = sqrt(P / 10);
        
            sNorm = s / gainMag;
        
            I = real(sNorm);
            Q = imag(sNorm);

            levels = [-3 -1 1 3];

            [~, idxI] = min(abs(I - levels), [], 2);
            [~, idxQ] = min(abs(Q - levels), [], 2);

            pamI = levels(idxI).';
            pamQ = levels(idxQ).';

            bitsI = demodulators.Qam16Demodulator.pamGray2bits(pamI(:));
            bitsQ = demodulators.Qam16Demodulator.pamGray2bits(pamQ(:));

            nSym    = numel(z);
            bitsMat = zeros(nSym, B);
            bitsMat(:,1:2) = bitsQ;
            bitsMat(:,3:4) = bitsI;

            bits = reshape(bitsMat.', [], 1);
        end
    end

    methods (Static, Access = private)
        function pam = bits2pamGray(b2)
            % Same mapping as in Qam16Modulator
            if size(b2,2) ~= 2
                error('Qam16Demodulator:bits2pamGray', ...
                      'Input must have 2 columns of bits.');
            end
            msb = b2(:,1);
            lsb = b2(:,2);
            val = msb*2 + lsb;
            map = [-3; -1; +3; +1];
            pam = map(val+1);
        end

        function b2 = pamGray2bits(pam)
            % Inverse of 4-PAM mapping:
            %   -3 -> 00
            %   -1 -> 01
            %   +1 -> 11
            %   +3 -> 10

            pam = pam(:);
            n   = numel(pam);
        
            idx = (pam + 3)/2 + 1;   % gives 1,2,3,4 for -3,-1,1,3
        
            lut = [0 0;
                   0 1;
                   1 1;
                   1 0];
        
            % Vectorized lookup
            b2 = lut(idx, :);

            b2 = double(b2);
        end
    end
end
