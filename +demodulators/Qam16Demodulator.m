classdef Qam16Demodulator < demodulators.AbstractDemodulator
    % 16-QAM Gray, matches Qam16Modulator.

    methods
        function obj = Qam16Demodulator()
            obj@demodulators.AbstractDemodulator(16, '16-QAM Demodulator');
        end

        function G = getAmbiguityRotations(obj) %#ok<MANU>
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

            I = real(s);
            Q = imag(s);

            levels = [-3 -1 1 3];

            % Nearest-neighbor quantization to 4-PAM
            [~, idxI] = min(abs(I - levels.'), [], 2);   % 1..4
            [~, idxQ] = min(abs(Q - levels.'), [], 2);

            pamI = levels(idxI).';
            pamQ = levels(idxQ).';

            bitsI = Qam16Demodulator.pamGray2bits(pamI(:));
            bitsQ = Qam16Demodulator.pamGray2bits(pamQ(:));

            nSym    = numel(z);
            bitsMat = zeros(nSym, B);
            bitsMat(:,1:2) = bitsQ;     % Q -> [b3 b2]
            bitsMat(:,3:4) = bitsI;     % I -> [b1 b0]

            bits = reshape(bitsMat.', [], 1);
        end

        function llr = demodulateLlr(obj, symbols, noiseVarPerDim)
            if isempty(symbols)
                llr = zeros(0,1);
                return;
            end
            if nargin < 3 || noiseVarPerDim <= 0
                error('Qam16Demodulator:NoiseVar', ...
                      'noiseVarPerDim must be positive.');
            end

            z = symbols(:);
            nSym = numel(z);
            B    = obj.BitsPerSymbol;   % 4
            M    = obj.M;               % 16

            % Build constellation + label table consistent with modulator
            bitsAll = de2bi(0:M-1, B, 'left-msb');  % [16 x 4]
            qBits   = bitsAll(:,1:2);
            iBits   = bitsAll(:,3:4);

            I = Qam16Demodulator.bits2pamGray(iBits);
            Q = Qam16Demodulator.bits2pamGray(qBits);

            normFactor = sqrt(10);
            const = (I + 1j*Q) / normFactor;   % [16 x 1]

            llr   = zeros(B*nSym,1);
            invNv = 1 / noiseVarPerDim;

            for n = 1:nSym
                y  = z(n);
                d2 = abs(y - const.').^2;      % 1 x 16

                for bIx = 1:B
                    idx0 = (bitsAll(:,bIx) == 0);
                    idx1 = ~idx0;
                    d0   = min(d2(idx0));
                    d1   = min(d2(idx1));
                    llr((n-1)*B + bIx) = (d1 - d0) * invNv;
                end
            end
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
            % Inverse of 4-PAM Gray mapping:
            %   -3 -> 00
            %   -1 -> 01
            %   +1 -> 11
            %   +3 -> 10
            pam = pam(:);
            n   = numel(pam);
            b2  = zeros(n, 2);

            for k = 1:n
                switch pam(k)
                    case -3
                        b2(k,:) = [0 0];
                    case -1
                        b2(k,:) = [0 1];
                    case 1
                        b2(k,:) = [1 1];
                    case 3
                        b2(k,:) = [1 0];
                    otherwise
                        error('Qam16Demodulator:pamGray2bits', ...
                              'Invalid PAM level %g for 16-QAM.', pam(k));
                end
            end
        end
    end
end
