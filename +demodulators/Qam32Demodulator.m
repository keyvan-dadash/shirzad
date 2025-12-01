classdef Qam32Demodulator < demodulators.AbstractDemodulator
    % 32-QAM rectangular (8x4), matches Qam32Modulator.

    methods
        function obj = Qam32Demodulator()
            obj@demodulators.AbstractDemodulator(32, '32-QAM Demodulator');
        end

        function G = getAmbiguityRotations(obj)
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(obj, symbols)
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            z = symbols(:);
            B = obj.BitsPerSymbol; % 5
            
            % Undo normalization
            normFactor = sqrt(26);
            s = z * normFactor;         

            P = mean(abs(s).^2);        
            if P <= 0
                bits = zeros(0,1);
                return;
            end
            gainMag = sqrt(P / 26);
        
            sNorm = s / gainMag;        
        
            I = real(sNorm);
            Q = imag(sNorm);

            % Quantize I to 8-PAM levels
            Ilevels   = (-7:2:7).';     
            [~, iIdx] = min(abs(I - Ilevels.'), [], 2);
            Iq        = Ilevels(iIdx).';

            % Quantize Q to 4-PAM levels
            Qlevels   = [-3 -1 1 3];
            [~, qIdx] = min(abs(Q - Qlevels.'), [], 2);
            Qq        = Qlevels(qIdx).';

            % I: natural 8-PAM => valI = (Iq+7)/2, then 3-bit binary
            valI  = uint8((Iq + 7)/2);
            bitsI = de2bi(valI, 3, 'left-msb');   % [N x 3]

            % Q: Gray 4-PAM, same as 16-QAM
            bitsQ = Qam32Demodulator.pam4Gray2bits(Qq(:));

            nSym    = numel(z);
            bitsMat = zeros(nSym, B);
            bitsMat(:,1:2) = bitsQ;    % [b4 b3]
            bitsMat(:,3:5) = bitsI;    % [b2 b1 b0]

            bits = reshape(bitsMat.', [], 1);
        end

        function llr = demodulateLlr(obj, symbols, noiseVarPerDim)
            if isempty(symbols)
                llr = zeros(0,1);
                return;
            end
            if nargin < 3 || noiseVarPerDim <= 0
                error('Qam32Demodulator:NoiseVar', ...
                      'noiseVarPerDim must be positive.');
            end

            z = symbols(:);
            nSym = numel(z);
            B    = obj.BitsPerSymbol;    % 5
            M    = obj.M;                % 32

            % Build constellation table consistent with modulator
            bitsAll = de2bi(0:M-1, B, 'left-msb');   % [32 x 5]
            qBits   = bitsAll(:,1:2);
            iBits   = bitsAll(:,3:5);

            I = Qam32Demodulator.bits3to8pam(iBits);
            Q = Qam32Demodulator.bits2to4pamGray(qBits);

            normFactor = sqrt(26);
            const = (I + 1j*Q) / normFactor;   % [32 x 1]

            llr   = zeros(B*nSym, 1);
            invNv = 1 / noiseVarPerDim;

            for n = 1:nSym
                y  = z(n);
                d2 = abs(y - const.').^2;       % 1 x 32

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
        function pam = bits3to8pam(b3)
            if size(b3,2) ~= 3
                error('Qam32Demodulator:bits3to8pam', ...
                      'Input must have 3 columns of bits.');
            end
            val = b3(:,1)*4 + b3(:,2)*2 + b3(:,3);  % 0..7
            pam = -7 + 2*val;
        end

        function pam = bits2to4pamGray(b2)
            if size(b2,2) ~= 2
                error('Qam32Demodulator:bits2to4pamGray', ...
                      'Input must have 2 columns of bits.');
            end
            msb = b2(:,1);
            lsb = b2(:,2);
            val = msb*2 + lsb;
            map = [-3; -1; +3; +1];
            pam = map(val+1);
        end

        function b2 = pam4Gray2bits(pam)
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
                        error('Qam32Demodulator:pam4Gray2bits', ...
                              'Invalid 4-PAM level %g.', pam(k));
                end
            end
        end
    end
end
