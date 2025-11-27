classdef QpskDemodulator < demodulators.AbstractDemodulator
    methods
        function obj = QpskDemodulator()
            obj@demodulators.AbstractDemodulator(4, 'QPSK Demodulator');
        end

        % --------- NEW: QPSK-specific symmetry rotations ----------
        function G = getAmbiguityRotations(obj) %#ok<MANU>
            % QPSK has 4-fold rotational symmetry → {1, -1, j, -j}
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(obj, symbols)
            if isempty(symbols) % if symbols are empty then we are done
                bits = zeros(0,1);
                return;
            end

            z = symbols(:);
            s = z * sqrt(2);

            I = real(s);
            Q = imag(s);

            b0 = double(I < 0); %LSB
            b1 = double(Q < 0); %MSB

            bits = zeros(2*numel(z),1);
            bits(1:2:end) = b1;
            bits(2:2:end) = b0;
        end

        function llr = demodulateLlr(obj, symbols, noiseVarPerDim)
            %Demodulate using log-likelihood

            if isempty(symbols)
                llr = zeros(0,1);
                return;
            end

            if nargin < 3 || noiseVarPerDim <= 0
                error('QpskDemodulator:NoiseVar', ...
                      'noiseVarPerDim must be positive.');
            end

            z = symbols(:);
            s = z * sqrt(2);

            I = real(s);
            Q = imag(s);

            llr0 = -2 * I / noiseVarPerDim;   %LSB
            llr1 = -2 * Q / noiseVarPerDim;   %MSB

            llr = zeros(2*numel(z),1);
            llr(1:2:end) = llr1;
            llr(2:2:end) = llr0;
        end
    end
end
