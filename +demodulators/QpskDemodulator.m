classdef QpskDemodulator < demodulators.AbstractDemodulator
    methods
        function obj = QpskDemodulator()
            obj@demodulators.AbstractDemodulator(4, 'QPSK Demodulator');
        end

        function G = getAmbiguityRotations(obj)
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(obj, symbols)
            if isempty(symbols)
                bits = zeros(0,1,'logical');
                return;
            end

            z = symbols(:);

            I = real(z);
            Q = imag(z);

            bI = I < 0;   % LSB (I)
            bQ = Q < 0;   % MSB (Q)

            % Interleave [bQ bI] -> [bQ(1); bI(1); bQ(2); bI(2); ...]
            nSym = numel(z);
            bits = false(2*nSym,1);
            bits(1:2:end) = bQ;
            bits(2:2:end) = bI;
        end
    end
end
