classdef CPPQam256Demodulator < demodulators.AbstractDemodulator
    % CPPQam256Demodulator uses c++ backend for demodulation

    methods
        function obj = CPPQam256Demodulator()
            obj@demodulators.AbstractDemodulator(256, 'CPP 256-QAM Demodulator');
        end

        function G = getAmbiguityRotations(~)
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(~, symbols)
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            bits = mex.qam256_demod_hard_mex(symbols);
        end
    end
end
