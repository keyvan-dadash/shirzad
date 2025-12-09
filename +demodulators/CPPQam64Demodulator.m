classdef CPPQam64Demodulator < demodulators.AbstractDemodulator
    % CPPQam64Demodulator uses c++ backend for demodulation

    methods
        function obj = CPPQam64Demodulator()
            obj@demodulators.AbstractDemodulator(64, 'CPP 64-QAM Demodulator');
        end

        function G = getAmbiguityRotations(~)
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(~, symbols)
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            bits = mex.qam64_demod_hard_mex(symbols);
        end
    end
end
