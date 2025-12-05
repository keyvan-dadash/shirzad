classdef CPPQam16Demodulator < demodulators.AbstractDemodulator
    % CPPQam16Demodulator uses c++ backend for demodulation

    methods
        function obj = CPPQam16Demodulator()
            obj@demodulators.AbstractDemodulator(16, 'CPP 16-QAM Demodulator');
        end

        function G = getAmbiguityRotations(~)
            G = [1, -1, 1j, -1j];
        end

        function bits = demodulateHard(~, symbols)
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            bits = demodulators.qam16_demod_hard_mex(symbols);
        end
    end
end
