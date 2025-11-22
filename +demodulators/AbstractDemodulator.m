classdef (Abstract) AbstractDemodulator < handle
    % Every demodulator should implement this class for unpacking symbols
    % to bits.

    properties (SetAccess = protected)
        M
        BitsPerSymbol
        Name
    end

    methods
        function obj = AbstractDemodulator(M, name)
            % Constructor
            obj.M = M;
            obj.BitsPerSymbol = log2(M);
        end
    end

    methods (Abstract)
        bits = demodulateHard(obj, symbols);
        llr  = demodulateLlr(obj, symbols, noiseVarPerDim);
    end
end
