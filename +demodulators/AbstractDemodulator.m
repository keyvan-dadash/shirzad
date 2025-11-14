classdef (Abstract) AbstractDemodulator < handle
    %ABSTRACTDEMODULATOR Base class for digital demodulators.
    %
    % Subclasses must implement:
    %   bits = demodulateHard(obj, symbols);
    %   llr  = demodulateLlr(obj, symbols, noiseVar);
    %
    % where:
    %   symbols : column vector of complex samples
    %   bits    : column vector of 0/1 (double)
    %   llr     : column vector of LLRs (log P(b=1)/P(b=0))

    properties (SetAccess = protected)
        M
        BitsPerSymbol
        Name
    end

    methods
        function obj = AbstractDemodulator(M, name)
            if nargin < 1
                error('AbstractDemodulator:MissingM', ...
                      'Constellation order M must be specified.');
            end
            obj.M = M;
            obj.BitsPerSymbol = log2(M);
            if nargin >= 2
                obj.Name = char(name);
            else
                obj.Name = sprintf('M=%d Demodulator', M);
            end
        end
    end

    methods (Abstract)
        bits = demodulateHard(obj, symbols);
        llr  = demodulateLlr(obj, symbols, noiseVarPerDim);
    end
end
