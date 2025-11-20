classdef (Abstract) AbstractModulator < handle
    %ABSTRACTMODULATOR Base class for digital modulators.
    %
    %   - M              : constellation order (e.g., 4 for QPSK)
    %   - BitsPerSymbol  : log2(M)
    %   - Name           : optional human-readable name
    %
    %   Subclasses must implement:
    %       symbols = modulate(obj, bits);
    %
    %   where:
    %       bits    : column vector of 0/1 (double or logical)
    %       symbols : column vector of complex baseband symbols

    properties (SetAccess = protected)
        M
        BitsPerSymbol
        Name
    end

    methods
        function obj = AbstractModulator(M, name)
            if nargin < 1
                error('AbstractModulator:MissingM', ...
                      'Constellation order M must be specified.');
            end
            obj.M = M;
            obj.BitsPerSymbol = log2(M);
            if nargin >= 2
                obj.Name = char(name);
            else
                obj.Name = sprintf('M=%d Modulator', M);
            end
        end
    end

    methods (Abstract)
        symbols = modulate(obj, bits);
    end
end
