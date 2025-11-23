classdef (Abstract) AbstractModulator < handle
    % Base class for digital modulators.
    %   - M: The level of this modulator
    %   - BitPerSymbol: how much this modulator transfers bits per symbol
    %   - Name: the name of this modulator
    properties (SetAccess = protected)
        M
        BitsPerSymbol
        Name
    end

    methods
        function obj = AbstractModulator(M, name)
            obj.M = M;
            obj.BitsPerSymbol = log2(M);
            obj.Name = char(name);
        end
    end

    methods (Abstract)
        symbols = modulate(obj, bits);
    end
end
