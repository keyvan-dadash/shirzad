classdef Preamble < handle
    properties (SetAccess = immutable)
        Modulator           % AbstractModulator (e.g. QpskModulator)
        HalfLengthSym
        Bits                 % column, full preamble bits (optional)
        Symbols              % column, complex preamble symbols
    end

    properties (Dependent)
        NumSymbols   % = 2*HalfLengthSym
    end

    methods
        function obj = Preamble(modulator, halfLenSym, bits)
            arguments
                modulator
                halfLenSym (1,1) double
                bits       (:,1) double
            end

            obj.Modulator   = modulator;
            obj.HalfLengthSym = halfLenSym;
            obj.Bits        = bits(:);

            % Map bits → symbols via provided modulator
            obj.Symbols     = modulator.modulate(obj.Bits);
        end

        function n = get.NumSymbols(obj)
            n = numel(obj.Symbols);
        end
    end

    methods (Static)
        function obj = fromMSequence(modulator, halfLenSym, varargin)
            p = inputParser;
            addParameter(p, 'Degree', 9);
            addParameter(p, 'Seed',   1001);
            parse(p, varargin{:});
            cfg = p.Results;

            bps = modulator.BitsPerSymbol;

            % Generate half-preamble bits via m-sequence
            gen = training.MSequenceGenerator('Degree', cfg.Degree);
            preBitsHalf = gen.generateBits(halfLenSym * bps);
            preBitsHalf = preBitsHalf(:);

            preBits = [preBitsHalf; preBitsHalf];

            obj = protocol.Preamble(modulator, halfLenSym, preBits);
        end
    end
end
