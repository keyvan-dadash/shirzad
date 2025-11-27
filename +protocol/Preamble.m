classdef Preamble < handle
    %PREAMBLE Generate and hold a repeated [a, a] preamble.
    %
    %   Typical usage (QPSK):
    %
    %       mod  = modulators.QpskModulator();
    %       pre  = Preamble.fromMSequence(mod, 128, ...
    %                  'Degree', 9, 'Seed', 1001);
    %
    %   Then:
    %       preSyms = pre.Symbols;       % [2*Lh x 1] complex
    %       Lpre    = pre.NumSymbols;    % == 2*Lh
    %
    % The RX side (Schmidl & Cox detector, etc.) stays the same – it just
    % needs to know Lh and the actual preamble symbols if you use
    % correlation for validation.

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
            % Preamble(modulator, halfLenSym, bits)
            %
            % bits must correspond to [a, a] in symbol domain.

            arguments
                modulator
                halfLenSym (1,1) double {mustBePositive, mustBeInteger}
                bits       (:,1) double {mustBeNonnegative, mustBeLessThanOrEqual(bits,1)}
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
            % fromMSequence(modulator, halfLenSym, 'Degree', 9, 'Seed', 1001)
            %
            % Builds a [a,a] preamble from an m-sequence in *bit* domain.
            % halfLenSym is the half-preamble length in *symbols*.

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

            % Full preamble bits = [a_bits; a_bits]
            preBits = [preBitsHalf; preBitsHalf];

            obj = protocol.Preamble(modulator, halfLenSym, preBits);
        end
    end
end
