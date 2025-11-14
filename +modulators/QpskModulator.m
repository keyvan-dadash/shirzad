classdef QpskModulator < modulators.AbstractModulator
    %QPSKMODULATOR Gray-coded QPSK, unit average power.
    %
    % Bit mapping per symbol (b1 = first bit, MSB; b0 = second bit, LSB):
    %
    %   b1 b0  ->  symbol
    %   0  0   ->  ( +1 + j*+1 ) / sqrt(2)
    %   0  1   ->  ( -1 + j*+1 ) / sqrt(2)
    %   1  1   ->  ( -1 + j*-1 ) / sqrt(2)
    %   1  0   ->  ( +1 + j*-1 ) / sqrt(2)
    %
    % Average symbol power = 1.

    methods
        function obj = QpskModulator()
            obj@modulators.AbstractModulator(4, 'QPSK Modulator');
        end

        function symbols = modulate(obj, bits)
            %MODULATE Map bits -> QPSK symbols.
            %
            % bits    : column vector of 0/1
            % symbols : column vector of complex values

            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:));  % ensure column, double
            if mod(numel(b), obj.BitsPerSymbol) ~= 0
                error('QpskModulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), obj.BitsPerSymbol);
            end

            % Group bits as [b1; b0] per symbol
            b1 = b(1:2:end);    % MSB (affects Q)
            b0 = b(2:2:end);    % LSB (affects I)

            % Map bits to I/Q in {+1, -1}
            % b=0 -> +1, b=1 -> -1
            I = 1 - 2*b0;       % LSB controls I
            Q = 1 - 2*b1;       % MSB controls Q

            symbols = (I + 1j*Q) / sqrt(2);
        end
    end
end
