classdef QpskModulator < modulators.AbstractModulator
    %QPSK modulator
    %
    % b0 b1
    % 0  0  -> +1 + 1j
    % 1  0  -> -1 + 1j
    % 1  1  -> -1 - 1j
    % 0  1  -> +1 - 1j

    methods
        function obj = QpskModulator()
            obj@modulators.AbstractModulator(4, 'QPSK Modulator');
        end

        function symbols = modulate(obj, bits)
            if isempty(bits)
                symbols = complex([]);
                return;
            end

            b = double(bits(:)); % columns wise
            if mod(numel(b), obj.BitsPerSymbol) ~= 0
                error('QpskModulator:BitLength', ...
                      'Length of bits (%d) must be a multiple of %d.', ...
                      numel(b), obj.BitsPerSymbol);
            end

            % Seperate bits into two different groups
            b1 = b(1:2:end);    % MSB (affects Q)
            b0 = b(2:2:end);    % LSB (affects I)

            % Map bits to I/Q in {+1, -1}
            % b=0 -> +1, b=1 -> -1
            I = 1 - 2*b0;       % LSB controls I
            Q = 1 - 2*b1;       % MSB controls Q

            % normalize
            symbols = (I + 1j*Q) / sqrt(2);
        end
    end
end
