classdef QpskDemodulator < demodulators.AbstractDemodulator
    %QPSKDEMODULATOR Gray-coded QPSK demodulator.
    %
    % Assumes same mapping as QpskModulator:
    %   b1 b0  ->  symbol
    %   0  0   ->  ( +1 + j*+1 ) / sqrt(2)
    %   0  1   ->  ( -1 + j*+1 ) / sqrt(2)
    %   1  1   ->  ( -1 + j*-1 ) / sqrt(2)
    %   1  0   ->  ( +1 + j*-1 ) / sqrt(2)
    %
    % For LLR:
    %   noiseVarPerDim = variance of noise per real dimension.
    %   LLR = log(P(b=1)/P(b=0)), so positive => bit '1'.

    methods
        function obj = QpskDemodulator()
            obj@demodulators.AbstractDemodulator(4, 'QPSK Demodulator');
        end

        function bits = demodulateHard(obj, symbols)
            %DEMODULATEHARD Hard decision demod -> bits (0/1).
            if isempty(symbols)
                bits = zeros(0,1);
                return;
            end

            z = symbols(:);       % column
            % Undo sqrt(2) scaling to work around small distortions
            s = z * sqrt(2);

            I = real(s);
            Q = imag(s);

            % Recall mapping: b0 from I, b1 from Q, where:
            %   b = 0 if component > 0; b = 1 if component < 0
            b0 = double(I < 0);   % LSB
            b1 = double(Q < 0);   % MSB

            % Interleave [b1 b0] per symbol
            bits = zeros(2*numel(z),1);
            bits(1:2:end) = b1;
            bits(2:2:end) = b0;
        end

        function llr = demodulateLlr(obj, symbols, noiseVarPerDim)
            %DEMODULATELLR Compute approximate LLRs for QPSK.
            %
            % symbols        : column vector of complex values
            % noiseVarPerDim : variance per real dimension (sigma^2)
            %
            % Returns LLR vector with same bit ordering as demodulateHard:
            %   [b1_1; b0_1; b1_2; b0_2; ...]
            %
            % For BPSK (±1) with AWGN:
            %   LLR(b=1) = log(P(b=1)/P(b=0)) = -2*r / sigma^2

            if isempty(symbols)
                llr = zeros(0,1);
                return;
            end

            if nargin < 3 || noiseVarPerDim <= 0
                error('QpskDemodulator:NoiseVar', ...
                      'noiseVarPerDim must be positive.');
            end

            z = symbols(:);
            s = z * sqrt(2);   % scale to ±1

            I = real(s);
            Q = imag(s);

            % LLR for bit=1 given observation r:
            %   LLR = -2*r / sigma^2  (b=0 -> +1, b=1 -> -1)
            llr0 = -2 * I / noiseVarPerDim;   % for b0 (LSB)
            llr1 = -2 * Q / noiseVarPerDim;   % for b1 (MSB)

            % Pack as [b1; b0] per symbol
            N = numel(z);
            llr = zeros(2*N,1);
            llr(1:2:end) = llr1;
            llr(2:2:end) = llr0;
        end
    end
end
