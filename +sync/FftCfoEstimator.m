classdef FftCfoEstimator < handle
    % FftCfoEstimator

    properties
        SampleRateSym = 1e5;   % Rsym
        PreambleSyms  = [];    % known preamble symbols
        Nfft          = 1024;  % FFT length

        NumCandidates = 3;     % number of FFT peaks to consider
    end

    methods
        function obj = FftCfoEstimator(varargin)
            if mod(numel(varargin),2) ~= 0
                error('FftCfoEstimator:NameValue', ...
                      'Constructor expects name-value pairs.');
            end
            for k = 1:2:numel(varargin)
                name  = varargin{k};
                value = varargin{k+1};
                switch lower(name)
                    case 'sampleratesym'
                        obj.SampleRateSym = value;
                    case 'preamblesyms'
                        obj.PreambleSyms  = value(:);
                    case 'nfft'
                        obj.Nfft = value;
                    case 'numcandidates'
                        obj.NumCandidates = max(1, round(value));
                    otherwise
                        error('FftCfoEstimator:UnknownParam', ...
                              'Unknown parameter "%s".', name);
                end
            end

            if isempty(obj.PreambleSyms)
                error('FftCfoEstimator:MissingPreamble', ...
                      'PreambleSyms must be provided.');
            end
        end

        function [wSym, fCfoHz, peakVal] = estimate(obj, ySym, preStartSym)
            Lpre = numel(obj.PreambleSyms);
            idx0 = preStartSym;
            idx1 = idx0 + Lpre - 1;

            if idx1 > numel(ySym)
                % Not enough symbols -> return zero
                wSym    = 0;
                fCfoHz  = 0;
                peakVal = 0;
                return;
            end

            r = ySym(idx0:idx1);      % received preamble @ 1 sps
            s = obj.PreambleSyms(:);  % known preamble

            % Remove known modulation, leaving approx pure CFO tone:
            z = r(:) .* conj(s);

            % FFT of z, zero-padded to Nfft
            N = obj.Nfft;
            Z = fftshift(fft(z, N));

            magZ = abs(Z);

            % Sort bins by magnitude (descending)
            [~, idxSorted] = sort(magZ, 'descend');

            K = min(obj.NumCandidates, numel(idxSorted));
            idxCand = idxSorted(1:K);

            % Bin index in range [-N/2 .. +N/2-1]
            kCand = idxCand - (N/2 + 1);

            % Cycles per symbol for each candidate
            f_cyc_per_sym = kCand / N;

            % Convert to Hz (candidates)
            fCandHz = f_cyc_per_sym * obj.SampleRateSym;

            % Take the strongest candidate
            fCfoHz = fCandHz(1);
            peakVal = magZ(idxCand(1));

            % Convert to rad/symbol
            wSym   = 2*pi * fCfoHz / obj.SampleRateSym;
        end
    end
end
