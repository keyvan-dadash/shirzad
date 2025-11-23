classdef FftCfoEstimator < handle
    % FftCfoEstimator uses known preamble and FFT to estimate the CFO

    properties
        SampleRateSym = 1e5;
        PreambleSyms  = [];
        Nfft          = 1024;

        UseHistory    = false;
        NumCandidates = 3;
        Alpha         = 0.3;
        MaxJumpHz     = 2e3;
    end

    properties (Access = private)
        lastCfoHz  = [];
        lastValid  = false;
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
                    case 'usehistory'
                        obj.UseHistory = logical(value);
                    case 'numcandidates'
                        obj.NumCandidates = max(1, round(value));
                    case 'alpha'
                        obj.Alpha = max(0, min(1, value));
                    case 'maxjumphz'
                        obj.MaxJumpHz = abs(value);
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

        function resetHistory(obj)
            obj.lastCfoHz = [];
            obj.lastValid = false;
        end

        function [wSym, fCfoHz, peakVal] = estimate(obj, ySym, preStartSym)
            % ESTIMATE  CFO from symbol-rate preamble
            %
            % [wSym, fCfoHz, peakVal] = obj.estimate(ySym, preStartSym)
            %
            % ySym        : full 1-sps symbol stream
            % preStartSym : 1-based index of first preamble symbol

            Lpre = numel(obj.PreambleSyms);
            idx0 = preStartSym;
            idx1 = idx0 + Lpre - 1;

            if idx1 > numel(ySym)
                % Not enough symbols -> return zero (and keep history)
                wSym    = 0;
                fCfoHz  = 0;
                peakVal = 0;
                return;
            end

            r = ySym(idx0:idx1);      % received preamble @ 1 sps
            s = obj.PreambleSyms(:);  % known preamble

            % Remove known modulation, leaving approx pure CFO tone:
            % z[n] ≈ A * exp(j*wSym*n)
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

            % Default: take the strongest (like original estimator)
            fInstHz = fCandHz(1);
            peakVal = magZ(idxCand(1));

            % Stateless mode: just return the instantaneous best estimate
            if ~obj.UseHistory
                fCfoHz = fInstHz;
                wSym   = 2*pi * fCfoHz / obj.SampleRateSym;
                return;
            end

            % --------- Tracking mode: use history to choose candidate ------
            if ~obj.lastValid
                fCfoHz = fInstHz;
                obj.lastCfoHz = fCfoHz;
                obj.lastValid = true;

                wSym = 2*pi * fCfoHz / obj.SampleRateSym;
                return;
            end

            % Find the candidate closest to the last CFO (in Hz)
            [~, bestIdx] = min(abs(fCandHz - obj.lastCfoHz));
            fChosenHz = fCandHz(bestIdx);
            peakVal   = magZ(idxCand(bestIdx));  % chosen peak

            % Limit the jump relative to history
            df = fChosenHz - obj.lastCfoHz;
            if abs(df) > obj.MaxJumpHz
                df = sign(df) * obj.MaxJumpHz;
            end
            fLimitedHz = obj.lastCfoHz + df;

            % Smooth in time
            fCfoHz = (1 - obj.Alpha)*obj.lastCfoHz + obj.Alpha * fLimitedHz;

            % Update history
            obj.lastCfoHz = fCfoHz;
            obj.lastValid = true;

            % Output rad/symbol
            wSym = 2*pi * fCfoHz / obj.SampleRateSym;
        end
    end
end
