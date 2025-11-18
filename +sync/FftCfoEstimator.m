% File: +sync/FftCfoEstimator.m
classdef FftCfoEstimator < handle
    % FftCfoEstimator
    %
    % Estimate carrier frequency offset (CFO) from a known preamble at
    % SYMBOL RATE using an FFT-based approach, with optional temporal
    % smoothing / tracking to avoid large spurious jumps.
    %
    % Idea:
    %   r[n] = s[n] * exp(j*wSym*n) + noise
    %   s[n] : known preamble symbols
    %   r[n] : received preamble symbols (at 1 sps)
    %
    % Form:
    %   z[n] = r[n] * conj(s[n]) ≈ A * exp(j*wSym*n)
    %
    % Take FFT of z[n] (zero-padded to Nfft), find a small set of
    % candidate peaks. Then, if history is enabled, choose the candidate
    % whose frequency is closest to the historical CFO (and limit the
    % allowed jump between frames).
    %
    % Properties (name-value in constructor):
    %   SampleRateSym : symbol rate (Rsym) in Hz
    %   PreambleSyms  : known preamble symbols s[n] (column vector)
    %   Nfft          : FFT length (power of two recommended)
    %
    %   UseHistory    : if true, track CFO over frames and avoid
    %                   large jumps. Default: false (stateless usage)
    %   NumCandidates : how many top FFT peaks to consider (>=1)
    %                   Default: 3
    %   Alpha         : smoothing factor for CFO in Hz
    %                   f_out = (1-Alpha)*f_prev + Alpha*f_inst
    %                   Default: 0.3
    %   MaxJumpHz     : max allowed CFO change (Hz) between frames
    %                   (before smoothing). Default: 2e3
    %
    % Methods:
    %   [wSym, fCfoHz, peakVal] = estimate(ySym, preStartSym)
    %   resetHistory()
    %
    %   ySym        : full 1-sps symbol stream
    %   preStartSym : 1-based index of the first preamble symbol in ySym
    %
    %   wSym    : CFO in rad/symbol
    %   fCfoHz  : CFO in Hz
    %   peakVal : magnitude of the chosen FFT peak

    properties
        SampleRateSym = 1e5;   % Rsym
        PreambleSyms  = [];    % known preamble symbols
        Nfft          = 1024;  % FFT length

        UseHistory    = false; % default: stateless
        NumCandidates = 3;     % number of FFT peaks to consider
        Alpha         = 0.3;   % smoothing factor (0..1)
        MaxJumpHz     = 2e3;   % max CFO jump allowed between frames
    end

    properties (Access = private)
        lastCfoHz  = [];       % last smoothed CFO estimate in Hz
        lastValid  = false;    % true if lastCfoHz is valid
    end

    methods
        function obj = FftCfoEstimator(varargin)
            % Constructor with name-value pairs
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
            % RESET HISTORY of the CFO tracker
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
