classdef RepeatedPreambleDetector < handle
    % RepeatedPreambleDetector is a preamble detector that implements
    % Schmidl & Cox technique. This technique forms a preamble that has two
    % repeated preamble: [a, a].
    %
    % Base on the M(k), we can detect the preamble of a frame. Before that,
    % we need to calculate P(d) which is:
    %   P(d) = r[d + m]*r[d + m + L] (from m = 0 until m = L - 1)
    % Also we need to calculate the energy of received signal on that
    % window, which is:
    %   R(d) = |r[d + m + K]| ^ 2 (from m = 0 until m = L - 1)
    %
    % Therefore, M(d) is:
    %   M(d) = |P(d)| ^ 2 / R(d) ^ 2
    %
    % Based on our calculation M(d) in this project is aournd 0.25 (and
    % also other projects)

    properties
        SamplesPerSymbol = 10;
        PreambleHalfLen  = 32;
        MetricThreshold  = 0.2;
        MinWindowPower   = 1e-6;
    end

    methods
        function obj = RepeatedPreambleDetector(varargin)
            % Constructor with name-value pairs
            if mod(numel(varargin),2) ~= 0
                error('RepeatedPreambleDetector:NameValue', ...
                      'Constructor expects name-value pairs.');
            end
            for k = 1:2:numel(varargin)
                name  = varargin{k};
                value = varargin{k+1};
                switch lower(name)
                    case 'samplespersymbol'
                        obj.SamplesPerSymbol = value;
                    case 'preamblehalflen'
                        obj.PreambleHalfLen  = value;
                    case 'metricthreshold'
                        obj.MetricThreshold  = value;
                    case 'minwindowpower'
                        obj.MinWindowPower   = value;
                    otherwise
                        error('RepeatedPreambleDetector:UnknownParam', ...
                              'Unknown parameter "%s".', name);
                end
            end
        end

        function res = detect(obj, y)
            sps  = obj.SamplesPerSymbol;
            Lh   = obj.PreambleHalfLen;
            Lpre = 2 * Lh;

            best.Metric           = 0;
            best.SampleOffset     = 0;
            best.PreambleStartSym = 0;
            best.WindowPower      = 0;
            best.Found            = false;

            if isempty(y)
                res = best;
                return;
            end

            % For all different sample offset
            for off = 0:(sps-1)
                ySym = y(1+off : sps : end);
                Ns   = numel(ySym);
                if Ns < Lpre + 1
                    continue;
                end

                % The window that we can slide our detection on
                Lwin = Ns - 2*Lh;
                if Lwin <= 0
                    continue;
                end

                P = complex(zeros(Lwin,1));
                R = zeros(Lwin,1);

                for k = 1:Lwin
                    idxA = k : k+Lh-1;
                    idxB = k+Lh : k+2*Lh-1;

                    a = ySym(idxA);
                    b = ySym(idxB);

                    P(k) = sum(b .* conj(a));
                    seg  = ySym(k : k+2*Lh-1);
                    R(k) = sum(abs(seg).^2);
                end

                M = abs(P).^2 ./ (R.^2 + eps);

                [Mmax, idxMax] = max(M);
                if Mmax > best.Metric && R(idxMax) > obj.MinWindowPower
                    best.Metric           = Mmax;
                    best.SampleOffset     = off;
                    best.PreambleStartSym = idxMax;  % 1-based
                    best.WindowPower      = R(idxMax);
                end
            end

            if best.Metric > obj.MetricThreshold && ...
               best.WindowPower > obj.MinWindowPower
                best.Found = true;
            else
                best.Found = false;
            end

            res = best;
        end
    end
end
