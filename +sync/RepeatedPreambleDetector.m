% File: +sync/RepeatedPreambleDetector.m
classdef RepeatedPreambleDetector < handle
    % RepeatedPreambleDetector
    %
    % Detects a preamble of the form [a, a] at symbol-rate, given a
    % sample-rate complex baseband stream y[n].
    %
    % Uses a Schmidl-style metric:
    %   P(k) = sum_{n=0}^{Lh-1} y_{k+n+Lh} * conj(y_{k+n})
    %   R(k) = sum_{n=0}^{2Lh-1} |y_{k+n}|^2
    %   M(k) = |P(k)|^2 / (R(k)^2 + eps)
    %
    % It tries all sample offsets 0..SamplesPerSymbol-1, down-samples to
    % 1-sps, and searches over k. Returns the best offset and symbol index.
    %
    % Properties (name-value in constructor):
    %   SamplesPerSymbol : integer sps
    %   PreambleHalfLen  : length of one half a (in symbols)
    %   MetricThreshold  : minimum M(k) to declare Found=true
    %   MinWindowPower   : minimum R(k) to avoid pure noise triggers
    %
    % Method:
    %   res = detect(y)
    %       y : column vector of complex samples at Fs
    %
    %   res is a struct with fields:
    %       Found            : logical
    %       Metric           : best M(k)
    %       WindowPower      : best R(k)
    %       SampleOffset     : best sample offset (0..sps-1)
    %       PreambleStartSym : best symbol index (1-based at 1 sps)

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
            % DETECT  Run Schmidl-style repeated-preamble detection
            %
            % res = obj.detect(y)
            %
            % y : complex column vector of samples at Fs

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

            % Try all possible sample offsets 0..sps-1
            for off = 0:(sps-1)
                ySym = y(1+off : sps : end);  % 1 sample per symbol
                Ns   = numel(ySym);
                if Ns < Lpre + 1
                    continue;
                end

                % Sliding window over possible starting symbol indices k
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

                    % fprintf('1 is %.3f\n', M(1));
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
