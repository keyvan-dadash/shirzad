classdef RepeatedPreambleDetector < sync.AbstractPreambleDetector
    %REPEATEDPREAMBLEDETECTOR
    %
    % Detects a preamble of the form [a, a] where each half has
    % PreambleHalfLen symbols.
    %
    % Uses a Schmidl-like metric:
    %
    %   P(k) = sum_{n=0}^{L-1} y[k+L+n] * conj(y[k+n])
    %   R(k) = sum_{n=0}^{L-1} (|y[k+n]|^2 + |y[k+L+n]|^2)
    %   M(k) = |P(k)|^2 / (R(k)^2 + eps)
    %
    % CFO estimate:
    %
    %   wSym_hat = angle(P(k0)) / L
    %
    % where k0 is the index that maximizes M(k).
    %
    % Result struct:
    %   - Found            : true/false
    %   - SampleOffset     : best sample offset in [0..sps-1]
    %   - PreambleStartSym : index (1-based) in symbol-rate sequence
    %   - CfoRadPerSym     : estimated CFO (rad/symbol)
    %   - Metric           : Mmax
    %   - WindowPower      : R(k0)
    %
    % NOTE:
    %   This detector works at *symbol rate*. It internally tries all
    %   sps offsets on the *sample-rate* yBuf, down-samples to 1 sps, and
    %   scans for the best M(k). It assumes the preamble is fully inside
    %   yBuf.

    properties
        MetricThreshold   (1,1) double = 0.2    % gate on M(k)
        MinWindowPower    (1,1) double = 1e-6   % gate on R(k)
    end

    methods
        function obj = RepeatedPreambleDetector(varargin)
            % RepeatedPreambleDetector('SamplesPerSymbol',10, ...
            %                         'PreambleHalfLen',64, ...
            %                         'MetricThreshold',0.2)
            p = inputParser;
            p.addParameter('SamplesPerSymbol', 10, @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('PreambleHalfLen',  64, @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('MetricThreshold',  0.2, @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('MinWindowPower',   1e-6, @(x)isnumeric(x)&&isscalar(x));
            p.parse(varargin{:});
            cfg = p.Results;

            obj@sync.AbstractPreambleDetector( ...
                'RepeatedPreambleDetector', ...
                cfg.SamplesPerSymbol, ...
                cfg.PreambleHalfLen);

            obj.MetricThreshold = cfg.MetricThreshold;
            obj.MinWindowPower  = cfg.MinWindowPower;
        end

        function result = detect(obj, yBuf)
            sps  = obj.SamplesPerSymbol;
            L    = obj.PreambleHalfLen;
            Ltot = 2*L;

            bestMetric = -inf;
            bestOff    = 0;
            bestK      = 0;
            bestP      = 0;
            bestR      = 0;
            foundAny   = false;

            N = numel(yBuf);
            if N < Ltot * sps
                result = obj.makeEmptyResult();
                return;
            end

            % Try each fractional-sample offset (0..sps-1)
            for off = 0:(sps-1)
                ySym = yBuf(1+off : sps : end);   % symbol-rate sequence
                Ns   = numel(ySym);
                if Ns < Ltot + 4
                    continue;
                end

                maxK = Ns - Ltot + 1;
                for k = 1:maxK
                    seg1 = ySym(k       : k+L-1);
                    seg2 = ySym(k+L     : k+2*L-1);

                    Pk = sum(seg2 .* conj(seg1));   % complex correlation between halves
                    Rk = sum(abs(seg1).^2 + abs(seg2).^2);

                    Mk = (abs(Pk)^2) / (Rk^2 + eps);

                    if Mk > bestMetric
                        bestMetric = Mk;
                        bestOff    = off;
                        bestK      = k;
                        bestP      = Pk;
                        bestR      = Rk;
                        foundAny   = true;
                    end
                end
            end

            if ~foundAny ...
               || bestMetric < obj.MetricThreshold ...
               || bestR < obj.MinWindowPower

                result = obj.makeEmptyResult();
                return;
            end

            % CFO estimate (rad/symbol)
            wSym_hat = angle(bestP) / L;

            result = struct( ...
                'Found',            true, ...
                'SampleOffset',     bestOff, ...
                'PreambleStartSym', bestK, ...
                'CfoRadPerSym',     wSym_hat, ...
                'Metric',           bestMetric, ...
                'WindowPower',      bestR);
        end
    end

    methods (Access = private)
        function r = makeEmptyResult(obj)
            %#ok<INUSD>
            r = struct( ...
                'Found',            false, ...
                'SampleOffset',     0, ...
                'PreambleStartSym', 0, ...
                'CfoRadPerSym',     0, ...
                'Metric',           0, ...
                'WindowPower',      0);
        end
    end

end
