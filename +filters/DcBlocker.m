classdef DcBlocker < handle
    %DCBLOCKER Simple DC blocker using exponential running-average subtraction.
    %
    %   y = obj.process(x) removes (slow) DC offset from x by tracking
    %   a smoothed mean and subtracting it:
    %
    %       mean_n = (1 - alpha)*mean_{n-1} + alpha*x[n]
    %       y[n]   = x[n] - mean_n
    %
    %   where alpha ≈ 1/Length.
    %
    %   Parameters (name/value in ctor):
    %       'Length' : effective averaging length (e.g. 64)
    %
    %   Usage:
    %       dc = filters.DcBlocker('Length',64);
    %       y  = dc.process(x);
    %
    %   State (mean estimate) is kept across calls for streaming use.

    properties
        Length (1,1) double {mustBePositive} = 64;
    end

    properties (Access = private)
        alpha     (1,1) double = 1/64;   % smoothing factor
        meanState (1,1) double = 0;      % complex DC estimate
    end

    methods
        function obj = DcBlocker(varargin)
            if ~isempty(varargin)
                p = inputParser;
                addParameter(p,'Length',obj.Length, ...
                    @(x)isnumeric(x)&&isscalar(x)&&x>0);
                parse(p, varargin{:});
                obj.Length = p.Results.Length;
            end
            obj.alpha = 1 / obj.Length;
            obj.reset();
        end

        function reset(obj)
            %RESET Reset internal DC estimate.
            obj.meanState = 0;
        end

        function y = process(obj, x)
            %PROCESS Remove DC from x (vector, real or complex).
            %
            %   x : row or column vector
            %   y : same size as x, with DC removed

            if isempty(x)
                y = x;
                return;
            end

            wasRow = isrow(x);
            x = x(:);

            N = numel(x);
            y = zeros(N,1,'like',x);

            m = obj.meanState;
            a = obj.alpha;

            for n = 1:N
                m = (1 - a)*m + a*x(n);
                y(n) = x(n) - m;
            end

            obj.meanState = m;   % carry state to next block

            if wasRow
                y = y.';
            end
        end
    end
end
