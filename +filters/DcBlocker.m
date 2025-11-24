classdef DcBlocker < handle
    % Simple DC blocker using exponential running-average subtraction.

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
            obj.meanState = 0;
        end

        function y = process(obj, x)
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
