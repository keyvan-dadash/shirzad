classdef FastDcBlocker < handle
    % Fast DC blocker using exponential running-average subtraction.

    properties
        Length (1,1) single {mustBePositive} = 64;
    end

    properties (Access = private)
        alpha     (1,1) single = 1/64;   % smoothing factor
        meanState (1,1) single = 0;      % complex DC estimate
    end

    methods
        function obj = FastDcBlocker(varargin)
            if ~isempty(varargin)
                p = inputParser;
                addParameter(p,'Length',obj.Length);
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
            % Basically, we use filter pre-built function to improve the
            % speed of dcblcoker.
            if isempty(x)
                y = x;
                return;
            end

            wasRow = isrow(x);
            x = x(:);   % column

            a = obj.alpha;
            % DC running average: m[n] = a x[n] + (1-a) m[n-1]
            % -> filter with b = a, a = [1, -(1-a)]
            b = a;
            A = [1, -(1-a)];

            % z0 is the previous output
            [m, zf] = filter(b, A, x, obj.meanState);

            % Subtract DC
            y = x - m;

            % Update state
            obj.meanState = zf;

            if wasRow
                y = y.';
            end
        end
    end
end
