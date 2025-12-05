classdef SimpleAgc < handle
    % Simple block-AGC with exponential power averaging.

    properties
        AveragingLength    = 1000;
        MaximumGain_dB     = 30;
        AdaptationStepSize = 1e-3;
        TargetPower        = 1.0;
    end

    properties (Access = private)
        avgPower   (1,1) single = 1.0;   % smoothed power estimate
        gainLinear (1,1) single = 1.0;   % current applied linear gain
    end

    methods
        function obj = SimpleAgc(varargin)
            if ~isempty(varargin)
                p = inputParser;
                addParameter(p,'AveragingLength',    obj.AveragingLength);
                addParameter(p,'MaximumGain_dB',     obj.MaximumGain_dB);
                addParameter(p,'AdaptationStepSize', obj.AdaptationStepSize);
                addParameter(p,'TargetPower',        obj.TargetPower);
                parse(p, varargin{:});
                cfg = p.Results;

                obj.AveragingLength    = cfg.AveragingLength;
                obj.MaximumGain_dB     = cfg.MaximumGain_dB;
                obj.AdaptationStepSize = cfg.AdaptationStepSize;
                obj.TargetPower        = cfg.TargetPower;
            end

            obj.reset();
        end

        function reset(obj)
            obj.avgPower   = obj.TargetPower;
            obj.gainLinear = 1.0;
        end

        function y = process(obj, x)
            if isempty(x)
                y = x;
                return;
            end

            wasRow = isrow(x);
            x = x(:);

            alpha = 1 / obj.AveragingLength; 
            instPow = mean(abs(x).^2);

            obj.avgPower = (1 - alpha)*obj.avgPower + alpha*instPow;
            if obj.avgPower <= 0
                obj.avgPower = eps;
            end

            idealGain = sqrt(obj.TargetPower / obj.avgPower);

            mu = obj.AdaptationStepSize;
            if mu > 0
                obj.gainLinear = (1 - mu)*obj.gainLinear + mu*idealGain;
            end

            maxGainLinear = 10^(obj.MaximumGain_dB/20);
            if obj.gainLinear > maxGainLinear
                obj.gainLinear = maxGainLinear;
            end

            y = obj.gainLinear * x;

            if wasRow
                y = y.';
            end
        end

        function g = getCurrentGain_dB(obj)
            g = 20*log10(max(obj.gainLinear, eps));
        end
    end
end
