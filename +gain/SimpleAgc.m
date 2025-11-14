classdef SimpleAgc < handle
    %SIMPLEAGC Simple block-AGC with exponential power averaging.
    %
    %   y = obj.process(x) scales x so that its *average* power
    %   (tracked over time) approaches TargetPower.
    %
    %   Parameters (name/value in ctor):
    %       'AveragingLength'   : effective length of power averaging window
    %                             (e.g. 1000 samples)
    %       'MaximumGain_dB'    : max allowed gain in dB (e.g. 30)
    %       'AdaptationStepSize': 0..1, how fast gain moves toward ideal
    %                             (e.g. 1e-3, set ~0 to freeze)
    %       'TargetPower'       : desired average power (default 1.0)
    %
    %   This is a lightweight replacement for comm.AGC for complex baseband.

    properties
        AveragingLength    (1,1) double {mustBePositive}      = 1000;
        MaximumGain_dB     (1,1) double {mustBeNonnegative}   = 30;
        AdaptationStepSize (1,1) double {mustBeNonnegative}   = 1e-3;
        TargetPower        (1,1) double {mustBePositive}      = 1.0;
    end

    properties (Access = private)
        avgPower   (1,1) double = 1.0;   % smoothed power estimate
        gainLinear (1,1) double = 1.0;   % current applied linear gain
    end

    methods
        function obj = SimpleAgc(varargin)
            % SimpleAgc('AveragingLength',1000, 'MaximumGain_dB',30, ...)
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
            %RESET Reset AGC state: power estimate and gain.
            obj.avgPower   = obj.TargetPower;
            obj.gainLinear = 1.0;
        end

        function y = process(obj, x)
            %PROCESS Apply AGC to input block x.
            %
            %   x : column or row vector (complex or real)
            %   y : same size as x, scaled

            if isempty(x)
                y = x;
                return;
            end

            % work with column internally
            wasRow = isrow(x);
            x = x(:);

            % ---- 1) update smoothed power estimate ----
            alpha = 1 / obj.AveragingLength;           % ~1/N smoothing
            instPow = mean(abs(x).^2);                 % block average

            obj.avgPower = (1 - alpha)*obj.avgPower + alpha*instPow;
            if obj.avgPower <= 0
                obj.avgPower = eps;
            end

            % ---- 2) ideal gain to hit target power ----
            idealGain = sqrt(obj.TargetPower / obj.avgPower);

            % ---- 3) first-order adaptation of internal gain ----
            mu = obj.AdaptationStepSize;
            if mu > 0
                obj.gainLinear = (1 - mu)*obj.gainLinear + mu*idealGain;
            end

            % ---- 4) enforce maximum gain ----
            maxGainLinear = 10^(obj.MaximumGain_dB/20);
            if obj.gainLinear > maxGainLinear
                obj.gainLinear = maxGainLinear;
            end

            % ---- 5) apply gain ----
            y = obj.gainLinear * x;

            if wasRow
                y = y.';
            end
        end

        function g = getCurrentGain_dB(obj)
            %GETCURRENTGAIN_DB Return current AGC gain in dB.
            g = 20*log10(max(obj.gainLinear, eps));
        end
    end
end
