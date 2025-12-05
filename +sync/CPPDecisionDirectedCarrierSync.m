classdef CPPDecisionDirectedCarrierSync < handle
    %CPPDecisionDirectedCarrierSync The PLL in the c++

    properties
        ModulationOrder        (1,1) double {mustBePositive, mustBeInteger} = 4
        SamplesPerSymbol       (1,1) double {mustBePositive} = 1
        DampingFactor          (1,1) double {mustBePositive} = 0.707
        NormalizedLoopBandwidth(1,1) double {mustBePositive} = 0.01
        phase   (1,1) double = 0;   % current phase
        freq    (1,1) double = 0;   % current frequency
    end

    properties (Access = private)
        Kp      (1,1) double = 0;   % proportional gain
        Ki      (1,1) double = 0;   % integral gain
    end

    methods
        function obj = CPPDecisionDirectedCarrierSync(varargin)
            if ~isempty(varargin)
                p = inputParser;
                addParameter(p, 'ModulationOrder',         obj.ModulationOrder);
                addParameter(p, 'SamplesPerSymbol',        obj.SamplesPerSymbol);
                addParameter(p, 'DampingFactor',           obj.DampingFactor);
                addParameter(p, 'NormalizedLoopBandwidth', obj.NormalizedLoopBandwidth);
                parse(p, varargin{:});
                cfg = p.Results;

                obj.ModulationOrder         = cfg.ModulationOrder;
                obj.SamplesPerSymbol        = cfg.SamplesPerSymbol;
                obj.DampingFactor           = cfg.DampingFactor;
                obj.NormalizedLoopBandwidth = cfg.NormalizedLoopBandwidth;
            end

            obj.configureLoopGains();
        end

        function reset(obj, phase, freq)
            obj.phase = phase;
            obj.freq  = freq;
        end

        function y = process(obj, x)
            [y, phaseOut, freqOut] = sync.decisionDirectedCarrierSyncMex( ...
                x, obj.ModulationOrder, obj.Kp, obj.Ki, obj.phase, obj.freq);
            obj.phase = phaseOut;
            obj.freq  = freqOut;
        end
    end

    methods (Access = private)
        function configureLoopGains(obj)
            Bn   = obj.NormalizedLoopBandwidth;
            zeta = obj.DampingFactor;

            if Bn <= 0
                Bn = 1e-4;
            end

            theta = Bn / (zeta + 0.25/zeta);
            d     = 1 + 2*zeta*theta + theta^2;

            obj.Kp = (4*zeta*theta) / d;
            obj.Ki = (4*theta^2)    / d;
        end
    end
end
