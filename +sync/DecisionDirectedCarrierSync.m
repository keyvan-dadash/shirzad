classdef DecisionDirectedCarrierSync < handle
    %DECISIONDIRECTEDCARRIERSYNC Simple decision-directed carrier/phase PLL.
    %
    %   y = obj.process(x) applies carrier/phase correction to input x
    %   (complex column vector) using a decision-directed loop.
    %
    %   Parameters:
    %       ModulationOrder        : 4 (QPSK), 16, ... (basic QAM support)
    %       SamplesPerSymbol       : typically 1 here
    %       DampingFactor          : ζ, e.g., 0.707
    %       NormalizedLoopBandwidth: Bn (normalized to symbol rate),
    %                                e.g., 0.01 (coarse), 0.002 (fine)
    %
    %   Internal state:
    %       phase, freq           : NCO state
    %       Kp, Ki                : loop filter gains
    %
    %   This roughly mimics comm.CarrierSynchronizer with
    %   'InputSamplesPerSymbol' = 1 and 'Modulation' = 'QAM'.

    properties
        ModulationOrder        (1,1) double {mustBePositive,mustBeInteger} = 4
        SamplesPerSymbol       (1,1) double {mustBePositive} = 1
        DampingFactor          (1,1) double {mustBePositive} = 0.707
        NormalizedLoopBandwidth(1,1) double {mustBePositive} = 0.01
    end

    properties (Access = private)
        phase   (1,1) double = 0;   % current NCO phase (rad)
        freq    (1,1) double = 0;   % current NCO frequency (rad/sym)
        Kp      (1,1) double = 0;   % proportional gain
        Ki      (1,1) double = 0;   % integral gain
    end

    methods
        function obj = DecisionDirectedCarrierSync(varargin)
            % Parse name/value pairs
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

        function reset(obj)
            %RESET Reset NCO state
            obj.phase = 0;
            obj.freq  = 0;
        end

        function y = process(obj, x)
            %PROCESS Apply carrier/phase correction to input x.
            %
            %   x : column vector of complex symbols
            %   y : column vector of corrected symbols

            x = x(:);               % ensure column
            N = numel(x);
            y = complex(zeros(N,1));

            for k = 1:N
                % NCO output: e^{-j*phase}
                rot = exp(-1j * obj.phase);

                % Apply correction
                yk = x(k) * rot;
                y(k) = yk;

                % Decision on constellation
                dk = obj.decision(yk);

                % Phase error: angle between corrected symbol and decision
                % e_k in range (-pi, pi)
                if dk ~= 0
                    e_k = angle(yk * conj(dk));
                else
                    e_k = 0;
                end

                % 2nd-order PLL update
                obj.freq  = obj.freq  + obj.Ki * e_k;
                obj.phase = obj.phase + obj.freq + obj.Kp * e_k;
            end

            % fprintf("f: %.3f and p: %.3f\n", obj.freq, obj.phase);
        end
    end

    methods (Access = private)
        function configureLoopGains(obj)
            %CONFIGURELOOPGAINS Compute Kp, Ki from Bn and ζ.
            %
            % Using standard 2nd-order digital PLL design formulae, assuming
            % unity phase detector gain and NCO gain, and sampling at
            % 1 sample/symbol.

            Bn = obj.NormalizedLoopBandwidth;  % normalized to symbol rate
            zeta = obj.DampingFactor;

            % Avoid pathological values
            if Bn <= 0
                Bn = 1e-4;
            end

            % These formulas are adapted from common digital PLL design
            % (e.g. Gardner).
            theta = Bn / (zeta + 0.25/zeta);
            d = 1 + 2*zeta*theta + theta^2;

            obj.Kp = (4*zeta*theta) / d;
            obj.Ki = (4*theta^2)    / d;
        end

        function d = decision(obj, y)
            %DECISION Hard decision on QAM constellation (Gray-ish).
            %
            % For now, we implement QPSK explicitly and a simple rectangular
            % QAM grid for higher orders.

            M = obj.ModulationOrder;

            if M == 4
                % QPSK: map to {±1 ± j}/sqrt(2)
                re = real(y);
                im = imag(y);

                if re >= 0
                    reHat = 1;
                else
                    reHat = -1;
                end
                if im >= 0
                    imHat = 1;
                else
                    imHat = -1;
                end
                d = (reHat + 1j*imHat) / sqrt(2);

            else
                % Generic square QAM, M = L^2
                L = sqrt(M);
                if abs(L - round(L)) > eps
                    error('DecisionDirectedCarrierSync:ModulationOrder', ...
                          'Only square QAM (M = L^2) supported for M>4.');
                end

                % Scale: assume average symbol energy ~1, so levels roughly
                % at ±1, ±3, ...
                m = (0:L-1) - (L-1)/2;
                levels = 2*m;             % e.g. L=4 => [-3 -1 1 3]

                reHat = DecisionDirectedCarrierSync.quantize(real(y), levels);
                imHat = DecisionDirectedCarrierSync.quantize(imag(y), levels);

                % Normalize so that average power ~1
                EsAvg = mean(levels.^2);
                d = (reHat + 1j*imHat) / sqrt(2*EsAvg);
            end
        end
    end

    methods (Static, Access = private)
        function vq = quantize(x, levels)
            %QUANTIZE Nearest-neighbor quantizer to set "levels".
            [~, idx] = min(abs(x - levels.'), [], 2);
            vq = levels(idx);
        end
    end
end
