classdef RootRaisedCosineFilter < filters.AbstractFirFilter
    %ROOTRAISEDCOSINEFILTER Square-root raised cosine FIR filter.
    %
    %   h[n] is designed for:
    %       - rolloff factor beta (0 < beta <= 1)
    %       - filter span in symbols (span)
    %       - samples per symbol (sps)
    %
    %   Time is normalized with T_sym = 1, so t = n/sps.
    %
    %   Usage (TX):
    %       txRRC = filters.RootRaisedCosineFilter(beta, span, sps);
    %       up = zeros(N*sps,1); up(1:sps:end) = symbols;
    %       y  = txRRC.process(up);
    %
    %   Usage (RX):
    %       rxRRC = filters.RootRaisedCosineFilter(beta, span, sps);
    %       y     = rxRRC.process(x);

    properties (SetAccess = private)
        Rolloff           % beta
        SpanInSymbols     % span
        SamplesPerSymbol  % sps
    end

    methods
        function obj = RootRaisedCosineFilter(beta, span, sps, normalizeEnergy)
            if nargin < 4
                normalizeEnergy = true;
            end

            if beta <= 0 || beta > 1
                error('RootRaisedCosineFilter:Beta', ...
                      'Rolloff factor beta must be in (0,1].');
            end
            if span <= 0 || mod(span,1) ~= 0
                error('RootRaisedCosineFilter:Span', ...
                      'FilterSpanInSymbols must be a positive integer.');
            end
            if sps <= 0 || mod(sps,1) ~= 0
                error('RootRaisedCosineFilter:Sps', ...
                      'SamplesPerSymbol must be a positive integer.');
            end

            b = filters.RootRaisedCosineFilter.designCoeffs(beta, span, sps, normalizeEnergy);

            obj@filters.AbstractFirFilter(b, 'Root Raised Cosine');
            obj.Rolloff          = beta;
            obj.SpanInSymbols    = span;
            obj.SamplesPerSymbol = sps;
        end
    end

    methods (Static, Access = private)
        function h = designCoeffs(beta, span, sps, normalizeEnergy)
            %DESIGNCOEFFS Generate RRC taps using closed-form impulse response.
            %
            % Based on standard root-raised cosine impulse response formula with
            % symbol period T = 1. 

            % Total length in samples (odd): L = span*sps + 1
            L = span * sps + 1;
            n = -(L-1)/2 : (L-1)/2;     % symmetric indices
            t = n / sps;                % normalized time, Tsym = 1

            h = zeros(size(t));

            Ts = 1;   % normalized symbol period

            % Handle special cases t = 0 and t = ±Ts/(4*beta)
            % Use small tolerance for comparisons
            tol = 1e-8;

            % t == 0
            idx0 = abs(t) < tol;
            if any(idx0)
                h(idx0) = (1 + beta*(4/pi - 1));   % 1/Ts * (1 + β(4/π -1)), Ts=1
            end

            % t == ± Ts/(4*beta)
            if beta ~= 0
                t1 = Ts / (4*beta);
                idx1 = abs(abs(t) - t1) < tol;
                if any(idx1)
                    h(idx1) = (beta / sqrt(2)) * ...
                        ( (1 + 2/pi) * sin(pi/(4*beta)) + ...
                          (1 - 2/pi) * cos(pi/(4*beta)) );
                end
            else
                idx1 = false(size(t));
            end

            % General case (all other t)
            idxGen = ~(idx0 | idx1);
            tg = t(idxGen);

            if ~isempty(tg)
                % Standard formula for RRC with Ts = 1:
                % h(t) = [ sin(pi t (1-β)) + 4β t cos(pi t (1+β)) ] /
                %        [ pi t (1 - (4β t)^2) ]
                num = sin(pi * tg * (1 - beta)) + ...
                      4 * beta .* tg .* cos(pi * tg * (1 + beta));
                den = pi * tg .* (1 - (4*beta.*tg).^2);

                h(idxGen) = num ./ den;
            end

            % Normalize energy if requested
            if normalizeEnergy
                h = h / sqrt(sum(abs(h).^2));
            end

            h = h(:).';   % row vector
        end
    end
end
