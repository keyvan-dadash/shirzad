classdef RootRaisedCosineFilter < filters.AbstractFirFilter
    % Square-root raised cosine FIR filter.
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

            b = filters.RootRaisedCosineFilter.designCoeffs(beta, span, sps, normalizeEnergy);

            obj@filters.AbstractFirFilter(b, 'Root Raised Cosine');
            obj.Rolloff          = beta;
            obj.SpanInSymbols    = span;
            obj.SamplesPerSymbol = sps;
        end
    end

    methods (Static, Access = private)
        function h = designCoeffs(beta, span, sps, normalizeEnergy)
            % Based on standard root-raised cosine impulse response formula with
            % symbol period T = 1.
            % https://en.wikipedia.org/wiki/Root-raised-cosine_filter

            % Total length in samples: L = span*sps + 1
            L = span * sps + 1;
            n = -(L-1)/2 : (L-1)/2;     % symmetric indices (from -span/2 to +span/2)
            t = n / sps;                % normalized time, Tsym = 1

            h = zeros(size(t));

            Ts = 1;   % normalized symbol period

            % Handle special cases t = 0 and t = ±Ts/(4*beta) base on the
            % formula. We use small tolerance for comparisons
            tol = 1e-8;

            % t == 0
            idx0 = abs(t) < tol;
            if any(idx0)
                h(idx0) = (1 + beta*(4/pi - 1));   % 1/Ts * (1 + β(4/π -1)), Ts=1
            end

            % t == (+-) Ts/(4*beta)
            t1 = Ts / (4*beta);
            idx1 = abs(abs(t) - t1) < tol;
            if any(idx1)
                h(idx1) = (beta / sqrt(2)) * ...
                    ( (1 + 2/pi) * sin(pi/(4*beta)) + ...
                      (1 - 2/pi) * cos(pi/(4*beta)) );
            end

            % General case (all other t)
            idxGen = ~(idx0 | idx1);
            tg = t(idxGen);

            % Handle the rest of cases
            if ~isempty(tg)
                num = sin(pi * tg * (1 - beta)) + ...
                      4 * beta .* tg .* cos(pi * tg * (1 + beta));
                den = pi * tg .* (1 - (4*beta.*tg).^2);

                h(idxGen) = num ./ den;
            end

            if normalizeEnergy
                h = h / sqrt(sum(abs(h).^2));
            end

            h = h(:).'; % row based vector
        end
    end
end
