classdef (Abstract) AbstractDemodulator < handle
    % Every demodulator should implement this class for unpacking symbols
    % to bits.

    properties (SetAccess = protected)
        M
        BitsPerSymbol
        Name
    end

    methods
        function obj = AbstractDemodulator(M, name)
            % Constructor
            obj.M = M;
            obj.BitsPerSymbol = log2(M);
            obj.Name = char(name);
        end

        function G = getAmbiguityRotations(obj) %#ok<INUSD>
            %GETAMBIGUITYROTATIONS
            % Default: no rotational ambiguity → single rotation 1+0j.
            % Specific demodulators (QPSK, QAM, etc.) can override.
            G = 1;
        end

        function [rxSymsFixed, bestIdx, errs] = resolvePhaseAmbiguity(obj, rxSymsEq, pilotBits)
            %RESOLVEPHASEAMBIGUITY Use known pilot bits to fix constellation
            % rotation ambiguity.
            %
            %   [rxSymsFixed, bestIdx, errs] = obj.resolvePhaseAmbiguity( ...
            %       rxSymsEq, pilotBits)
            %
            % rxSymsEq : complex column (equalized symbols, including pilot part)
            % pilotBits: known pilot bits (column vector 0/1)
            %
            % rxSymsFixed : rotated symbols after ambiguity resolution
            % bestIdx     : index of best rotation in getAmbiguityRotations()
            % errs        : vector of pilot bit error rates for each rotation

            if isempty(pilotBits) || isempty(rxSymsEq)
                % Nothing to do
                rxSymsFixed = rxSymsEq;
                bestIdx     = 1;
                errs        = [];
                return;
            end

            numOfPilotBitsSym = floor(numel(pilotBits) / obj.BitsPerSymbol);
            z = rxSymsEq(:);
            zPilot = rxSymsEq(1:numOfPilotBitsSym + 1, :);
            G = obj.getAmbiguityRotations();
            G = G(:).';               % row vector
            nRot = numel(G);

            errs = zeros(1, nRot);
            for g = 1:nRot
                % Apply candidate rotation
                zRot = zPilot * G(g);

                % Demodulate to bits
                rb = obj.demodulateHard(zRot);

                % Compare first Kc bits with known pilot bits
                Kc = min(numel(rb), numel(pilotBits));
                if Kc == 0
                    errs(g) = 1;      % no pilot bits => treat as worst
                else
                    errs(g) = mean(rb(1:Kc) ~= pilotBits(1:Kc));
                end
            end

            [~, bestIdx] = min(errs);
            rxSymsFixed  = z * G(bestIdx);
        end
    end

    methods (Abstract)
        bits = demodulateHard(obj, symbols);
        llr  = demodulateLlr(obj, symbols, noiseVarPerDim);
    end
end
