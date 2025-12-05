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

        function G = getAmbiguityRotations(obj)
            G = 1;
        end

        function [rxSymsFixed, bestIdx, errs] = resolvePhaseAmbiguity(obj, rxSymsEq, pilotBits)
            % Use known pilot bits to fix constellation
            % rotation ambiguity.

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
                    rbK     = rb(1:Kc);                % one slice
                    pilotK  = pilotBits(1:Kc);         % one slice
                    
                    % Faster than 'mean'
                    mismatches = 0;
                    for k = 1:Kc
                        if rbK(k) ~= pilotK(k)
                            mismatches = mismatches + 1;
                        end
                    end
                    errs(g) = mismatches / Kc;
                    % fprintf('------------------------Start %d %d %d %d-----------------------\n', g, numel(rb), numel(pilotBits), Kc);
                    % for k = 1: 4 :Kc
                    %     fprintf('%d %d %d %d | ', rb(k), rb(k+1), rb(k+2), rb(k+3));
                    % end
                    % fprintf('\n');
                    % for k = 1: 4 :Kc
                    %     fprintf('%d %d %d %d | ', pilotBits(k), pilotBits(k+1), pilotBits(k+2), pilotBits(k+3));
                    % end
                    % fprintf('\n');
                    % fprintf('------------------------End   %d-----------------------\n', g);
                end
            end

            [~, bestIdx] = min(errs);
            rxSymsFixed  = z * G(bestIdx);
        end
    end

    methods (Abstract)
        bits = demodulateHard(obj, symbols);
    end
end
