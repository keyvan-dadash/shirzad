classdef ViterbiDecoder < handle
    % Hard-decision Viterbi decoder for binary convolutional code.
    %
    % Uses same generator structure as ConvEncoder:
    %   - G: [nOut x K] binary matrix
    %   - rate = 1 / nOut
    %
    % Assumes "terminated" encoding: encoder started in all-zero state
    % and appended K-1 zeros at the end. This decoder:
    %   - assumes initial state = 0
    %   - chooses best final state (usually 0)
    %   - RETURNS ONLY THE INFORMATION BITS (it drops last K-1 bits).

    properties
        G              % [nOut x K] binary
        K              % constraint length
        nOut           % outputs per input bit
        Memory         % K-1
        NumStates      % 2^(K-1)
        StateBits      % [NumStates x Memory] representation of each state
        NextState      % [NumStates x 2] for input bit 0/1
        OutputBits     % [NumStates x 2 x nOut] expected outputs
    end

    methods
        function obj = ViterbiDecoder(G)
            % G: [nOut x K] binary matrix
            if nargin < 1
                G = [1 1 1;
                     1 0 1];  % same as ConvEncoder default
            end
            obj.G      = double(G ~= 0);
            obj.nOut   = size(obj.G,1);
            obj.K      = size(obj.G,2);
            obj.Memory = obj.K - 1;
            obj.NumStates = 2^(obj.Memory);

            % Enumerate all states as binary vectors (u(k-1),...,u(k-K+1))
            obj.StateBits = zeros(obj.NumStates, obj.Memory);
            for s = 0:obj.NumStates-1
                obj.StateBits(s+1,:) = de2bi(s, obj.Memory, 'left-msb');
            end

            % Precompute transition tables
            obj.NextState  = zeros(obj.NumStates, 2, 'uint16');
            obj.OutputBits = zeros(obj.NumStates, 2, obj.nOut);

            GD = obj.G;
            for sIdx = 1:obj.NumStates
                mem = obj.StateBits(sIdx,:);   % 1 x (K-1)
                for b = 0:1
                    % Build full register for this transition
                    reg = [b, mem];            % 1 x K

                    % Output bits for this transition
                    out = mod(reg * GD.', 2);  % 1 x nOut

                    % Next state's memory bits: [b, mem(1:end-1)]
                    newMem = reg(1:end-1);
                    newStateIdx = uint16(bi2de(newMem, 'left-msb') + 1);

                    obj.NextState(sIdx, b+1) = newStateIdx;
                    obj.OutputBits(sIdx, b+1, :) = out;
                end
            end
        end

        function uHat = decode(obj, v)
            % Decode hard bits with Viterbi.
            %
            % v : column or row vector of {0,1}; length must be multiple of nOut
            %
            % uHat : column vector of estimated INFO bits (tail bits removed)

            v = v(:).';
            v = double(v ~= 0);
            nOut = obj.nOut;

            if mod(numel(v), nOut) ~= 0
                error('ViterbiDecoder: input length (%d) not multiple of nOut=%d.', ...
                      numel(v), nOut);
            end

            T = numel(v) / nOut;   % number of input-bit steps
            rx = reshape(v, nOut, T).';   % T x nOut

            S  = obj.NumStates;
            NS = obj.NextState;
            OB = obj.OutputBits;

            % Path metrics (large initial values)
            PM = inf(1,S);
            PM(1) = 0;  % start at all-zero state (index 1)

            % Survivor info
            PrevState = zeros(T, S, 'uint16');
            PrevInput = false(T, S);

            for t = 1:T
                newPM       = inf(1,S);
                newPrevSta  = zeros(1,S, 'uint16');
                newPrevInp  = false(1,S);

                rxRow = rx(t,:);  % 1 x nOut

                for sIdx = 1:S
                    pmOld = PM(sIdx);
                    if isinf(pmOld)
                        continue;
                    end

                    % Try input bit 0 and 1
                    for b = 0:1
                        ns = NS(sIdx, b+1);
                        out = squeeze(OB(sIdx, b+1, :)).';  % 1 x nOut

                        % Hamming branch metric
                        bm = sum(out ~= rxRow);
                        metric = pmOld + bm;

                        if metric < newPM(ns)
                            newPM(ns)       = metric;
                            newPrevSta(ns)  = uint16(sIdx);
                            newPrevInp(ns)  = logical(b);
                        end
                    end
                end

                PM             = newPM;
                PrevState(t,:) = newPrevSta;
                PrevInput(t,:) = newPrevInp;
            end

            % Traceback from best final state
            [~, bestState] = min(PM);
            uFull = false(T,1);
            s = uint16(bestState);
            for t = T:-1:1
                uFull(t) = PrevInput(t,s);
                s        = PrevState(t,s);
            end

            % Drop tail bits (K-1)
            infoLen = T - obj.Memory;
            if infoLen <= 0
                error('Sequence too short (%d) for K=%d.', T, obj.K);
            end
            uHat = double(uFull(1:infoLen));
        end
    end

    methods (Static)
        function obj = rateHalf_K3()
            % Convenience constructor: rate-1/2, K=3, G = [111; 101].
            G = [1 1 1;
                 1 0 1];
            obj = fec.ViterbiDecoder(G);
        end
    end
end
