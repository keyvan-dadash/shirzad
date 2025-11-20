classdef ConvEncoder < handle
    % Simple rate-1/2 convolutional encoder (binary, non-recursive).
    %
    % State representation:
    %   Memory bits = [u(k-1), u(k-2), ..., u(k-K+1)]  (1 x (K-1))
    %
    % Generator matrix G is size [nOut x K], with rows = polynomials
    % in binary, left-most column = current input bit.
    %
    % Example for rate 1/2, K=3:
    %   G = [1 1 1;   % g0(D) = 1 + D + D^2
    %        1 0 1];  % g1(D) = 1 + D^2

    properties
        G              % [nOut x K] binary (0/1)
        K              % constraint length
        nOut           % number of output bits per input bit
        Memory         % K-1
        State          % 1 x (K-1) memory bits (double 0/1)
    end

    methods
        function obj = ConvEncoder(G)
            % G: [nOut x K] binary matrix
            if nargin < 1
                % Default to rate-1/2, K=3, G = [111; 101]
                G = [1 1 1;
                     1 0 1];
            end
            obj.G      = double(G ~= 0);
            obj.nOut   = size(obj.G,1);
            obj.K      = size(obj.G,2);
            obj.Memory = obj.K - 1;
            obj.State  = zeros(1, obj.Memory);  % start in all-zero state
        end

        function reset(obj)
            % Reset encoder to all-zero state.
            obj.State(:) = 0;
        end

        function v = encode(obj, u, terminate)
            % Encode a sequence of bits.
            %
            % u         : column or row vector of {0,1}
            % terminate : if true (default), appends K-1 zero bits to
            %             force final state to zero.
            %
            % v : column vector of {0,1}

            if nargin < 3
                terminate = true;
            end

            u = u(:).';               % row
            u = double(u ~= 0);       % ensure 0/1 doubles
            GD = obj.G;               % nOut x K
            K  = obj.K;

            % Optional termination bits to bring state back to zero
            if terminate
                u = [u, zeros(1, obj.Memory)];
            end

            nBits    = numel(u);
            v        = zeros(1, nBits * obj.nOut);
            outIndex = 1;

            for k = 1:nBits
                uk = u(k);
                % reg = [current input, previous memory bits]
                reg = [uk, obj.State];

                % output bits: vRow = reg * G.' (mod 2)
                vRow = mod(reg * GD.', 2);    % 1 x nOut

                % update state: new memory = reg(1:K-1)
                obj.State = reg(1:K-1);

                v(outIndex : outIndex + obj.nOut - 1) = vRow;
                outIndex = outIndex + obj.nOut;
            end

            v = v(:);  % column
        end
    end

    methods (Static)
        function obj = rateHalf_K3()
            % Convenience constructor: rate-1/2, K=3, G = [111; 101].
            G = [1 1 1;
                 1 0 1];
            obj = fec.ConvEncoder(G);
        end
    end
end
