classdef ConvEncoder < handle
    % Simple rate-1/2 convolutional encoder.

    properties
        G              % [nOut x K] binary (0/1)
        K              % constraint length
        nOut           % number of output bits per input bit
        Memory         % K-1
        State          % 1 x (K-1) memory bits (double 0/1)
    end

    methods
        function obj = ConvEncoder(G)
            if nargin < 1
                % Default to rate-1/2, K=3
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
            obj.State(:) = 0;
        end

        function v = encode(obj, u, terminate)
            if nargin < 3
                terminate = true;
            end

            u = u(:).';
            u = double(u ~= 0);
            GD = obj.G;
            K  = obj.K;

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
                vRow = mod(reg * GD.', 2);

                % update state: new memory = reg(1:K-1)
                obj.State = reg(1:K-1);

                v(outIndex : outIndex + obj.nOut - 1) = vRow;
                outIndex = outIndex + obj.nOut;
            end

            v = v(:);
        end
    end

    methods (Static)
        function obj = rateHalf_K3()
            G = [1 1 1;
                 1 0 1];
            obj = fec.ConvEncoder(G);
        end
    end
end
