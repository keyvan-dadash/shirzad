classdef PayloadBitSource < sources.AbstractSource
    %PAYLOADBITSOURCE Source of random payload bits for the TX chain.
    %
    % This implements the "payload" concept from your TX script:
    %   - Each frame: infoBitsLen bits
    %   - Uses a reproducible RNG so TX and RX can share the same seed.
    %
    % Example (from TX):
    %   infoBitsLen = round(payloadSyms * bps * R);
    %   src = sources.PayloadBitSource(infoBitsLen, 'Seed', 1001);
    %   [bits, info] = src.readFrame();
    %
    % You can later replace this with a VideoSource that returns bytes,
    % but the interface (readFrame) stays the same.

    properties (SetAccess = private)
        NumBitsPerFrame    % Number of bits produced per call
        Seed               % RNG seed for reproducibility
        RandStream         % RandStream object (local RNG)
        FrameIndex = 0     % Counts how many frames have been generated
    end

    methods
        function obj = PayloadBitSource(numBitsPerFrame, varargin)
            % PayloadBitSource(numBitsPerFrame, 'Seed', 1001)
            if nargin < 1
                error('PayloadBitSource:NumBitsPerFrame', ...
                      'numBitsPerFrame must be specified.');
            end

            % Parse optional args
            p = inputParser;
            p.addParameter('Seed', 1001, @(x)isnumeric(x)&&isscalar(x));
            p.parse(varargin{:});

            obj@sources.AbstractSource('PayloadBitSource', NaN, numBitsPerFrame);

            obj.NumBitsPerFrame = numBitsPerFrame;
            obj.Seed            = p.Results.Seed;
            obj.RandStream      = RandStream('mt19937ar','Seed',obj.Seed);
        end

        function [bits, info] = readFrame(obj)
            %READFRAME Generate one frame of random bits.
            %
            % bits: column vector (double) in {0,1}
            % info: struct with
            %   - IsValid   : always true
            %   - NumBits   : number of bits in this frame
            %   - FrameIndex: monotonically increasing index

            b = randi(obj.RandStream, [0 1], obj.NumBitsPerFrame, 1);
            obj.FrameIndex = obj.FrameIndex + 1;

            bits = double(b);  % match your original code type
            info = struct( ...
                'IsValid',    true, ...
                'NumBits',    obj.NumBitsPerFrame, ...
                'FrameIndex', obj.FrameIndex);
        end

        function reset(obj)
            %RESET Reset RNG and frame counter.
            obj.RandStream = RandStream('mt19937ar','Seed',obj.Seed);
            obj.FrameIndex = 0;
        end

        function release(obj)
            % Nothing to release for pure-RNG source.
        end
    end
end
