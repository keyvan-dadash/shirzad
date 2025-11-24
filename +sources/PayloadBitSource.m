classdef PayloadBitSource < sources.AbstractSource
    % PayloadBitSource provides a source that streams data, mostly it used
    % for random payload for testing.

    properties (SetAccess = private)
        NumBitsPerFrame
        Seed
        RandStream
        FrameIndex = 0
    end

    methods
        function obj = PayloadBitSource(numBitsPerFrame, varargin)
            if nargin < 1
                error('PayloadBitSource:NumBitsPerFrame', ...
                      'numBitsPerFrame must be specified.');
            end

            p = inputParser;
            p.addParameter('Seed', 1001, @(x)isnumeric(x)&&isscalar(x));
            p.parse(varargin{:});

            obj@sources.AbstractSource('PayloadBitSource', NaN, numBitsPerFrame);

            obj.NumBitsPerFrame = numBitsPerFrame;
            obj.Seed            = p.Results.Seed;
            obj.RandStream      = RandStream('mt19937ar','Seed',obj.Seed);
        end

        function [bits, info] = readFrame(obj)
            b = randi(obj.RandStream, [0 1], obj.NumBitsPerFrame, 1);
            obj.FrameIndex = obj.FrameIndex + 1;

            bits = double(b);
            info = struct( ...
                'IsValid',    true, ...
                'NumBits',    obj.NumBitsPerFrame, ...
                'FrameIndex', obj.FrameIndex);
        end

        function reset(obj)
            obj.RandStream = RandStream('mt19937ar','Seed',obj.Seed);
            obj.FrameIndex = 0;
        end

        function release(obj)
            % Nothing to release
        end
    end
end
