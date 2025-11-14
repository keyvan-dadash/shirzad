classdef PayloadCollectorSink < sinks.AbstractSink
    %PAYLOADCOLLECTORSINK RX sink that stores decoded payload bits.
    %
    % This implements the idea "sink = payload" on the RX side.
    % It doesn't do any DSP; it just receives payload frames and keeps
    % them in memory (or optionally discards them and only counts).
    %
    % Usage in RX:
    %   paySink = sinks.PayloadCollectorSink();
    %   paySink.writeFrame(decBits, struct('FrameIndex', frames));
    %
    %   % Later:
    %   allBits = paySink.concatenateAll();

    properties
        StoreHistory   logical = true;   % If false, we only count frames
    end

    properties (SetAccess = private)
        Frames         cell   = {};      % Cell array of payload vectors
        FrameLengths   double = [];      % Length of each frame
        NumFrames      double = 0;       % Number of frames written
        TotalBits      double = 0;       % Total number of bits
    end

    methods
        function obj = PayloadCollectorSink()
            obj@sinks.AbstractSink('PayloadCollectorSink', NaN);
        end

        function writeFrame(obj, payloadBits, info)
            %WRITEFRAME Store one frame of payload bits.
            %
            % payloadBits: column vector in {0,1} (double/logical)
            % info       : optional struct (e.g., FrameIndex)

            if nargin < 3
                info = struct();
            end

            if isempty(payloadBits)
                return;
            end

            obj.NumFrames = obj.NumFrames + 1;
            L = numel(payloadBits);
            obj.TotalBits = obj.TotalBits + L;
            obj.FrameLengths(end+1,1) = L;

            if obj.StoreHistory
                obj.Frames{end+1,1} = payloadBits(:);
            end

            % Optional debug print
            if isfield(info,'Verbose') && info.Verbose
                fprintf('Stored RX payload frame %d (%d bits)\n', ...
                        obj.NumFrames, L);
            end
        end

        function bits = concatenateAll(obj)
            %CONCATENATEALL Concatenate all stored frames into one vector.
            if ~obj.StoreHistory
                error('PayloadCollectorSink:NoHistory', ...
                      'History is not being stored (StoreHistory=false).');
            end
            bits = vertcat(obj.Frames{:});
        end

        function reset(obj)
            %RESET Clear history and counters.
            obj.Frames       = {};
            obj.FrameLengths = [];
            obj.NumFrames    = 0;
            obj.TotalBits    = 0;
        end

        function release(obj)
            % Nothing external to release.
        end
    end
end
