classdef PayloadCollectorSink < sinks.AbstractSink
    % PayloadCollectorSink collects payloads

    properties
        StoreHistory   logical = true;
    end

    properties (SetAccess = private)
        Frames         cell   = {};
        FrameLengths   double = [];
        NumFrames      double = 0;
        TotalBits      double = 0;
    end

    methods
        function obj = PayloadCollectorSink()
            obj@sinks.AbstractSink('PayloadCollectorSink', NaN);
        end

        function writeFrame(obj, payloadBits, info)
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

            if isfield(info,'Verbose') && info.Verbose
                fprintf('Stored RX payload frame %d (%d bits)\n', ...
                        obj.NumFrames, L);
            end
        end

        function bits = concatenateAll(obj)
            % Lets concate the payloads
            if ~obj.StoreHistory
                error('PayloadCollectorSink:NoHistory', ...
                      'History is not being stored (StoreHistory=false).');
            end
            bits = vertcat(obj.Frames{:});
        end

        function reset(obj)
            obj.Frames       = {};
            obj.FrameLengths = [];
            obj.NumFrames    = 0;
            obj.TotalBits    = 0;
        end

        function release(obj)
            % No need to release
        end
    end
end
