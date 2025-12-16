classdef PayloadCollectorSink < sinks.AbstractSink
    % PayloadCollectorSink

    properties
        % If true, keep raw datagram bytes in Frames{} for debugging
        StoreHistory   logical = true;
    end

    properties (SetAccess = private)
        % Raw datagram history (if StoreHistory)
        Frames         cell   = {};
        FrameLengths   double = [];
        NumFrames      double = 0;
        TotalBytes     double = 0;

        % StreamId -> io.Writer
        % keys are double(StreamId)
        Writers
        % Whether to call writer.close() when FLAG_END is seen
        WritersCloseOnEnd
    end

    methods
        function obj = PayloadCollectorSink()
            % Call AbstractSink ctor: (name, sampleRate)
            obj@sinks.AbstractSink('PayloadCollectorSink', NaN);

            obj.Writers          = containers.Map('KeyType','double', 'ValueType','any');
            obj.WritersCloseOnEnd= containers.Map('KeyType','double', 'ValueType','any');
        end

        function registerWriter(obj, streamId, writer, varargin)
            % registerWriter(obj, streamId, writer, 'CloseOnEnd', true/false)
            %
            % streamId : uint8 or numeric (will be cast to double key)
            % writer   : io.Writer implementation
            p = inputParser;
            addParameter(p, 'CloseOnEnd', false, @(x)islogical(x)&&isscalar(x));
            parse(p, varargin{:});

            sidKey = double(streamId);
            obj.Writers(sidKey)          = writer;
            obj.WritersCloseOnEnd(sidKey)= p.Results.CloseOnEnd;
        end

        function unregisterWriter(obj, streamId)
            sidKey = double(streamId);
            if isKey(obj.Writers, sidKey)
                obj.Writers.remove(sidKey);
            end
            if isKey(obj.WritersCloseOnEnd, sidKey)
                obj.WritersCloseOnEnd.remove(sidKey);
            end
        end

        function writeFrame(obj, dataBytes, info)
            % writeFrame(obj, dataBytes, info)
            %
            % dataBytes : uint8 column vector holding *one datagram*
            % info      : optional struct (e.g. FrameIndex, etc.), unused here
            if nargin < 3
                info = struct();
            end

            if isempty(dataBytes)
                return;
            end

            if ~isa(dataBytes, 'uint8')
                dataBytes = uint8(dataBytes);
            end
            dataBytes = dataBytes(:);

            % Bookkeeping / history
            obj.NumFrames = obj.NumFrames + 1;
            L             = numel(dataBytes);
            obj.TotalBytes = obj.TotalBytes + L;
            obj.FrameLengths(end+1,1) = L;

            if obj.StoreHistory
                obj.Frames{end+1,1} = dataBytes;
            end

            % Parse datagram
            try
                [pkt, ok] = protocol.Datagram.fromBytes(dataBytes);
            catch ME
                warning('PayloadCollectorSink:BadDatagram', ...
                    'Frame %d: failed to parse Datagram: %s', ...
                    obj.NumFrames, ME.message);
                return;
            end

            if ~ok
                warning('PayloadCollectorSink:ChecksumFailed', ...
                    'Frame %d: Datagram checksum FAILED (Seq=%d, StreamId=%d). Dropping payload.', ...
                    obj.NumFrames, pkt.SeqNum, pkt.StreamId);
                % pkt.debugPrint();  % optional
                return;
            end

            sidKey = double(pkt.StreamId);
            if ~isKey(obj.Writers, sidKey)
                % No writer registered for this stream -> silently drop
                % (or log if you like)
                % fprintf('No writer for StreamId=%d, dropping.\n', pkt.StreamId);
                return;
            end

            writer = obj.Writers(sidKey);

            % Only the first PayloadLen bytes are "real" payload
            pay = pkt.Payload(1 : pkt.PayloadLen);

            % Deliver to application
            writer.write(pay);

            % Optional close on FLAG_END
            if isKey(obj.WritersCloseOnEnd, sidKey)
                closeOnEnd = obj.WritersCloseOnEnd(sidKey);
            else
                closeOnEnd = false;
            end

            if closeOnEnd && bitand(pkt.Flags, protocol.Datagram.FLAG_END)
                try
                    writer.close();
                catch ME
                    warning('PayloadCollectorSink:WriterCloseError', ...
                        'Error closing writer for StreamId=%d: %s', ...
                        pkt.StreamId, ME.message);
                end
            end
        end

        function bits = concatenateAll(obj)
            if ~obj.StoreHistory
                error('PayloadCollectorSink:NoHistory', ...
                      'History is not being stored (StoreHistory=false).');
            end
            bits = vertcat(obj.Frames{:});
        end

        function reset(obj)
            obj.Frames         = {};
            obj.FrameLengths   = [];
            obj.NumFrames      = 0;
            obj.TotalBytes     = 0;
            % Writers stay registered; only data history is cleared.
        end

        function release(obj)
            % Close all writers that asked for CloseOnEnd (best-effort).
            keys = obj.Writers.keys;
            for k = 1:numel(keys)
                sidKey = keys{k};
                writer = obj.Writers(sidKey);
                try
                    writer.close();
                catch
                    % ignore
                end
            end
        end
    end
end
