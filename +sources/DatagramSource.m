classdef DatagramSource < sources.AbstractSource
    properties (SetAccess = private)
        MsgCapBytes       double 
        Streams           struct 
        NextStreamIdx     double = 1
        FrameIndex        double = 0
    end

    methods
        function obj = DatagramSource(msgCapBytes, streamSpecs)
            if nargin < 2
                error('DatagramSource:MissingStreams', ...
                    'You must provide msgCapBytes and streamSpecs.');
            end

            hdrBytes = double(protocol.Datagram.HEADER_BYTES);
            if msgCapBytes <= hdrBytes
                error('DatagramSource:msgCapBytes', ...
                    'msgCapBytes=%d must be > HEADER_BYTES=%d.', ...
                    msgCapBytes, hdrBytes);
            end

            if ~isstruct(streamSpecs)
                error('DatagramSource:StreamSpecsType', ...
                    'streamSpecs must be a struct array with fields StreamId, Reader.');
            end

            % Validate / normalize stream specs
            nStreams = numel(streamSpecs);
            streams  = repmat(struct( ...
                'StreamId', uint8(0), ...
                'Reader',  [], ...
                'Done',    false), nStreams, 1);

            for k = 1:nStreams
                if ~isfield(streamSpecs(k),'StreamId') || ~isfield(streamSpecs(k),'Reader')
                    error('DatagramSource:StreamSpecsFields', ...
                        'Each streamSpecs element must have StreamId and Reader.');
                end
                sid = uint8(streamSpecs(k).StreamId);
                rd  = streamSpecs(k).Reader;
                if ~isa(rd, 'io.Reader')
                    % You can relax this to just a handle class with read()
                    error('DatagramSource:ReaderType', ...
                        'streamSpecs(%d).Reader must be an io.Reader.', k);
                end
                streams(k).StreamId = sid;
                streams(k).Reader   = rd;
                streams(k).Done     = false;
            end

            obj@sources.AbstractSource('DatagramSource', NaN, msgCapBytes);

            obj.MsgCapBytes   = double(msgCapBytes);
            obj.Streams       = streams;
            obj.NextStreamIdx = 1;
            obj.FrameIndex    = 0;
        end

        function [bytes, info] = readFrame(obj)
            hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
            maxPayloadBytes = obj.MsgCapBytes - hdrBytes;

            nStreams = numel(obj.Streams);
            if nStreams == 0
                error('DatagramSource:NoStreams', 'No streams registered.');
            end

            % Find a stream that is not Done
            payload = uint8([]);
            sid     = uint8(0);
            eofAll  = false;

            foundStream = false;
            tried = 0;

            while ~foundStream && tried < nStreams
                idx = obj.NextStreamIdx;
                st  = obj.Streams(idx);

                if ~st.Done
                    [data, count, eof] = st.Reader.read(maxPayloadBytes);

                    if count > 0
                        payload = uint8(data(1:count));
                        sid     = st.StreamId;

                        % If reader reports eof, mark Done
                        if eof
                            st.Done = true;
                        end

                        obj.Streams(idx) = st;
                        foundStream      = true;
                    else
                        % No data from this stream; if eof, mark Done
                        if eof
                            st.Done = true;
                            obj.Streams(idx) = st;
                        end
                    end
                end

                obj.NextStreamIdx = idx + 1;
                if obj.NextStreamIdx > nStreams
                    obj.NextStreamIdx = 1;
                end
                tried = tried + 1;
            end

            if ~foundStream
                % No stream had data; we still return an "empty" datagram.
                % All streams are done.
                eofAll = true;
                sid    = obj.Streams(1).StreamId;
                payload= uint8([]);
            end

            % Construct a datagram packet
            d = protocol.Datagram(sid, payload);

            % Encode to fixed-size frame size
            bytes = d.toBytes(obj.MsgCapBytes);

            obj.FrameIndex = obj.FrameIndex + 1;

            info = struct( ...
                'IsValid',    true, ...
                'FrameIndex', obj.FrameIndex, ...
                'StreamId',   sid, ...
                'PayloadLen', numel(payload), ...
                'EOFAll',     eofAll);
        end

        function reset(obj)
            for k = 1:numel(obj.Streams)
                obj.Streams(k).Done   = false;
            end
            obj.NextStreamIdx = 1;
            obj.FrameIndex    = 0;
        end

        function release(obj)
            % Nothing to do
        end
    end
end
