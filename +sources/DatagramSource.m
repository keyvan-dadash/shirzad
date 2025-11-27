classdef DatagramSource < sources.AbstractSource
    % DatagramSource
    %
    % TX-side "application → datagram" source.
    %
    %  - Holds a set of (StreamId, io.Reader) pairs.
    %  - On readFrame(), picks one stream (round-robin), reads up to
    %    maxProtoPayload bytes, wraps them into a protocol.Datagram
    %    and returns its fixed-size byte representation.
    %
    % For now we mark every datagram as FLAG_START|FLAG_END (i.e.,
    % each datagram is a complete application "message"). If you later
    % want multi-datagram segmentation, we can extend this with state.

    properties (SetAccess = private)
        MsgCapBytes       double          % total datagram bytes, inc. header
        Streams           struct          % array of structs with fields:
                                           %   StreamId (uint8)
                                           %   Reader  (io.Reader)
                                           %   SeqNum  (uint16)
                                           %   Done    (logical)
        NextStreamIdx     double = 1
        FrameIndex        double = 0
    end

    methods
        function obj = DatagramSource(msgCapBytes, streamSpecs)
            % streamSpecs: either a single struct or an array of structs with fields:
            %   .StreamId : uint8
            %   .Reader   : io.Reader (implements [data,count,eof]=read(maxBytes))
            %
            % Example:
            %   s(1).StreamId = uint8(0);
            %   s(1).Reader   = io.FixedMessageReader("Hello", true);
            %   src = sources.DatagramSource(40, s);

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
                'SeqNum',  uint16(0), ...
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
                streams(k).SeqNum   = uint16(0);
                streams(k).Done     = false;
            end

            obj@sources.AbstractSource('DatagramSource', NaN, msgCapBytes);

            obj.MsgCapBytes   = double(msgCapBytes);
            obj.Streams       = streams;
            obj.NextStreamIdx = 1;
            obj.FrameIndex    = 0;
        end

        function [bytes, info] = readFrame(obj)
            % [bytes, info] = readFrame(obj)
            %
            % bytes : uint8 column, length = msgCapBytes (Datagram)
            % info  : struct with fields:
            %   - IsValid
            %   - FrameIndex
            %   - StreamId
            %   - SeqNum
            %   - Flags
            %   - EOFAll (true if all streams are done after this frame)

            hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
            maxPayloadBytes = obj.MsgCapBytes - hdrBytes;

            nStreams = numel(obj.Streams);
            if nStreams == 0
                error('DatagramSource:NoStreams', 'No streams registered.');
            end

            % Find a stream that is not Done
            payload = uint8([]);
            sid     = uint8(0);
            seq     = uint16(0);
            flags   = bitor(protocol.Datagram.FLAG_START, protocol.Datagram.FLAG_END);
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
                        seq     = st.SeqNum;

                        % For now, each datagram is a standalone "message":
                        flags = bitor(protocol.Datagram.FLAG_START, ...
                                      protocol.Datagram.FLAG_END);

                        % Advance seq for this stream
                        st.SeqNum = st.SeqNum + uint16(1);

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
                seq    = obj.Streams(1).SeqNum;
                payload= uint8([]);
                flags  = bitor(protocol.Datagram.FLAG_START, ...
                               protocol.Datagram.FLAG_END);
            end

            d = protocol.Datagram(seq, flags, payload, sid);
            bytes = d.toBytes(obj.MsgCapBytes);

            obj.FrameIndex = obj.FrameIndex + 1;

            info = struct( ...
                'IsValid',    true, ...
                'FrameIndex', obj.FrameIndex, ...
                'StreamId',   sid, ...
                'SeqNum',     seq, ...
                'Flags',      flags, ...
                'EOFAll',     eofAll);
        end

        function reset(obj)
            % Reset seq numbers and mark streams as not Done.
            for k = 1:numel(obj.Streams)
                obj.Streams(k).SeqNum = uint16(0);
                obj.Streams(k).Done   = false;
            end
            obj.NextStreamIdx = 1;
            obj.FrameIndex    = 0;
        end

        function release(obj)
            % Nothing special here; if your Readers need closing,
            % you can extend this to call a close() method on them.
        end
    end
end
