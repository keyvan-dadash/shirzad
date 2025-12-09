classdef FileChunkReaderSegment < io.Reader
    % FileChunkReaderSegment
    %   Reads only a [startOffset,endOffset) byte range from a file,
    %   and encodes it as a sequence of FileChunks with offsets
    %   relative to the start of the segment (0 .. segmentLen-1).

    properties (Access = private)
        Fid
        FileName
        FileSize       uint32
        SegmentStart   uint32   % global start in file
        SegmentEnd     uint32   % global end in file (exclusive)
        SegmentSize    uint32   % SegmentEnd - SegmentStart
        Offset         uint32   % next offset within segment (0..SegmentSize)
        MaxDataBytes   uint32   % data bytes per chunk
        FileId         uint8
        Loop           logical
    end

    properties (Constant)
        HEADER_BYTES = uint32(10);  % must match FileChunk.HEADER_BYTES
    end

    methods
        function obj = FileChunkReaderSegment(fileName, fileId, maxDataBytes, ...
                                              segmentStart, segmentEnd, loop)
            if nargin < 6
                loop = false;
            end

            obj.FileName    = fileName;
            obj.FileId      = uint8(fileId);
            obj.Loop        = logical(loop);
            obj.MaxDataBytes = uint32(maxDataBytes);

            [fid, msg] = fopen(fileName, 'r');
            if fid < 0
                error('FileChunkReaderSegment:CannotOpen', ...
                      'Failed to open "%s": %s', fileName, msg);
            end
            obj.Fid = fid;

            fseek(fid, 0, 'eof');
            fsz = ftell(fid);
            if fsz < 0
                error('FileChunkReaderSegment:FtellFailed', ...
                      'Could not determine file size for "%s".', fileName);
            end
            obj.FileSize = uint32(fsz);

            % clamp / validate segment range
            if segmentStart < 0 || segmentStart >= double(obj.FileSize)
                error('FileChunkReaderSegment:BadSegmentStart', ...
                      'segmentStart (%g) out of range [0,%u).', ...
                      segmentStart, obj.FileSize);
            end
            if segmentEnd <= segmentStart
                error('FileChunkReaderSegment:BadSegmentEnd', ...
                      'segmentEnd (%g) must be > segmentStart (%g).', ...
                      segmentEnd, segmentStart);
            end

            segStart = uint32(segmentStart);
            segEnd   = uint32(min(segmentEnd, double(obj.FileSize)));

            obj.SegmentStart = segStart;
            obj.SegmentEnd   = segEnd;
            obj.SegmentSize  = segEnd - segStart;

            obj.Offset = uint32(0);
        end

        function delete(obj)
            if ~isempty(obj.Fid) && obj.Fid > 0
                fclose(obj.Fid);
                obj.Fid = [];
            end
        end

        function [data, count, eof] = read(obj, maxBytes)
            hdrB = double(io.FileChunkReaderSegment.HEADER_BYTES);

            if maxBytes < hdrB + 1
                error('FileChunkReaderSegment:MaxBytesTooSmall', ...
                      'maxBytes=%d < HEADER_BYTES+1=%d', maxBytes, hdrB+1);
            end

            maxData = min(double(obj.MaxDataBytes), maxBytes - hdrB);
            eof = false;

            % End of segment?
            if obj.Offset >= obj.SegmentSize
                if obj.Loop
                    obj.Offset = uint32(0);
                else
                    data  = uint8([]);
                    count = 0;
                    eof   = true;
                    return;
                end
            end

            % Global file position = SegmentStart + Offset
            globalPos = double(obj.SegmentStart + obj.Offset);
            fseek(obj.Fid, globalPos, 'bof');

            bytesLeft = double(obj.SegmentSize - obj.Offset);
            nData     = min(maxData, bytesLeft);

            if nData <= 0
                if obj.Loop
                    obj.Offset = uint32(0);
                    [data, count, eof] = obj.read(maxBytes);
                    return;
                else
                    data  = uint8([]);
                    count = 0;
                    eof   = true;
                    return;
                end
            end

            buf = fread(obj.Fid, nData, '*uint8');
            if isempty(buf)
                data  = uint8([]);
                count = 0;
                eof   = true;
                return;
            end
            buf   = buf(:);
            nRead = numel(buf);

            % offset and totalSize are relative to this segment
            segOffset    = obj.Offset;
            segTotalSize = obj.SegmentSize;
            isLast       = ((double(obj.Offset) + nRead) >= double(obj.SegmentSize));

            data = filetransfer.FileChunk.encode( ...
                        obj.FileId, segOffset, segTotalSize, isLast, buf);
            count = numel(data);

            obj.Offset = obj.Offset + uint32(nRead);

            if obj.Offset >= obj.SegmentSize && obj.Loop
                obj.Offset = uint32(0);
            end
        end
    end
end
