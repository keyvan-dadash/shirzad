classdef FileChunkReader < io.Reader
    % FileChunkReader
    %   Reader that splits a file into small chunks suitable for a
    %   Datagram.Payload and adds a mini header by using
    %   filetransfer.FileChunk

    properties (Access = private)
        Fid
        FileName
        FileSize    uint32
        Offset      uint32      % next file byte to send
        MaxDataBytes uint32     % desired data bytes per chunk (<= maxPayload-10) (header size is 10)
        FileId      uint8
        Loop        logical
    end

    properties (Constant)
        HEADER_BYTES = uint32(10);   % mini header bytes. Should be matched FileChunk.HEADER_BYTES
    end

    methods
        function obj = FileChunkReader(fileName, fileId, maxDataBytes, loop)
            if nargin < 4
                loop = true;
            end

            if nargin < 3
                error('FileChunkReader requires (fileName, fileId, maxDataBytes).');
            end

            obj.FileName    = fileName;
            obj.FileId      = uint8(fileId);
            obj.Loop        = logical(loop);
            obj.MaxDataBytes = uint32(maxDataBytes);

            [fid, msg] = fopen(fileName, 'r');
            if fid < 0
                error('FileChunkReader:CannotOpen', ...
                      'Failed to open "%s": %s', fileName, msg);
            end
            obj.Fid = fid;

            fseek(fid, 0, 'eof');
            fsz = ftell(fid);
            if fsz < 0
                error('FileChunkReader:FtellFailed', ...
                      'Could not determine file size for "%s".', fileName);
            end
            obj.FileSize = uint32(fsz);
            fseek(fid, 0, 'bof');

            obj.Offset = uint32(0);
        end

        function delete(obj)
            if ~isempty(obj.Fid) && obj.Fid > 0
                fclose(obj.Fid);
                obj.Fid = [];
            end
        end

        function [data, count, eof] = read(obj, maxBytes)
            hdrB = double(io.FileChunkReader.HEADER_BYTES);

            if maxBytes < hdrB + 1
                error('FileChunkReader:MaxBytesTooSmall', ...
                      'maxBytes=%d < HEADER_BYTES+1=%d', maxBytes, hdrB+1);
            end

            maxData = min(double(obj.MaxDataBytes), maxBytes - hdrB);
            eof = false;

            if obj.Offset >= obj.FileSize
                if obj.Loop
                    % lets re-read the file
                    obj.Offset = uint32(0);
                    fseek(obj.Fid, 0, 'bof');
                else
                    data  = uint8([]);
                    count = 0;
                    eof   = true;
                    return;
                end
            end

            fseek(obj.Fid, double(obj.Offset), 'bof');

            bytesLeft = double(obj.FileSize - obj.Offset);
            nData     = min(maxData, bytesLeft);

            if nData <= 0
                if obj.Loop
                    % lets loop and read
                    obj.Offset = uint32(0);
                    fseek(obj.Fid, 0, 'bof');
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

            isLast = ((double(obj.Offset) + nRead) >= double(obj.FileSize));

            % Encode the chunk with FileChunk
            data  = filetransfer.FileChunk.encode( ...
                        obj.FileId, obj.Offset, obj.FileSize, isLast, buf);
            count = numel(data);

            obj.Offset = obj.Offset + uint32(nRead);
            if obj.Offset >= obj.FileSize && obj.Loop
                obj.Offset = uint32(0);
                fseek(obj.Fid, 0, 'bof');
            end
        end
    end
end
