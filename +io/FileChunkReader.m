classdef FileChunkReader < io.Reader
    % FileChunkReader
    %   Reader that splits a file into small chunks suitable for a
    %   Datagram.Payload and adds a mini header:
    %
    %   Byte 1   : FileId (uint8)
    %   Byte 2–5 : Offset (uint32, big-endian)
    %   Byte 6–9 : TotalSize (uint32, big-endian)
    %   Byte 10  : Flags (uint8), bit0 = LAST_CHUNK
    %   Byte 11– : Data bytes (chunk)
    %
    % Typical usage:
    %   r = io.FileChunkReader('input.bin', 1, 22, true);
    %   [payload, n, eof] = r.read(32);   % payload fits in datagram
    
    properties (Access = private)
        Fid
        FileName
        FileSize  uint32
        Offset    uint32      % next file byte to send
        MaxDataBytes uint32   % desired data bytes per chunk (<= maxPayload-10)
        FileId    uint8
        Loop      logical
    end
    
    properties (Constant)
        HEADER_BYTES = uint32(10);   % mini header inside Datagram.Payload
    end
    
    methods
        function obj = FileChunkReader(fileName, fileId, maxDataBytes, loop)
            % fileName     : path to input file
            % fileId       : uint8 ID for this file
            % maxDataBytes : max number of data bytes per chunk (excl. header)
            % loop         : if true, wrap around and repeat forever
            
            if nargin < 4
                loop = true;
            end
            
            if nargin < 3
                error('FileChunkReader requires (fileName, fileId, maxDataBytes).');
            end
            
            obj.FileName = fileName;
            obj.FileId   = uint8(fileId);
            obj.Loop     = logical(loop);
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
            % [data, count, eof] = read(obj, maxBytes)
            % maxBytes should be >= HEADER_BYTES + 1.
            %
            % data  : uint8 column, miniHeader + chunk bytes
            % count : numel(data)
            % eof   : true if no more data AND Loop = false.
            
            hdrB = double(io.FileChunkReader.HEADER_BYTES);
            
            if maxBytes < hdrB + 1
                error('FileChunkReader:MaxBytesTooSmall', ...
                      'maxBytes=%d < HEADER_BYTES+1=%d', maxBytes, hdrB+1);
            end
            
            % We will not use more than maxData = min(MaxDataBytes, maxBytes-hdrB)
            maxData = min(double(obj.MaxDataBytes), maxBytes - hdrB);
            eof = false;
            
            % Handle EOF / looping
            if obj.Offset >= obj.FileSize
                if obj.Loop
                    % Wrap
                    obj.Offset = uint32(0);
                    fseek(obj.Fid, 0, 'bof');
                else
                    data  = uint8([]);
                    count = 0;
                    eof   = true;
                    return;
                end
            end
            
            % Seek to current offset (in case something got out of sync)
            fseek(obj.Fid, double(obj.Offset), 'bof');
            
            % Compute how many bytes we can read before EOF
            bytesLeft = double(obj.FileSize - obj.Offset);
            nData = min(maxData, bytesLeft);
            
            if nData <= 0
                % We are exactly at EOF
                if obj.Loop
                    obj.Offset = uint32(0);
                    fseek(obj.Fid, 0, 'bof');
                    % Try again recursively
                    [data, count, eof] = obj.read(maxBytes);
                    return;
                else
                    data  = uint8([]);
                    count = 0;
                    eof   = true;
                    return;
                end
            end
            
            % Read the chunk
            buf = fread(obj.Fid, nData, '*uint8');
            if isempty(buf)
                data  = uint8([]);
                count = 0;
                eof   = true;
                return;
            end
            buf   = buf(:);
            nRead = numel(buf);
    
            % Determine LAST_CHUNK flag for this pass
            isLast = ((double(obj.Offset) + nRead) >= double(obj.FileSize));
    
            % Build mini header
            hdrB = double(io.FileChunkReader.HEADER_BYTES);
            hdr  = zeros(hdrB, 1, 'uint8');
    
            % Byte 1: FileId
            hdr(1) = obj.FileId;
    
            % Bytes 2–5: Offset (uint32, big-endian)
            offLE      = typecast(uint32(obj.Offset), 'uint8');   % little-endian
            hdr(2:5)   = offLE(end:-1:1);                         % reverse => big-endian
    
            % fprintf('we are gonna print the offset of %.3f\n', obj.Offset);

            % Bytes 6–9: Total size (uint32, big-endian)
            totLE      = typecast(uint32(obj.FileSize), 'uint8');
            hdr(6:9)   = totLE(end:-1:1);
    
            % Byte 10: Flags
            flags = uint8(0);
            if isLast
                flags = bitor(flags, uint8(1));   % bit0 = LAST_CHUNK
            end
            hdr(10) = flags;
    
            % Optional sanity check: decode our own header back (can remove later)
            offCheckBE = hdr(2:5);
            offCheck   = typecast(uint8(offCheckBE(end:-1:1)), 'uint32');
            % fprintf('TX: Offset field = %u (obj.Offset=%u)\n', offCheck, obj.Offset);
    
            data  = [hdr; buf];
            count = numel(data);
    
            % Update offset and potentially wrap
            obj.Offset = obj.Offset + uint32(nRead);
            if obj.Offset >= obj.FileSize && obj.Loop
                obj.Offset = uint32(0);
                fseek(obj.Fid, 0, 'bof');
            end
        end
    end
end
