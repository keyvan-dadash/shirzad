classdef FileChunk
    % FileChunk
    %   Helper for encoding/decoding the file-chunk header:
    %
    %   Byte 1   : FileId (uint8)
    %   Byte 2–5 : Offset (uint32, big-endian)
    %   Byte 6–9 : TotalSize (uint32, big-endian)
    %   Byte 10  : Flags (uint8), bit0 = LAST_CHUNK (EoF)
    %   Byte 11– : Data bytes

    properties (Constant)
        HEADER_BYTES = uint32(10);
        FLAG_LAST    = uint8(1);
    end

    methods (Static)
        function payload = encode(fileId, offset, totalSize, isLast, data)
            % encode -> [header(10 bytes); data]
            if ~isa(data,'uint8')
                data = uint8(data);
            end
            data = data(:);

            hdr = zeros(double(filetransfer.FileChunk.HEADER_BYTES), 1, 'uint8');
            hdr(1) = uint8(fileId);

            off = uint32(offset);
            hdr(2) = uint8( bitand(bitshift(off, -24), 255) );
            hdr(3) = uint8( bitand(bitshift(off, -16), 255) );
            hdr(4) = uint8( bitand(bitshift(off, -8), 255) );
            hdr(5) = uint8(bitand(off, 255));

            tot = uint32(totalSize);
            hdr(6) = uint8( bitand(bitshift(tot, -24), 255) );
            hdr(7) = uint8( bitand(bitshift(tot, -16), 255) );
            hdr(8) = uint8( bitand(bitshift(tot, -8), 255) );
            hdr(9) = uint8(bitand(tot, 255));

            flags = uint8(0);
            if isLast
                flags = bitor(flags, filetransfer.FileChunk.FLAG_LAST);
            end
            hdr(10) = flags;

            payload = [hdr; data];
        end

        function [meta, data] = decode(payload)
            if ~isa(payload, 'uint8')
                payload = uint8(payload);
            end
            payload = payload(:);
        
            hdrB = double(io.FileChunkReader.HEADER_BYTES);   % 10
            if numel(payload) < hdrB
                error('FileChunk:PayloadTooShort', ...
                      'Payload length %d < header bytes %d.', ...
                      numel(payload), hdrB);
            end
        
            hdr  = payload(1:hdrB);
            data = payload(hdrB+1:end);
        
            % Meta Data
            meta = struct();
        
            % Byte 1: FileId
            meta.FileId = hdr(1);
        
            % Bytes 2–5: Offset (uint32, big-endian)
            offBytesBE   = hdr(2:5);
            meta.Offset  = typecast(uint8(offBytesBE(end:-1:1)), 'uint32');
        
            % Bytes 6–9: TotalSize (uint32, big-endian)
            totBytesBE   = hdr(6:9);
            meta.TotalSize = typecast(uint8(totBytesBE(end:-1:1)), 'uint32');
        
            % Byte 10: Flags
            flags        = hdr(10);
            meta.IsLast  = bitand(flags, uint8(1)) ~= 0;

            data = payload(11:end);
        end
    end
end
