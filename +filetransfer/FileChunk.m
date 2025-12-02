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
            if ~isa(data,'uint8')
                data = uint8(data);
            end
            data = data(:);

            hdrLen = double(filetransfer.FileChunk.HEADER_BYTES);
            hdr    = zeros(hdrLen, 1, 'uint8');

            % Byte 1: FileId
            hdr(1) = uint8(fileId);

            % Bytes 2–5: Offset (uint32, big-endian)
            off = uint32(offset);
            offLE = typecast(off, 'uint8'); % little-endian
            hdr(2:5) = offLE(end:-1:1); % reverse => big-endian

            % Bytes 6–9: TotalSize (uint32, big-endian)
            tot   = uint32(totalSize);
            totLE = typecast(tot, 'uint8'); % little-endian
            hdr(6:9) = totLE(end:-1:1); % reverse => big-endian

            % Byte 10: Flags
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

            hdrB = double(filetransfer.FileChunk.HEADER_BYTES);
            if numel(payload) < hdrB
                error('FileChunk:PayloadTooShort', ...
                      'Payload length %d < header bytes %d.', ...
                      numel(payload), hdrB);
            end

            hdr  = payload(1:hdrB);
            data = payload(hdrB+1:end);

            meta = struct();

            % Byte 1: FileId
            meta.FileId = hdr(1);

            % Bytes 2–5: Offset (uint32, big-endian)
            offBytesBE   = hdr(2:5);
            meta.Offset  = typecast(uint8(offBytesBE(end:-1:1)), 'uint32');

            % Bytes 6–9: TotalSize (uint32, big-endian)
            totBytesBE     = hdr(6:9);
            meta.TotalSize = typecast(uint8(totBytesBE(end:-1:1)), 'uint32');

            % Byte 10: Flags
            flags       = hdr(10);
            meta.IsLast = bitand(flags, filetransfer.FileChunk.FLAG_LAST) ~= 0;
        end
    end
end
