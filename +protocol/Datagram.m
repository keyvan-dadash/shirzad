classdef Datagram
    % Datagram protocol for sending data in organized way from tx to rx

    properties (Constant)
        HEADER_BYTES = uint8(5);   % 1 byte (StreamId) + 2 bytes (Len) + 2 bytes (Checksum)
    end

    properties
        StreamId   (1,1) uint8   = uint8(0);
        PayloadLen (1,1) uint16  = uint16(0);
        Payload           uint8  = uint8([]);   % column vector
        Checksum  (1,1) uint16  = uint16(0);   % stored checksum
    end

    methods
        function obj = Datagram(streamId, payload)
            if nargin < 1, streamId = uint8(0); end
            if nargin < 2, payload  = uint8([]); end

            obj.StreamId = uint8(streamId);
            payload      = uint8(payload(:));

            if numel(payload) > double(intmax('uint16'))
                error('Datagram:PayloadTooLarge', ...
                      'Payload length %d exceeds uint16 limit.', numel(payload));
            end

            obj.Payload    = payload;
            obj.PayloadLen = uint16(numel(payload));
        end

        function debugPrint(obj, maxPreviewBytes)
            if nargin < 2
                maxPreviewBytes = 32;
            end

            fprintf('Datagram -----------------------------\n');
            fprintf('  StreamId    : %d\n', obj.StreamId);
            fprintf('  PayloadLen  : %d\n', obj.PayloadLen);
            fprintf('  Checksum    : 0x%04X (%d)\n', obj.Checksum, obj.Checksum);

            payLen = double(min(obj.PayloadLen, numel(obj.Payload)));
            fprintf('  PayloadHex  :');
            for i = 1:payLen
                fprintf(' %02X', obj.Payload(i));
                if mod(i,16) == 0 && i < payLen
                    fprintf('\n               ');
                end
            end
            if payLen == 0
                fprintf(' <empty>');
            end
            fprintf('\n');

            nPrev = min(payLen, maxPreviewBytes);
            if nPrev > 0
                bytes = obj.Payload(1:nPrev);
                ascii = char(bytes);
                mask  = (bytes < 32) | (bytes > 126);
                ascii(mask) = '.';
                fprintf('  PayloadASCII: %s\n', ascii);
            else
                fprintf('  PayloadASCII: <empty>\n');
            end
            fprintf('-------------------------------------\n');
        end

        function bytes = toBytes(obj, totalBytes)
            hdrLen = double(protocol.Datagram.HEADER_BYTES);

            if nargin < 2
                totalBytes = hdrLen + double(obj.PayloadLen);
            end

            totalBytes = double(totalBytes);
            if totalBytes < hdrLen
                error('Datagram:TotalBytesTooSmall', ...
                      'totalBytes (%d) < header length (%d).', totalBytes, hdrLen);
            end

            maxPayloadBytes = totalBytes - hdrLen;
            pay = obj.Payload;

            if numel(pay) > maxPayloadBytes
                error('Datagram:PayloadTooLarge', ...
                      'Payload too large (%d) for totalBytes=%d (max payload=%d).', ...
                      numel(pay), totalBytes, maxPayloadBytes);
            end

            padLen = maxPayloadBytes - numel(pay);
            if padLen > 0
                pay = [pay; zeros(padLen,1,'uint8')];
            end

            % Header with checksum bytes = 0 for checksum calculation
            hdr = zeros(hdrLen,1,'uint8');

            % Byte 1: StreamId
            hdr(1) = obj.StreamId;

            % Bytes 2–3: PayloadLen (uint16, big-endian)
            len = obj.PayloadLen;
            hdr(2) = uint8(bitshift(len, -8));
            hdr(3) = uint8(bitand(len, 255));

            % Bytes 4–5: Checksum placeholder = 0
            hdr(4) = uint8(0);
            hdr(5) = uint8(0);

            pkt = [hdr; pay];

            % Compute checksum over header-with-zero-checksum + payload
            cksum = protocol.Datagram.calcChecksum(pkt);
            obj.Checksum = cksum;

            % Write checksum back into header (big-endian)
            hdr(4) = uint8(bitshift(cksum, -8));
            hdr(5) = uint8(bitand(cksum, 255));

            bytes = [hdr; pay];
        end
    end

    methods (Static)
        function [obj, ok] = fromBytes(bytes)
            if ~isa(bytes, 'uint8')
                bytes = uint8(bytes);
            end
            bytes = bytes(:);

            hdrLen = double(protocol.Datagram.HEADER_BYTES);
            if numel(bytes) < hdrLen
                error('Datagram:TooShort', ...
                      'Datagram too short: %d bytes (need at least %d).', ...
                      numel(bytes), hdrLen);
            end

            hdr = bytes(1:hdrLen);
            pay = bytes(hdrLen+1:end);

            streamId   = hdr(1);
            payloadLen = bitor(uint16(hdr(2)) * 256, uint16(hdr(3)));
            checksumRx = bitor(uint16(hdr(4)) * 256, uint16(hdr(5)));

            if payloadLen > numel(pay)
                payloadLen = uint16(numel(pay));
            end

            hdrZero        = hdr;
            hdrZero(4)     = uint8(0);
            hdrZero(5)     = uint8(0);
            payEff         = pay(1:double(payloadLen));
            bytesForCksum  = [hdrZero; payEff];

            checksumCalc = protocol.Datagram.calcChecksum(bytesForCksum);
            ok           = isequal(checksumCalc, checksumRx);

            obj            = protocol.Datagram(streamId, payEff);
            obj.Checksum   = checksumRx;
        end

        function cksum = calcChecksum(bytes)
            if ~isa(bytes,'uint8')
                bytes = uint8(bytes);
            end
            bytes = bytes(:);

            if mod(numel(bytes),2) == 1
                bytes(end+1,1) = uint8(0);
            end

            hi = uint16(bytes(1:2:end));
            lo = uint16(bytes(2:2:end));
            words = bitor(bitshift(hi,8), lo);

            sum32 = uint32(0);
            for k = 1:numel(words)
                sum32 = sum32 + uint32(words(k));
                if sum32 > 65535
                    sum32 = bitand(sum32, uint32(65535)) + 1;
                end
            end

            cksum = bitcmp(uint16(sum32));
        end
    end
end
