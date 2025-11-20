classdef Datagram
    % Simple UDP-like datagram header + payload for our PHY link.
    %
    % Layout (all big-endian):
    %   Byte 1: Version       (uint8)
    %   Byte 2: Flags         (uint8)
    %   Byte 3-4: SeqNum      (uint16)
    %   Byte 5: StreamId      (uint8)
    %   Byte 6: PayloadLen    (uint8)
    %   Byte 7-8: Checksum    (uint16, ones'-complement over header+payload, with these bytes = 0)

    properties (Constant)
        VERSION       = uint8(1);
        HEADER_BYTES  = uint8(8);

        % Flag bits
        FLAG_START    = uint8(1);   % 00000001
        FLAG_END      = uint8(2);   % 00000010
    end

    properties
        Version   (1,1) uint8  = protocol.Datagram.VERSION;
        Flags     (1,1) uint8  = uint8(0);
        SeqNum    (1,1) uint16 = uint16(0);
        StreamId  (1,1) uint8  = uint8(0);
        PayloadLen(1,1) uint8  = uint8(0);
        Payload           uint8 = uint8([]);  % column vector
        Checksum (1,1) uint16 = uint16(0);   % stored checksum
    end

    methods
        function obj = Datagram(seqNum, flags, payload, streamId)
            if nargin < 1, seqNum = uint16(0); end
            if nargin < 2, flags  = uint8(0);  end
            if nargin < 3, payload = uint8([]); end
            if nargin < 4, streamId = uint8(0); end

            obj.SeqNum   = uint16(seqNum);
            obj.Flags    = uint8(flags);
            obj.StreamId = uint8(streamId);
            obj.Payload  = uint8(payload(:));
            obj.PayloadLen = uint8(numel(obj.Payload));
        end

        function debugPrint(obj, maxPreviewBytes)
            % Pretty-print datagram contents for debugging.
            %
            % maxPreviewBytes: how many payload bytes to show in ASCII preview
            % (default 32).
    
            if nargin < 2
                maxPreviewBytes = 32;
            end
    
            fprintf('Datagram -----------------------------\n');
            fprintf('  Version     : %d\n', obj.Version);
    
            % Flags as hex + symbolic names
            fprintf('  Flags       : 0x%02X', obj.Flags);
            flagNames = {};
            if bitand(obj.Flags, protocol.Datagram.FLAG_START)
                flagNames{end+1} = 'START'; %#ok<AGROW>
            end
            if bitand(obj.Flags, protocol.Datagram.FLAG_END)
                flagNames{end+1} = 'END'; %#ok<AGROW>
            end
            if ~isempty(flagNames)
                fprintf(' (%s)', strjoin(flagNames, '|'));
            end
            fprintf('\n');
    
            fprintf('  SeqNum      : %d\n', obj.SeqNum);
            fprintf('  StreamId    : %d\n', obj.StreamId);
            fprintf('  PayloadLen  : %d\n', obj.PayloadLen);
            fprintf('  Checksum    : 0x%04X (%d)\n', obj.Checksum, obj.Checksum);
    
            % Hex dump of payload
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
    
            % ASCII preview
            nPrev = min(payLen, maxPreviewBytes);
            if nPrev > 0
                bytes  = obj.Payload(1:nPrev);
                ascii  = char(bytes);
                % replace non-printables
                mask = (bytes < 32) | (bytes > 126);
                ascii(mask) = '.';
                fprintf('  PayloadASCII: %s\n', ascii);
            else
                fprintf('  PayloadASCII: <empty>\n');
            end
                fprintf('-------------------------------------\n');
            end

        function bytes = toBytes(obj, totalBytes)
            % Encode datagram into uint8 column vector.
            % If totalBytes is given, pad payload with zeros up to that size.
            %
            % totalBytes must be >= HEADER_BYTES.

            hdrLen = double(protocol.Datagram.HEADER_BYTES);

            if nargin < 2
                totalBytes = hdrLen + double(obj.PayloadLen);
            end

            totalBytes = double(totalBytes);
            if totalBytes < hdrLen
                error('totalBytes (%d) < header length (%d).', totalBytes, hdrLen);
            end

            % Effective payload: original + padding
            maxPayloadBytes = totalBytes - hdrLen;
            pay = obj.Payload;
            if numel(pay) > maxPayloadBytes
                error('Payload too large (%d) for totalBytes=%d (max payload=%d).', ...
                    numel(pay), totalBytes, maxPayloadBytes);
            end
            padLen = maxPayloadBytes - numel(pay);
            if padLen > 0
                pay = [pay; zeros(padLen,1,'uint8')];
            end

            % Build header with checksum = 0 for computation
            hdr = zeros(hdrLen,1,'uint8');
            hdr(1) = obj.Version;
            hdr(2) = obj.Flags;
            % SeqNum big-endian
            hdr(3) = uint8(bitshift(obj.SeqNum, -8));
            hdr(4) = uint8(bitand(obj.SeqNum, 255));
            hdr(5) = obj.StreamId;
            hdr(6) = obj.PayloadLen;
            hdr(7) = uint8(0);
            hdr(8) = uint8(0);

            pkt = [hdr; pay];

            % Compute checksum over packet with checksum bytes = 0
            cksum = protocol.Datagram.calcChecksum(pkt);
            obj.Checksum = cksum;

            % Write checksum back into header (big-endian)
            hdr(7) = uint8(bitshift(cksum, -8));
            hdr(8) = uint8(bitand(cksum, 255));

            bytes = [hdr; pay];
        end
    end

    methods (Static)
        function [obj, ok] = fromBytes(bytes)
            % Parse datagram from bytes (uint8 column).
            % Returns obj and ok (true if checksum validated).
    
            if ~isa(bytes, 'uint8')
                bytes = uint8(bytes);
            end
            bytes = bytes(:);
    
            hdrLen = double(protocol.Datagram.HEADER_BYTES);
            if numel(bytes) < hdrLen
                error('Datagram too short: %d bytes (need at least %d).', ...
                    numel(bytes), hdrLen);
            end
    
            hdr = bytes(1:hdrLen);
            pay = bytes(hdrLen+1:end);
    
            version    = hdr(1);
            flags      = hdr(2);
            seqNum     = bitor(uint16(hdr(3)) * 256, uint16(hdr(4)));
            streamId   = hdr(5);
            payloadLen = hdr(6);
            checksumRx = bitor(uint16(hdr(7)) * 256, uint16(hdr(8)));
    
            % Clamp payloadLen if header claims more than we actually have
            if payloadLen > numel(pay)
                payloadLen = uint8(numel(pay));
            end
    
            % ===== Checksum only over: header-with-cksum-zeroed + *actual* payload =====
            hdrZero          = hdr;
            hdrZero(7)       = uint8(0);
            hdrZero(8)       = uint8(0);
            payEff           = pay(1:payloadLen);      % only the first PayloadLen bytes
            bytesForChecksum = [hdrZero; payEff];
    
            checksumCalc = protocol.Datagram.calcChecksum(bytesForChecksum);
            ok = isequal(checksumCalc, checksumRx);
    
            fprintf('got checksum: %d and suppose checksum: %d\n', ...
                checksumRx, checksumCalc);
    
            % Build object (even if ok = false, so we can debug it)
            obj = protocol.Datagram(seqNum, flags, pay(1:payloadLen), streamId);
            obj.Version  = version;
            obj.Checksum = checksumRx;
        end

        function cksum = calcChecksum(bytes)
            % 16-bit ones' complement sum (UDP-style) over bytes.
            %
            % bytes: uint8 column. If odd length, pad one zero byte.

            if ~isa(bytes,'uint8')
                bytes = uint8(bytes);
            end
            bytes = bytes(:);

            if mod(numel(bytes),2) == 1
                bytes(end+1,1) = uint8(0);
            end

            % Interpret as uint16 words, big-endian
            hi = uint16(bytes(1:2:end));
            lo = uint16(bytes(2:2:end));
            words = bitor(bitshift(hi,8), lo);  % hi<<8 | lo

            sum32 = uint32(0);
            for k = 1:numel(words)
                sum32 = sum32 + uint32(words(k));
                % wrap-around carry
                if sum32 > 65535
                    sum32 = bitand(sum32, uint32(65535)) + 1;
                end
            end

            cksum = bitcmp(uint16(sum32));   % ones' complement
        end
    end
end
