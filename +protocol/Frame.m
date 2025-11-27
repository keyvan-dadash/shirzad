classdef Frame < handle
    %FRAME Represents one PHY frame: [preamble | payload].
    %
    %   Holds:
    %       - Preamble (Preamble object)
    %       - Payload  (Payload object)
    %
    %   TX:
    %       [frameSyms, info] = fr.encode(dataBytes);
    %
    %   RX (assuming you've already done preamble detection,
    %   CFO correction, and isolated the payload symbols):
    %
    %       [dataBytesHat, info] = fr.decodeFromPayload(rxPayloadSyms);
    %
    %   Or if you already have the full frame symbols with perfect
    %   alignment (for testing):
    %
    %       [dataBytesHat, info] = fr.decodeFromFrame(rxFrameSyms);

    properties (SetAccess = immutable)
        Preamble
        Payload
    end

    properties (Dependent)
        NumPreambleSymbols
        NumPayloadSymbols
        NumFrameSymbols
    end

    methods
        function obj = Frame(preambleObj, payloadObj)
            arguments
                preambleObj
                payloadObj
            end

            obj.Preamble = preambleObj;
            obj.Payload  = payloadObj;
        end

        % ----- Dependent sizes -----
        function n = get.NumPreambleSymbols(obj)
            n = obj.Preamble.NumSymbols;
        end

        function n = get.NumPayloadSymbols(obj)
            n = obj.Payload.PayloadSyms;
        end

        function n = get.NumFrameSymbols(obj)
            n = obj.NumPreambleSymbols + obj.NumPayloadSymbols;
        end

        % ----- TX: bytes -> full frame symbols -----
        function [frameSyms, info] = encode(obj, dataBytes)
            % [frameSyms, info] = encode(obj, dataBytes)
            %
            % dataBytes : uint8 (<= MsgCapBytes)
            % frameSyms : [NumFrameSymbols x 1] complex

            [paySyms, payInfo] = obj.Payload.encode(dataBytes);
            frameSyms = [obj.Preamble.Symbols; paySyms];

            if nargout > 1
                info = struct();
                info.preambleSymbols = obj.Preamble.Symbols;
                info.payload         = payInfo;
            end
        end

        % ----- RX: full frame symbols -> bytes (ideal alignment) -----
        function [dataBytesHat, info] = decodeFromFrame(obj, rxFrameSyms)
            % [dataBytesHat, info] = decodeFromFrame(obj, rxFrameSyms)
            %
            % *Testing* helper: assumes rxFrameSyms is perfectly aligned
            % and already CFO/PLL corrected.

            rxFrameSyms = rxFrameSyms(:);
            if numel(rxFrameSyms) ~= obj.NumFrameSymbols
                error('Frame:BadLen', ...
                      'Expected %d frame symbols, got %d.', ...
                      obj.NumFrameSymbols, numel(rxFrameSyms));
            end

            rxPayloadSyms = rxFrameSyms(obj.NumPreambleSymbols+1:end);
            [dataBytesHat, payInfo] = obj.Payload.decode(rxPayloadSyms);

            if nargout > 1
                info = struct();
                info.payload = payInfo;
            end
        end

        % ----- RX: only payload symbols -> bytes -----
        function [dataBytesHat, info] = decodeFromPayload(obj, rxPayloadSyms)
            % [dataBytesHat, info] = decodeFromPayload(obj, rxPayloadSyms)
            %
            % This is what you’ll use in your *real* RX, since the
            % front-end already did preamble detection and sliced the
            % payload for you.

            [dataBytesHat, payInfo] = obj.Payload.decode(rxPayloadSyms);

            if nargout > 1
                info = struct();
                info.payload = payInfo;
            end
        end
    end
end
