classdef Frame < handle
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

        function n = get.NumPreambleSymbols(obj)
            n = obj.Preamble.NumSymbols;
        end

        function n = get.NumPayloadSymbols(obj)
            n = obj.Payload.PayloadSyms;
        end

        function n = get.NumFrameSymbols(obj)
            n = obj.NumPreambleSymbols + obj.NumPayloadSymbols;
        end

        function [frameSyms, info] = encode(obj, dataBytes)
            [paySyms, payInfo] = obj.Payload.encode(dataBytes);
            frameSyms = [obj.Preamble.Symbols; paySyms];

            if nargout > 1
                info = struct();
                info.preambleSymbols = obj.Preamble.Symbols;
                info.payload         = payInfo;
            end
        end

        function [codedBits, info] = decodeFromFrame(obj, rxFrameSyms)
            rxFrameSyms = rxFrameSyms(:);
            if numel(rxFrameSyms) ~= obj.NumFrameSymbols
                error('Frame:BadLen', ...
                      'Expected %d frame symbols, got %d.', ...
                      obj.NumFrameSymbols, numel(rxFrameSyms));
            end

            rxPayloadSyms = rxFrameSyms(obj.NumPreambleSymbols+1:end);
            [codedBits, payInfo] = obj.Payload.decode(rxPayloadSyms);

            if nargout > 1
                info = struct();
                info.payload = payInfo;
            end
        end

        function [codedBits, info] = decodeFromPayload(obj, rxPayloadSyms)
            [codedBits, payInfo] = obj.Payload.decode(rxPayloadSyms);

            if nargout > 1
                info = struct();
                info.payload = payInfo;
            end
        end
    end
end
