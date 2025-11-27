classdef Payload < handle
    %PAYLOAD Encapsulates FEC + pilot + modulation for the payload part
    %of a frame.
    %
    %   Design:
    %       - You configure it with:
    %           * Modulator / Demodulator
    %           * payloadSyms      (number of *symbols* in payload)
    %           * msgCapBytes      (bytes in the datagram)
    %           * pilotBitsLen     (number of pilot bits at front)
    %           * fecEncFcn        (dataBits -> codedBits)
    %           * fecDecFcn        (codedBits -> decodedBits)
    %
    %       - It then:
    %           * computes infoBitsLen = payloadSyms * bps
    %           * probes fecEncFcn with zeros to learn codedBitsLen
    %           * computes padBitsLen = infoBitsLen - pilotBitsLen
    %                     - codedBitsLen
    %           * generates fixed pilotBits
    %
    %   TX side:
    %       [syms, info] = pay.encode(dataBytes);
    %
    %   RX side:
    %       [dataBytesHat, info] = pay.decode(rxSyms);
    %
    %   Where:
    %       - dataBytes      is uint8 column (<= msgCapBytes).
    %       - syms           is [payloadSyms x 1] complex.
    %       - rxSyms         is same shape after CFO/PLL etc.
    %       - info is a struct with intermediate stuff (optional).

    properties (SetAccess = immutable)
        Modulator               % AbstractModulator
        Demodulator             % AbstractDemodulator

        PayloadSyms
        MsgCapBytes
        PilotBitsLen

        InfoBitsLen
        DataBitsLen
        CodedBitsLen
        PadBitsLen

        PilotBits

        FecEncodeFcn
        FecDecodeFcn
    end

    methods
        function obj = Payload(modulator, demodulator, payloadSyms, ...
                               msgCapBytes, pilotBitsLen, ...
                               fecEncodeFcn, fecDecodeFcn)
            % Payload(mod, demod, payloadSyms, msgCapBytes, pilotBitsLen,
            %         fecEncodeFcn, fecDecodeFcn)

            arguments
                modulator
                demodulator
                payloadSyms   (1,1) double {mustBePositive, mustBeInteger}
                msgCapBytes   (1,1) double {mustBePositive, mustBeInteger}
                pilotBitsLen  (1,1) double {mustBeNonnegative, mustBeInteger}
                fecEncodeFcn  (1,1) function_handle
                fecDecodeFcn  (1,1) function_handle
            end

            obj.Modulator   = modulator;
            obj.Demodulator = demodulator;

            obj.PayloadSyms  = payloadSyms;
            obj.MsgCapBytes  = msgCapBytes;
            obj.PilotBitsLen = pilotBitsLen;

            bps = modulator.BitsPerSymbol;

            obj.InfoBitsLen = payloadSyms * bps;
            obj.DataBitsLen = 8 * msgCapBytes;

            obj.FecEncodeFcn = fecEncodeFcn;
            obj.FecDecodeFcn = fecDecodeFcn;

            % Probe FEC encoder once to figure out codedBitsLen
            testInBits = zeros(obj.DataBitsLen, 1);
            testOut    = fecEncodeFcn(testInBits);
            obj.CodedBitsLen = numel(testOut);

            obj.PadBitsLen = obj.InfoBitsLen - obj.PilotBitsLen - obj.CodedBitsLen;
            if obj.PadBitsLen < 0
                error('Payload:BadLayout', ...
                      ['infoBitsLen=%d, pilotBitsLen=%d, codedBitsLen=%d ' ...
                       '=> padBitsLen=%d < 0. Inconsistent config.'], ...
                      obj.InfoBitsLen, obj.PilotBitsLen, ...
                      obj.CodedBitsLen, obj.PadBitsLen);
            end

            % Fixed pilot bits (can be made configurable)
            rng(1001);
            obj.PilotBits = logical(randi([0 1], obj.PilotBitsLen, 1));
        end

        % ----------- TX: bytes -> frame payload symbols -----------
        function [syms, info] = encode(obj, dataBytes)
            % [syms, info] = encode(obj, dataBytes)
            %
            % dataBytes : uint8 column (<= MsgCapBytes).
            % syms      : [PayloadSyms x 1] complex
            %
            % info      : struct with fields
            %               .dataBytesFull
            %               .dataBits
            %               .codedBits
            %               .infoBits

            if ~isa(dataBytes, 'uint8')
                dataBytes = uint8(dataBytes);
            end
            dataBytes = dataBytes(:);
            nB = numel(dataBytes);
            if nB > obj.MsgCapBytes
                error('Payload:TooManyBytes', ...
                      'Got %d bytes, but MsgCapBytes=%d.', ...
                      nB, obj.MsgCapBytes);
            end

            % Pad bytes to capacity (like fixed-length datagram)
            dataBytesFull = zeros(obj.MsgCapBytes, 1, 'uint8');
            dataBytesFull(1:nB) = dataBytes;

            % Bytes -> bits (MSB first)
            bitsMat  = de2bi(dataBytesFull, 8, 'left-msb');  % [N x 8]
            dataBits = bitsMat.';                            % [8 x N]
            dataBits = dataBits(:);                          % [8N x 1]
            dataBits = double(dataBits ~= 0);                % make sure it's 0/1 double

            if numel(dataBits) ~= obj.DataBitsLen
                error('Payload:InternalBitLenMismatch', ...
                      'Expected DataBitsLen=%d, got %d.', ...
                      obj.DataBitsLen, numel(dataBits));
            end

            % FEC encode
            codedBits = obj.FecEncodeFcn(dataBits(:));
            codedBits = double(codedBits(:) ~= 0);

            if numel(codedBits) ~= obj.CodedBitsLen
                error('Payload:FecLenMismatch', ...
                      'Expected CodedBitsLen=%d, got %d from encoder.', ...
                      obj.CodedBitsLen, numel(codedBits));
            end

            % Build info bits = [pilot | coded | pad]
            infoBits = [obj.PilotBits(:); ...
                        codedBits(:); ...
                        zeros(obj.PadBitsLen,1)];

            if numel(infoBits) ~= obj.InfoBitsLen
                error('Payload:InfoLenMismatch', ...
                      'Expected InfoBitsLen=%d, got %d.', ...
                      obj.InfoBitsLen, numel(infoBits));
            end

            % Modulate to payload symbols
            syms = obj.Modulator.modulate(infoBits);

            if numel(syms) ~= obj.PayloadSyms
                error('Payload:SymbolLenMismatch', ...
                      'Expected PayloadSyms=%d, got %d from modulator.', ...
                      obj.PayloadSyms, numel(syms));
            end

            if nargout > 1
                info = struct();
                info.dataBytesFull = dataBytesFull;
                info.dataBits      = dataBits;
                info.codedBits     = codedBits;
                info.infoBits      = infoBits;
            end
        end

        % ----------- RX: frame payload symbols -> bytes -----------
        function [dataBytesHat, info] = decode(obj, rxSyms)
            % [dataBytesHat, info] = decode(obj, rxSyms)
            %
            % rxSyms       : [PayloadSyms x 1] complex, after CFO/PLL/etc.
            % dataBytesHat : [MsgCapBytes x 1] uint8
            %
            % info fields:
            %   .rxBits
            %   .codedBits
            %   .dataBitsHat
            %   .hardErrorRate (optional, vs re-encoded pilot)

            rxSyms = rxSyms(:);
            if numel(rxSyms) ~= obj.PayloadSyms
                error('Payload:BadRxLen', ...
                      'Expected %d payload symbols, got %d.', ...
                      obj.PayloadSyms, numel(rxSyms));
            end

            % Hard demap
            rxBits = obj.Demodulator.demodulateHard(rxSyms);
            rxBits = double(rxBits(:) ~= 0);

            if numel(rxBits) ~= obj.InfoBitsLen
                error('Payload:RxBitLenMismatch', ...
                      'Expected %d bits from demodulator, got %d.', ...
                      obj.InfoBitsLen, numel(rxBits));
            end

            % Strip pilot & pad
            codedBits = rxBits(obj.PilotBitsLen + 1 : ...
                               obj.PilotBitsLen + obj.CodedBitsLen);

            if numel(codedBits) ~= obj.CodedBitsLen
                error('Payload:SliceError', ...
                      'Slicing coded bits produced %d, expected %d.', ...
                      numel(codedBits), obj.CodedBitsLen);
            end

            % FEC decode
            dataBitsHat = obj.FecDecodeFcn(codedBits(:));
            dataBitsHat = double(dataBitsHat(:) ~= 0);

            if numel(dataBitsHat) < obj.DataBitsLen
                error('Payload:DecLenTooShort', ...
                      'Decoder returned %d bits, need at least %d.', ...
                      numel(dataBitsHat), obj.DataBitsLen);
            end

            dataBitsHat = dataBitsHat(1:obj.DataBitsLen);

            % Bits -> bytes
            bitMat = reshape(dataBitsHat, 8, []).';    % [N x 8]
            dataBytesHat = uint8(bi2de(bitMat, 'left-msb'));

            if nargout > 1
                info = struct();
                info.rxBits      = rxBits;
                info.codedBits   = codedBits;
                info.dataBitsHat = dataBitsHat;

                % Optional: quick pilot check
                recPilot = rxBits(1:obj.PilotBitsLen);
                if ~isempty(recPilot)
                    info.pilotErrorRate = mean(recPilot ~= obj.PilotBits(:));
                else
                    info.pilotErrorRate = NaN;
                end
            end
        end
    end
end
