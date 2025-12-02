classdef Payload < handle
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
        FecDecodeFcn           % kept for compatibility
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

            obj.InfoBitsLen = payloadSyms * bps; % hard limit on the phy
            obj.DataBitsLen = 8 * msgCapBytes;

            obj.FecEncodeFcn = fecEncodeFcn;
            obj.FecDecodeFcn = fecDecodeFcn;  % not used in front-end decode (we use in background c++ code)

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

            % Fixed pilot bits
            rng(1001);
            obj.PilotBits = logical(randi([0 1], obj.PilotBitsLen, 1));
        end

        function [syms, info] = encode(obj, dataBytes)
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

            % Pad bytes to capacity (fixed-length datagram)
            dataBytesFull = zeros(obj.MsgCapBytes, 1, 'uint8');
            dataBytesFull(1:nB) = dataBytes;

            % Bytes -> bits (MSB first)
            bitsMat  = de2bi(dataBytesFull, 8, 'left-msb');
            dataBits = bitsMat.';
            dataBits = dataBits(:);
            dataBits = double(dataBits ~= 0);

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

        function [codedBits, info] = decode(obj, rxSyms)
            rxSyms = rxSyms(:);
            if numel(rxSyms) ~= obj.PayloadSyms
                error('Payload:BadRxLen', ...
                      'Expected %d payload symbols, got %d.', ...
                      obj.PayloadSyms, numel(rxSyms));
            end

            % Hard demap (TODO: should we do llr?)
            rxBits = obj.Demodulator.demodulateHard(rxSyms);
            rxBits = double(rxBits(:) ~= 0);

            if numel(rxBits) ~= obj.InfoBitsLen
                error('Payload:RxBitLenMismatch', ...
                      'Expected %d bits from demodulator, got %d.', ...
                      obj.InfoBitsLen, numel(rxBits));
            end

            % Strip pilot & pad: keep only coded bits
            codedBits = rxBits(obj.PilotBitsLen + 1 : ...
                               obj.PilotBitsLen + obj.CodedBitsLen);

            if numel(codedBits) ~= obj.CodedBitsLen
                error('Payload:SliceError', ...
                      'Slicing coded bits produced %d, expected %d.', ...
                      numel(codedBits), obj.CodedBitsLen);
            end

            if nargout > 1
                info = struct();
                info.rxBits         = rxBits;
                info.codedBits      = codedBits;
            end
        end
    end
end
