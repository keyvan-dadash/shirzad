clear; clc;

cfg = phyAppConfig();

scrSeedBits = logical([ ...
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ...
]);

fc              = cfg.Link.fcTx;
MasterClockRate = cfg.Link.MasterClockRate;
Interp          = cfg.Link.Interp;
Fs              = MasterClockRate / Interp;

sps   = cfg.Link.Sps;
beta  = cfg.Link.RrcBeta;
span  = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;
txGain_dB       = cfg.Link.TxGain_dB;

msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen;

modulator   = modulators.getMmodulator(cfg.Modulation.Name);
demodulator = demodulators.getDemodulator(cfg.Modulation.Name);

M   = modulator.M;
bps = modulator.BitsPerSymbol;

enc = fec.ConvEncoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

fecEncodeFcn = @(dataBits) enc.encode(logical(dataBits), true);
% fecEncodeFcn = @(dataBits) ...
%     fec.puncture78_k3( enc.encode(logical(dataBits), true) );

% Decode function is not used in TX, but Payload wants a handle.
fecDecodeDummy = @(codedBits) error('Payload.decode is not used in TX.');

pre = protocol.Preamble.fromMSequence( ...
    modulator, ...
    preambleHalfLen, ...
    'Degree', cfg.Frame.MseqDegree, ...
    'Seed',   cfg.Frame.MseqSeed);

% Payload object (computes codedBitsLen, padBitsLen internally)
pay = protocol.Payload(modulator, demodulator, ...
              payloadSyms, msgCapBytes, pilotBitsLen, ...
              fecEncodeFcn, fecDecodeDummy);

% Full PHY frame = [preamble | payload]
fr = protocol.Frame(pre, pay);
preambleLen = fr.NumPreambleSymbols;

hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

databitsLen  = fr.Payload.DataBitsLen;
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2 * L_in, ...
    'TX: codedBitsLen (%d) != 2*(databitsLen+%d)=%d.', ...
    codedBitsLen, Kminus1, 2*L_in);
% assert(codedBitsLen == floor(8/7 * L_in), ...
%     'TX: codedBitsLen (%d) != 2*(databitsLen+%d)=%d.', ...
%     codedBitsLen, Kminus1, 8/7 * L_in);
% assert(mod(L_in, 7) == 0, 'L_in (trellis steps) must be a multiple of 7 for rate 7/8.');

fprintf('TX protocol+FEC:\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  T = L_in        : %d time steps\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

dgramSrc = sources.DatagramSource(msgCapBytes, cfg.Tx.StreamSpecs);

txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',           cfg.SDR.TxIPAddress, ...
  'CenterFrequency',     fc, ...
  'MasterClockRate',     MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',                txGain_dB, ...
  'UseExternalRef',      false, ...
  'TransportDataType', 'int8');

fprintf('\nTX: pre-buffering frames from DatagramSource until EOF...\n');

maxBufferedFrames = 100000;
pilotAmp          = cfg.Frame.PilotAmpOffset;

encodedFrames = cell(maxBufferedFrames, 1);
frameCount    = 0;

while true
    [protoBytes, dInfo] = dgramSrc.readFrame();

    % Treat empty protoBytes as EOF (TODO: stupid logic because it waits
    % for all streams to finish)
    if dInfo.EOFAll
        fprintf('  DatagramSource returned empty protoBytes -> EOF.\n');
        break;
    end

    frameCount = frameCount + 1;
    if frameCount > maxBufferedFrames
        fprintf('  Reached maxBufferedFrames=%d, stopping buffer.\n', maxBufferedFrames);
        frameCount = maxBufferedFrames;
        break;
    end

    protoBytes = scrambler.scrambleBytes(protoBytes, scrSeedBits);

    % Encode one PHY frame
    [frmSyms_raw, ~] = fr.encode(protoBytes);
    frmSyms          = pilotAmp + frmSyms_raw;

    encodedFrames{frameCount} = frmSyms;

    if mod(frameCount, 1000) == 0
        fprintf('  Buffered %d frames so far...\n', frameCount);
    end

    if isstruct(dInfo)
        if (isfield(dInfo, 'Eof') && dInfo.Eof) || ...
           (isfield(dInfo, 'eof') && dInfo.eof)
            fprintf('  DatagramSource dInfo signaled EOF after %d frames.\n', frameCount);
            break;
        end
    end
end

if frameCount == 0
    error('TX: No frames were read from DatagramSource (EOF immediately?).');
end

encodedFrames = encodedFrames(1:frameCount);

NsymFrame = numel(encodedFrames{1});
for k = 2:frameCount
    if numel(encodedFrames{k}) ~= NsymFrame
        error('TX: Not all encoded frames have equal symbol length.');
    end
end

NsampFrame = NsymFrame * sps;

fprintf('TX: buffered %d frames | each frame: %d symbols, %d samples (upsampled)\n', ...
        frameCount, NsymFrame, NsampFrame);
fprintf('TX: entering streaming loop (cycling over buffered frames)...\n\n');

k         = 0;
frameIdx  = 1;
txMon.t0  = tic;
txMon.lastPrintK  = 0;
txMon.printEveryFrames = 1000;

txMon.payloadPerFrameBytes = maxProtoPayload;

disp('TX: streaming buffered frames via USRP. Ctrl+C to stop.');

index = 0;
frameIdx = 1;

while true
    frmSyms = encodedFrames{frameIdx};

    index = index + 1;
    if index > 2
        index = 0;
        frameIdx = frameIdx + 1;
        if frameIdx >= frameCount + 1
            frameIdx = 1;
        end
    end

    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);

    mx = max(abs(txWave));
    if mx > 0
        txWave = txWave ./ mx * 0.8;
    end

    txSink.writeFrame(txWave, struct('FrameIndex', k+1));
    k = k + 1;

    if k - txMon.lastPrintK >= txMon.printEveryFrames
        elapsed    = toc(txMon.t0);
        framesThis = k - txMon.lastPrintK;
        fps        = framesThis / elapsed;

        phyBytesPerFrame = msgCapBytes;  % header + payload
        phyRate_kBps     = (fps * cfg.ethernetPayloadLenght) / 1024;

        appBytesPerFrame = txMon.payloadPerFrameBytes;
        appRate_kBps     = (fps * appBytesPerFrame) / 1024;

        fprintf(['TX: frames/s = %.1f | PHY ~ %.1f kB/s (MsgCap=%dB) ' ...
                 '| APP ~ %.1f kB/s (payload≈%dB)\n'], ...
                fps, phyRate_kBps, phyBytesPerFrame, ...
                appRate_kBps, appBytesPerFrame);

        txMon.t0         = tic;
        txMon.lastPrintK = k;
    end
end
