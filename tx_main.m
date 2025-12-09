clear; clc;

%% ---------- Load shared config ----------
cfg = phyAppConfig();

scrSeedBits = logical([ ...
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ...
]);

%% ---------- Link / USRP params ----------
fc              = cfg.Link.fcTx;
MasterClockRate = cfg.Link.MasterClockRate;
Interp          = cfg.Link.Interp;
Fs              = MasterClockRate / Interp; %#ok<NASGU>

sps   = cfg.Link.Sps;
beta  = cfg.Link.RrcBeta;
span  = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;
txGain_dB       = cfg.Link.TxGain_dB;

msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen; %#ok<NASGU>

%% ---------- Modulator / Demodulator (for layout) ----------
modulator   = modulators.getMmodulator(cfg.Modulation.Name);
demodulator = demodulators.getDemodulator(cfg.Modulation.Name);   % not strictly used in TX

M   = modulator.M; %#ok<NASGU>
bps = modulator.BitsPerSymbol; %#ok<NASGU>

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

% FEC encoder: dataBits (0/1) -> coded bits (0/1), terminated
fecEncodeFcn = @(dataBits) enc.encode(logical(dataBits), true);

% Decode function is not used in TX, but Payload wants a handle.
fecDecodeDummy = @(codedBits) error('Payload.decode is not used in TX.');

%% ---------- Payload / Frame structure ----------
% Preamble from m-sequence [a, a]
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

%% ---------- Protocol / FEC layout printout ----------
hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

databitsLen  = fr.Payload.DataBitsLen;   % typically 8 * msgCapBytes
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;

L_in = databitsLen + Kminus1;           % "time steps" into encoder
assert(codedBitsLen == 2 * L_in, ...
    'TX: codedBitsLen (%d) != 2*(databitsLen+%d)=%d.', ...
    codedBitsLen, Kminus1, 2*L_in);

fprintf('TX protocol+FEC:\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  T = L_in        : %d time steps\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

%% ---------- Datagram source (application → datagram bytes) ----------
% cfg.Tx.StreamSpecs: array of structs with StreamId + Reader
dgramSrc = sources.DatagramSource(msgCapBytes, cfg.Tx.StreamSpecs);

%% ---------- RRC filter (streaming) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- USRP sink ----------
txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',           cfg.SDR.TxIPAddress, ...
  'CenterFrequency',     fc, ...
  'MasterClockRate',     MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',                txGain_dB, ...
  'UseExternalRef',      false, ...
  'TransportDataType', 'int8');

%% ---------- Pre-buffer ALL frames from IO (until EOF / stop) ----------
fprintf('\nTX: pre-buffering frames from DatagramSource until EOF...\n');

maxBufferedFrames = 100000;   % safety limit in case Reader loops forever
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
    [frmSyms_raw, ~] = fr.encode(protoBytes);     % [NsymFrame x 1]
    frmSyms          = pilotAmp + frmSyms_raw;    % apply DC/pilot offset

    encodedFrames{frameCount} = frmSyms;

    % if frameCount == 595
    %     fprintf('');
    % end

    if mod(frameCount, 1000) == 0
        fprintf('  Buffered %d frames so far...\n', frameCount);
    end

    % Check EOF flag in dInfo if available
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

% Truncate cell array to actual size
encodedFrames = encodedFrames(1:frameCount);

% Sanity check: all frames same length
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

%% ---------- TX monitoring setup ----------
k         = 0;           % total frames sent
frameIdx  = 1;           % 1..frameCount, cyclic
txMon.t0  = tic;
txMon.lastPrintK  = 0;
txMon.printEveryFrames = 1000;

% If you know the *actual* application payload per datagram, set it here.
% Otherwise you can leave it as an approximate number.
txMon.payloadPerFrameBytes = maxProtoPayload;   % upper bound app payload

%% ---------- Main USRP streaming loop ----------
disp('TX: streaming buffered frames via USRP. Ctrl+C to stop.');

index = 0;
% frameIdx = 596;
frameIdx = 1;
% a = encodedFrames{594};
% b = encodedFrames{595};
% c = encodedFrames{596};
% return;
while true
    %% ---------- Take next pre-encoded frame (cyclic) ----------
    frmSyms = encodedFrames{frameIdx};   % [NsymFrame x 1], complex

    % Advance circular index
    index = index + 1;
    if index > 2
        index = 0;
        frameIdx = frameIdx + 1;
        if frameIdx >= frameCount + 1
            frameIdx = 1;
        end
    end
    % if frameIdx > 600
    %     frameIdx = 590;
    % end

    %% ---------- Upsample & RRC ----------
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);

    % Normalize to avoid clipping
    mx = max(abs(txWave));
    if mx > 0
        txWave = txWave ./ mx * 0.8;
    end

    %% ---------- Send to USRP ----------
    txSink.writeFrame(txWave, struct('FrameIndex', k+1));
    k = k + 1;

    %% ---------- Throughput monitor ----------
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

    % if mod(k, 500) == 0
    %     fprintf('TX sent %d frames (cycling over %d buffered frames)...\n', ...
    %             k, frameCount);
    % end
end
