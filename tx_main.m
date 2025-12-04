clear; clc;

%% ---------- Load shared config ----------
cfg = phyAppConfig();

%% ---------- Link / USRP params ----------
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

%% ---------- Modulator / Demodulator (for layout) ----------
modulator = modulators.getMmodulator(cfg.Modulation.Name);
demodulator = demodulators.getDemodulator(cfg.Modulation.Name);   % not strictly used in TX

M   = modulator.M;
bps = modulator.BitsPerSymbol;

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
  'UseExternalRef',      false);

disp('TX: streaming frames via USRP. Ctrl+C to stop.');

k = 0;
globalSampleIndex = 0; %#ok<NASGU>

% --- simple TX throughput monitor ---
txMon.t0          = tic;
txMon.lastPrintK  = 0;
txMon.payloadPerFrameBytes = 29;   % from RX logs (dg.payloadLen)
txMon.printEveryFrames     = 500;  % how often to print

while true
    %% ---------- Get one datagram from source ----------
    % protoBytes : uint8 column, length = msgCapBytes
    [protoBytes, dInfo] = dgramSrc.readFrame(); %#ok<NASGU>

    %% ---------- Frame encode: bytes -> [preamble | payload] symbols ----------
    [frmSyms_raw, frameInfoTX] = fr.encode(protoBytes); %#ok<NASGU>
    % frmSyms_raw: [preambleLen + payloadSyms x 1] complex

    % Optional DC offset / pilot amplitude
    pilotAmp = cfg.Frame.PilotAmpOffset;
    frmSyms  = pilotAmp + frmSyms_raw;
    % frmSyms  = frmSyms_raw;

    %% ---------- Upsample & RRC ----------
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);

    if max(abs(txWave)) > 0
        txWave = txWave ./ max(abs(txWave)) * 0.8;
    end

    %% ---------- Send to USRP ----------
    txSink.writeFrame(txWave, struct('FrameIndex', k+1));
    k = k + 1;

    if k - txMon.lastPrintK >= txMon.printEveryFrames
        elapsed = toc(txMon.t0);
        framesThis = k - txMon.lastPrintK;

        fps = framesThis / elapsed;

        % Upper bound using MsgCapBytes (header + payload)
        phyBytesPerFrame = cfg.Frame.MsgCapBytes;
        phyRate_kBps = (fps * phyBytesPerFrame) / 1024;

        % Application payload rate assuming ~29B payload per frame
        appBytesPerFrame = txMon.payloadPerFrameBytes;
        appRate_kBps     = (fps * appBytesPerFrame) / 1024;

        fprintf(['TX: frames/s = %.1f | PHY ~ %.1f kB/s (MsgCap=%dB) ' ...
                 '| APP ~ %.1f kB/s (payload≈%dB)\n'], ...
                fps, phyRate_kBps, phyBytesPerFrame, ...
                appRate_kBps, appBytesPerFrame);

        txMon.t0         = tic;
        txMon.lastPrintK = k;
    end


    if mod(k, 500) == 0
        fprintf('TX sent %d frames...\n', k);
    end
end
