clear; clc;

%% ---------- Link / USRP params ----------
fc              = 10e6;
MasterClockRate = 100e6;
Interp          = 128;
Fs              = MasterClockRate / Interp;

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleHalfLen = 128;
payloadSyms     = 512;
txGain_dB       = 0;

%% ---------- Modulator / Demodulator ----------
modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();   % not strictly used in TX, but kept for symmetry

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();
K   = 3;              % constraint length
Kminus1 = K - 1;      % memory

% FEC encoder: dataBits (0/1) -> coded bits (0/1), terminated
fecEncodeFcn = @(dataBits) enc.encode(logical(dataBits), true);

% Decode function is not used in TX, but Payload wants a handle.
fecDecodeDummy = @(codedBits) error('Payload.decode is not used in TX.');

%% ---------- Payload / Frame structure ----------
msgCapBytes   = 40;    % total datagram bytes (header + payload)
pilotBitsLen  = 100;   % must match RX

% Preamble from m-sequence [a, a]
pre = protocol.Preamble.fromMSequence(modQPSK, preambleHalfLen, ...
                             'Degree', 9, 'Seed', 1001);

% Payload object (computes codedBitsLen, padBitsLen internally)
pay = protocol.Payload(modQPSK, demQPSK, ...
              payloadSyms, msgCapBytes, pilotBitsLen, ...
              fecEncodeFcn, fecDecodeDummy);

% Full PHY frame = [preamble | payload]
fr = protocol.Frame(pre, pay);

preambleLen = fr.NumPreambleSymbols;

%% ---------- Protocol / FEC layout printout ----------
hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

databitsLen  = fr.Payload.DataBitsLen;   % 8 * msgCapBytes
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
% Example app: single text stream on StreamId=0
msgReader = io.FixedMessageReader('Hello from TX via USRP!', true);

streamSpecs(1).StreamId = uint8(0);
streamSpecs(1).Reader   = msgReader;

dgramSrc = sources.DatagramSource(msgCapBytes, streamSpecs);

%% ---------- RRC filter (streaming) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- USRP sink ----------
txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',           '192.168.10.5', ...
  'CenterFrequency',     fc, ...
  'MasterClockRate',     MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',                txGain_dB, ...
  'UseExternalRef',      false);

disp('TX: streaming frames via USRP. Ctrl+C to stop.');

k = 0;
globalSampleIndex = 0; %#ok<NASGU>

while true
    %% ---------- Get one datagram from source ----------
    % protoBytes : uint8 column, length = msgCapBytes
    [protoBytes, dInfo] = dgramSrc.readFrame(); %#ok<NASGU>

    %% ---------- Frame encode: bytes -> [preamble | payload] QPSK symbols ----------
    [frmSyms_raw, frameInfoTX] = fr.encode(protoBytes); %#ok<NASGU>
    % frmSyms_raw: [preambleLen + payloadSyms x 1] complex

    % Optional DC offset / pilot amplitude, as in your original script
    pilotAmp = 0.8;
    frmSyms  = pilotAmp + frmSyms_raw;

    %% ---------- Upsample & RRC ----------
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);

    if max(abs(txWave)) > 0
        txWave = txWave ./ max(abs(txWave)) * 0.8;
    end

    %% ---------- Optional CFO injection (disabled) ----------
    % N       = numel(txWave);
    % n       = (0:N-1).' + globalSampleIndex;
    % cfoHz   = 1000;
    % txWave  = txWave .* exp(1j*2*pi*cfoHz*n/Fs);
    % globalSampleIndex = globalSampleIndex + N;

    %% ---------- Send to USRP ----------
    txSink.writeFrame(txWave, struct('FrameIndex', k+1));
    k = k + 1;

    if mod(k,50) == 0
        fprintf('TX sent %d frames...\n', k);
    end
end
