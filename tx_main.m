clear; clc;

%% ---------- Params ----------
fc              = 10e6;
MasterClockRate = 100e6;
Fs              = 1e6;
Interp          = MasterClockRate/Fs;

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleHalfLen = 64;
preambleLen     = 2*preambleHalfLen;

payloadSyms = 512;          % *** unchanged ***
txGain_dB   = 0;

modQPSK = modulators.QpskModulator();

%% ---------- FEC encoder (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();
K   = 3;
Kminus1 = K - 1;

%% ---------- Payload structure: pilot + FEC-coded protocol.Datagram ----------
infoBitsLen  = payloadSyms * bps;   % 512*2 = 1024 bits
pilotBitsLen = 100;                 % must match RX

% Fixed datagram size
msgCapBytes    = 40;                                   % total datagram bytes
hdrBytes       = double(protocol.Datagram.HEADER_BYTES); % 8
maxProtoPayload = msgCapBytes - hdrBytes;              % 32 payload bytes

databitsLen   = 8 * msgCapBytes;   % 40*8 = 320 datagram bits

% TERMINATED code:
%  - Encoder input: databitsLen info bits
%  - Encoder appends K-1 tail zeros internally
%  - Total time-steps T = databitsLen + (K-1)
%  - Coded bits = 2*T
L_in          = databitsLen + Kminus1;   % T = 320 + 2 = 322
codedBitsLen  = 2 * L_in;               % 644 bits

bitsAfterPilot = infoBitsLen - pilotBitsLen;  % 1024 - 100 = 924
padBitsLen    = bitsAfterPilot - codedBitsLen;  % 924 - 644 = 280

if padBitsLen < 0
    error('TX: FEC layout invalid (padBitsLen < 0).');
end

fprintf('TX protocol+FEC:\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  T = L_in        : %d time steps\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

%% ---------- Known pilot bits (must match RX) ----------
rng(1001);
pilotBits = randi([0 1], pilotBitsLen, 1);   % double 0/1

%% ---------- Preamble (known at TX and RX) ----------
mseqGen = training.MSequenceGenerator('Degree', 9);
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];   % [a, a]

%% ---------- RRC filter (STREAMING) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- Reader: source of messages ----------
msgReader = io.FixedMessageReader('Hello from TX via USRP!', true);

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
seqNum = uint16(0);
globalSampleIndex = 0; %#ok<NASGU>

while true
    %% ---------- Get message bytes from Reader ----------
    [msgBytes, n, eof] = msgReader.read(maxProtoPayload); %#ok<NASGU>

    if n == 0
        payload = uint8([]);
    else
        payload = uint8(msgBytes(1:n));
    end

    % Single-datagram message => START + END flags
    flags = bitor(protocol.Datagram.FLAG_START, protocol.Datagram.FLAG_END);

    %% ---------- Build protocol datagram ----------
    dgram = protocol.Datagram(seqNum, flags, payload, uint8(0));
    seqNum = seqNum + uint16(1);

    %% ---------- Encode into fixed-size datagram (27 bytes) ----------
    protoBytes = dgram.toBytes(msgCapBytes);   % always 27 bytes

    %% ---------- bytes -> bits (column) ----------
    dataBitsMatrix = de2bi(protoBytes, 8, 'left-msb').';
    dataBits       = dataBitsMatrix(:);       % 216 bits, double 0/1

    %% ---------- FEC encode (TERMINATED) ----------
    % uBits: ONLY the 216 datagram bits.
    % ConvEncoder.encode(..., true) appends K-1 tail zeros internally.
    uBits = logical(dataBits);             % length = 216

    codedBits = enc.encode(uBits, true);   % length = 2*(216+2) = 436
    codedBits = double(codedBits(:));      % column

    if numel(codedBits) ~= codedBitsLen
        warning('TX: codedBits length %d != expected %d', ...
            numel(codedBits), codedBitsLen);
    end

    %% ---------- Full info bits = [pilotBits; codedBits; padBits] ----------
    padBits = zeros(padBitsLen,1);         % 4 zero bits
    infoBits = [pilotBits; codedBits; padBits];  % 100+436+4 = 540

    %% ---------- map to QPSK symbols ----------
    paySyms = modQPSK.modulate(infoBits);

    %% ---------- full frame = [preamble; payload] ----------
    frmSyms = [preSyms; paySyms];

    %% ---------- upsample & RRC ----------
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);
    txWave = txWave ./ max(abs(txWave)) * 0.8;

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
        fprintf('TX sent %d frames (seq up to %d)...\n', k, uint16(seqNum-1));
    end
end
