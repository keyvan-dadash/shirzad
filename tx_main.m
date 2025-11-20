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

payloadSyms = 270;
txGain_dB   = 0;

modQPSK = modulators.QpskModulator();

%% ---------- Payload structure: pilot + protocol.Datagram ----------
infoBitsLen  = payloadSyms * bps;   % 270*2 = 540 bits
pilotBitsLen = 100;                 % must match RX

msgCapBits   = infoBitsLen - pilotBitsLen;   % bits for protocol datagram
assert(mod(msgCapBits,8)==0, 'msgCapBits must be a multiple of 8.');
msgCapBytes  = msgCapBits/8;                 % should be 55 with these params

% Datagram header is 8 bytes -> protocol payload = 55 - 8 = 47 bytes
hdrBytes        = double(protocol.Datagram.HEADER_BYTES);  % 8
maxProtoPayload = msgCapBytes - hdrBytes;                  % 47 bytes
maxMsgBytes     = maxProtoPayload;                         % what Reader sees

fprintf('TX protocol: %d header bytes, %d payload bytes, total %d bytes\n', ...
    hdrBytes, maxProtoPayload, msgCapBytes);

%% ---------- Known pilot bits (must match RX) ----------
rng(1001);
pilotBits = randi([0 1], pilotBitsLen, 1);

%% ---------- Preamble (known at TX and RX) ----------
mseqGen = training.MSequenceGenerator('Degree', 9);
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];   % [a, a]

%% ---------- RRC filter (STREAMING) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- Reader: source of messages ----------
% This can later be swapped with a FileReader, etc.
msgReader = io.FixedMessageReader('Hello from TX via USRP!', true);

%% ---------- USRP sink ----------
txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',         '192.168.10.5', ...
  'CenterFrequency',   fc, ...
  'MasterClockRate',   MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',              txGain_dB, ...
  'UseExternalRef',    false);

disp('TX: streaming frames via USRP. Ctrl+C to stop.');

k = 0;
seqNum = uint16(0);        % protocol sequence number
globalSampleIndex = 0; %#ok<NASGU>

while true
    %% ---------- Get message bytes from Reader ----------
    [msgBytes, n, eof] = msgReader.read(maxMsgBytes); %#ok<NASGU>

    if n == 0
        % No data (EOF or empty) -> send empty datagram
        payload = uint8([]);
    else
        payload = uint8(msgBytes(1:n));
    end

    % Single-datagram message => START + END flags
    flags = bitor(protocol.Datagram.FLAG_START, protocol.Datagram.FLAG_END);

    %% ---------- Build protocol datagram ----------
    dgram = protocol.Datagram(seqNum, flags, payload, uint8(0));
    % increment seqNum as uint16 (wraps naturally at 65535 -> 0)
    seqNum = seqNum + uint16(1);

    %% ---------- Encode into fixed-size protocol payload (55 bytes) ----------
    protoBytes = dgram.toBytes(msgCapBytes);   % always 55 bytes

    % Sanity (optional)
    % if numel(protoBytes) ~= msgCapBytes
    %     error('protoBytes length %d != %d', numel(protoBytes), msgCapBytes);
    % end

    %% ---------- bytes -> bits (column) ----------
    dataBitsMatrix = de2bi(protoBytes, 8, 'left-msb').';
    dataBits       = dataBitsMatrix(:);   % 55 * 8 = 440 bits

    %% ---------- full info bits = [pilotBits; protocol bytes] ----------
    infoBits = [pilotBits; dataBits];     % 100 + 440 = 540 bits

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
