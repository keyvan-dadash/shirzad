clear; clc;

%% ---------- Params ----------
fc              = 10e6;
MasterClockRate = 100e6;
Interp          = 64;
Fs              = MasterClockRate/Interp;

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleHalfLen = 128;
preambleLen     = 2*preambleHalfLen;

% Bigger payload: 2048 QPSK symbols
payloadSyms = 2048;
txGain_dB   = 0;

modQPSK = modulators.QpskModulator();

%% ---------- FEC encoder (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();
K   = 3;
Kminus1 = K - 1;

% Info bits carried by QPSK payload symbols
infoBitsLen  = payloadSyms * bps;   % 2048*2 = 4096 bits
pilotBitsLen = 100;                 % must match RX

% ---------- Datagram size (bigger now) ----------
% Total datagram bytes (header + payload)
msgCapBytes    = 248;                                   % e.g. 248 bytes
hdrBytes       = double(protocol.Datagram.HEADER_BYTES); % 8
maxProtoPayload = msgCapBytes - hdrBytes;               % 248 - 8 = 240 bytes

% You said your mini file-chunk header is 10 bytes:
miniHdrBytes = 10;
maxDataBytes = maxProtoPayload - miniHdrBytes;          % 240 - 10 = 230 bytes
if maxDataBytes <= 0
    error('Not enough payload for file data.');
end

% FEC layout
databitsLen   = 8 * msgCapBytes;   % 248 * 8 = 1984 bits
L_in          = databitsLen + Kminus1;   % 1984 + 2 = 1986
codedBitsLen  = 2 * L_in;               % 3972 bits
bitsAfterPilot = infoBitsLen - pilotBitsLen;  % 4096 - 100 = 3996
padBitsLen    = bitsAfterPilot - codedBitsLen;  % 3996 - 3972 = 24 bits

if padBitsLen < 0
    error('TX: FEC layout invalid (padBitsLen < 0).');
end

fprintf('TX protocol+FEC:\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  T = L_in        : %d time steps\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);
fprintf('  maxDataBytes/chunk (file) = %d bytes\n', maxDataBytes);

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

%% ---------- Reader: FILE source ----------
fileName = 'U:\Chalmers\MCC125\codes\shirzad\onsala.xlsx';  % file to send
fileId   = uint8(1);

% FileChunkReader will build payload bytes:
%   [10-byte mini header | up to maxDataBytes of file data]
msgReader = io.FileChunkReader(fileName, fileId, maxDataBytes, true);

%% ---------- USRP sink ----------
txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',           '192.168.10.5', ...
  'CenterFrequency',     fc, ...
  'MasterClockRate',     MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',                txGain_dB, ...
  'UseExternalRef',      false);

disp('TX: streaming FILE frames via USRP. Ctrl+C to stop.');

k = 0;
seqNum = uint16(0);
globalSampleIndex = 0; %#ok<NASGU>

while true
    %% ---------- Get datagram payload from FileChunkReader ----------
    % payload: uint8 column, length <= maxProtoPayload
    [payload, n, eof] = msgReader.read(maxProtoPayload); %#ok<NASGU>
    
    if n == 0
        % In loop mode this shouldn't happen, but just in case
        payload = uint8([]);
    else
        payload = payload(1:n);
    end
    
    % Single-datagram message per frame => START + END flags
    flags = bitor(protocol.Datagram.FLAG_START, protocol.Datagram.FLAG_END);
    
    %% ---------- Build protocol datagram ----------
    dgram = protocol.Datagram(seqNum, flags, payload, uint8(0));
    seqNum = seqNum + uint16(1);
    
    protoBytes = dgram.toBytes(msgCapBytes);   % always 248 bytes
    
    %% ---------- bytes -> bits ----------
    dataBitsMatrix = de2bi(protoBytes, 8, 'left-msb').';
    dataBits       = dataBitsMatrix(:);       % 1984 bits
    
    %% ---------- FEC encode (terminated) ----------
    uBits = logical(dataBits);             % 1984 bits
    codedBits = enc.encode(uBits, true);   % 2*(1984+2) = 3972 bits
    codedBits = double(codedBits(:));
    
    if numel(codedBits) ~= codedBitsLen
        warning('TX: codedBits length %d != expected %d', ...
            numel(codedBits), codedBitsLen);
    end
    
    %% ---------- Full info bits = [pilotBits; codedBits; padBits] ----------
    padBits = zeros(padBitsLen,1);   % 24 bits
    infoBits = [pilotBits; codedBits; padBits];  % 100 + 3972 + 24 = 4096 bits
    
    %% ---------- map to QPSK symbols ----------
    paySyms = modQPSK.modulate(infoBits);   % 2048 symbols
    
    %% ---------- full frame = [preamble; payload] ----------
    frmSyms_raw = [preSyms; paySyms];       % 256 + 2048 = 2304 symbols
    
    pilotAmp = 0.2;  % modest DC pilot; used by super-coarse CFO at RX
    frmSyms  = pilotAmp + frmSyms_raw;
    
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
