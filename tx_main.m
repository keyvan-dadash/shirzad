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

%% ---------- Payload structure: pilot + {len + msg + padding} ----------
infoBitsLen  = payloadSyms * bps;   % 270*2 = 540 bits
pilotBitsLen = 100;                 % must match RX
msgCapBits   = infoBitsLen - pilotBitsLen;
msgCapBytes  = msgCapBits/8;       % should be 55 with these params
maxMsgBytes  = msgCapBytes - 1;    % 1 byte for length => 54 bytes max

% Known pilot bits (must match RX)
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
% This can later be swapped with a FileReader, NetworkReader, etc.
msgReader = io.FixedMessageReader('Hello from TX via USRP!', true);

%% ---------- USRP sink ----------
txSink = sinks.SDRuWaveformSink( ...
  'IPAddress',         '192.168.10.5', ...
  'CenterFrequency',   fc, ...
  'MasterClockRate',   MasterClockRate, ...
  'InterpolationFactor', Interp, ...
  'Gain',              txGain_dB, ...
  'UseExternalRef',    false);

% sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
%     'PlotAsTwoSidedSpectrum',true, ...
%     'SpectrumType','Power density', ...
%     'Title','TX waveform spectrum');

disp('TX: streaming frames via USRP. Ctrl+C to stop.');

k = 0;
globalSampleIndex = 0;  %#ok<NASGU>

while true
    %% ---------- Get message bytes from Reader ----------
    [msgBytes, n, eof] = msgReader.read(maxMsgBytes);

    if n == 0
        % No data (EOF or empty) -> send empty message (len = 0)
        msgLen   = uint8(0);
        msgBytes = uint8([]);
    else
        msgBytes = msgBytes(1:n);
        msgLen   = uint8(n);  % guaranteed <= maxMsgBytes
    end

    % Fixed-size payload bytes: [len][msg][padding]
    payloadBytes    = zeros(msgCapBytes, 1, 'uint8');
    payloadBytes(1) = msgLen;
    if msgLen > 0
        payloadBytes(2:1+double(msgLen)) = msgBytes(:);
    end

    % bytes -> bits (column)
    dataBitsMatrix = de2bi(payloadBytes, 8, 'left-msb').';
    dataBits       = dataBitsMatrix(:);

    % full info bits = [pilotBits; dataBits]
    infoBits = [pilotBits; dataBits];   % length 540 bits

    % map to QPSK symbols
    paySyms = modQPSK.modulate(infoBits);

    % full frame = [preamble; payload]
    frmSyms = [preSyms; paySyms];

    % upsample & RRC
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);

    % Optional CFO injection (kept commented)
    % N       = numel(txWave);
    % n       = (0:N-1).' + globalSampleIndex;
    % cfoHz   = 0;  % or 1000, etc.
    % txWave  = txWave .* exp(1j*2*pi*cfoHz*n/Fs);
    % globalSampleIndex = globalSampleIndex + N;

    % sa(txWave);

    txSink.writeFrame(txWave, struct('FrameIndex', k+1));
    k = k + 1;

    if mod(k,50) == 0
        fprintf('TX sent %d frames...\n', k);
    end
end
