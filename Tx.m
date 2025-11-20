% TX: QPSK/16-QAM over USRP N210 with RRC + preamble + FEC (continuous frames)
clear; clc;

assert(exist('comm.SDRuTransmitter','class')==8, ...
  ['Install "Communications Toolbox Support Package for USRP Radio". ', ...
   'Home > Add-Ons > Get Hardware Support Packages.']);

%% ---------- Shared params (match RX) ----------
fc              = 10e6;     % Hz (use 10e6 only with LFRX/LFTX)
MasterClockRate = 100e6;
Fs              = 1e6;
Interp          = MasterClockRate/Fs;  assert(round(Interp)==Interp,'Interp must be integer');

M   = 4;  bps = log2(M);     % 4=QPSK, 16=16-QAM
sps = 10; beta = 0.35; span = 10;

preambleLen = 128;           % symbols
payloadSyms = 2000;          % symbols per frame (on-air payload)
txGain_dB   = 3;
useExternalRef = false;

%% ---------- FEC (rate 1/2, K=7) ----------
R    = 1/2;
trel = poly2trellis(7,[171 133]);     % octal polynomials (171,133)
convEnc = comm.ConvolutionalEncoder( ...
    'TrellisStructure', trel, ...
    'TerminationMethod','Truncated'); % fixed length, no tails

% Choose info length so coded length = payloadSyms*bps
infoBitsLen = round(payloadSyms * bps * R);   % for QPSK: 2000*2*1/2 = 2000

%% ---------- Waveform ----------
% Preamble (uncoded)
rng(42);
preBits = randi([0 1], preambleLen*bps, 1);
preSyms = qammod(preBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

% Payload (info -> encode -> QPSK). Seed must match RX's BER reference.
rng(1001);
payloadBits_info = randi([0 1], infoBitsLen, 1);
encBits = convEnc(payloadBits_info);               % length = payloadSyms*bps
paySyms = qammod(encBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

% Pulse shaping
txRRC  = comm.RaisedCosineTransmitFilter( ...
    'RolloffFactor',beta,'FilterSpanInSymbols',span,'OutputSamplesPerSymbol',sps);
frmSyms = [preSyms; paySyms];
txWave  = txRRC(frmSyms);
txWave  = txWave ./ max(abs(txWave)) * 0.8;

%% ---------- Radio ----------
tx = comm.SDRuTransmitter('Platform','N200/N210/USRP2', ...
    'IPAddress','192.168.10.5', ...
    'CenterFrequency',fc, ...
    'MasterClockRate',MasterClockRate, ...
    'InterpolationFactor',Interp, ...
    'Gain',txGain_dB, ...
    'TransportDataType','int16');

if useExternalRef
    try, tx.ClockSource='External'; tx.PPSSource='External'; end
end

disp('TX: streaming frames. Ctrl+C to stop.');
k=0;
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, 'SpectrumType','Power density', ...
    'Title','TX waveform spectrum');

while true
    sa(txWave);
    tx(txWave);
    k=k+1; if mod(k,50)==0, fprintf('TX sent %d frames…\n',k); end
end
