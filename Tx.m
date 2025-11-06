% TX: QPSK/16-QAM over USRP N210 with RRC + preamble (continuous frames)
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
sps = 8;  beta = 0.35; span = 10;

preambleLen = 128;           % symbols
payloadSyms = 2000;          % symbols per frame
txGain_dB   = 0;
useExternalRef = false;

%% ---------- Waveform ----------
rng(1001); payloadBits = randi([0 1], payloadSyms*bps, 1);
paySyms = qammod(payloadBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

rng(42);   preBits = randi([0 1], preambleLen*bps, 1);
preSyms = qammod(preBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

txRRC = comm.RaisedCosineTransmitFilter( ...
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
