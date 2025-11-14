clear; clc;

assert(exist('comm.SDRuTransmitter','class')==8, ...
  ['Install "Communications Toolbox Support Package for USRP Radio". ', ...
   'Home > Add-Ons > Get Hardware Support Packages.']);

%% ---------- Shared params (match RX) ----------
fc              = 10e6;
MasterClockRate = 100e6;
Fs              = 1e6;
Interp          = MasterClockRate/Fs;  assert(round(Interp)==Interp,'Interp must be integer');

M   = 4;  bps = log2(M);   % QPSK
sps = 10; beta = 0.35; span = 10;

preambleLen = 128;
payloadSyms = 270;
txGain_dB   = 3;
useExternalRef = false;

modQPSK = modulators.QpskModulator();

%% ---------- FEC ----------
R    = 1/2;
trel = poly2trellis(7,[171 133]);
convEnc = comm.ConvolutionalEncoder( ...
    'TrellisStructure', trel, ...
    'TerminationMethod','Truncated');

infoBitsLen = round(payloadSyms * bps * R);

%% ---------- Sources & Sinks ----------
% Payload source (replaces rng + randi in original)
payloadSrc = sources.PayloadBitSource(infoBitsLen, 'Seed', 1001);

% TX sink (USRP)
% txSink = sinks.SDRuWaveformSink( ...
%     'IPAddress',          '192.168.10.5', ...
%     'CenterFrequency',    fc, ...
%     'MasterClockRate',    MasterClockRate, ...
%     'InterpolationFactor',Interp, ...
%     'Gain',               txGain_dB, ...
%     'UseExternalRef',     useExternalRef);

txSink = sinks.UDPWaveformSink( ...
    'RemoteIPAddress', '127.0.0.1', ...
    'RemoteIPPort',    31000, ...
    'SampleRate',      Fs);

%% ---------- Waveform chain (unchanged algorithms) ----------
% Preamble (uncoded)
rng(42);
preBits = randi([0 1], preambleLen*bps, 1);
% preSyms = qammod(preBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);
preSyms = modQPSK.modulate(preBits);

% Pulse shaping filter
txRRC  = comm.RaisedCosineTransmitFilter( ...
    'RolloffFactor',beta,'FilterSpanInSymbols',span,'OutputSamplesPerSymbol',sps);

% Spectrum analyzer (optional)
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, 'SpectrumType','Power density', ...
    'Title','TX waveform spectrum');

disp('TX: streaming frames via SDRuWaveformSink. Ctrl+C to stop.');

k = 0;
while true
    % ---- get payload bits from Source ----
    [payloadBits_info, infoPay] = payloadSrc.readFrame();

    % ---- encode and modulate as in original ----
    encBits = convEnc(payloadBits_info);           % length = payloadSyms*bps
    % paySyms = qammod(encBits, M, 'gray', ...
                     % 'InputType','bit','UnitAveragePower', true);
    paySyms = modQPSK.modulate(encBits);

    frmSyms = [preSyms; paySyms];
    txWave  = txRRC(frmSyms);
    txWave  = txWave ./ max(abs(txWave)) * 0.8;

    % ---- visualize + send via Sink ----
    sa(txWave);
    txSink.writeFrame(txWave, struct('FrameIndex', infoPay.FrameIndex));

    k = k+1;
    if mod(k,50)==0
        fprintf('TX sent %d frames...\n',k);
    end
end
