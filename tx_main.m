clear; clc;

%% ---------- Params ----------
fc              = 10e6;
MasterClockRate = 100e6;
Fs              = 1e6;
Interp          = MasterClockRate/Fs;

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleLen = 128;
payloadSyms = 270;
txGain_dB   = 0;

modQPSK = modulators.QpskModulator();

%% ---------- Fixed known payload (no FEC) ----------
infoBitsLen = payloadSyms * bps;   % 270*2 = 540 bits

% This must match RX:
rng(1001);
payloadBits_info = randi([0 1], infoBitsLen, 1);
paySyms = modQPSK.modulate(payloadBits_info);

% Preamble (uncoded, fixed)
rng(42);
preBits = randi([0 1], preambleLen*bps, 1);
preSyms = modQPSK.modulate(preBits);

%% ---------- RRC filter (STREAMING) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- UDP sink ----------
txSink = sinks.UDPWaveformSink( ...
    'RemoteIPAddress', '127.0.0.1', ...
    'RemoteIPPort',    31000, ...
    'SampleRate',      Fs);

% txSink = sinks.SDRuWaveformSink( ...
%   'IPAddress',         '192.168.10.5', ...
%   'CenterFrequency',   fc, ...
%   'MasterClockRate',   MasterClockRate, ...
%   'InterpolationFactor', Interp, ...
%   'Gain',              txGain_dB, ...
%   'UseExternalRef',    false);

sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','TX waveform spectrum');

disp('TX: streaming frames via UDP. Ctrl+C to stop.');

k = 0;
cfoHz = 1000;
Fs    = 1e6;

globalSampleIndex = 0;   % or make it persistent in a function

while true
    % Same frame symbols every time
    frmSyms = [preSyms; paySyms];

    % Up-sample by sps
    up = zeros(numel(frmSyms)*sps, 1);
    up(1:sps:end) = frmSyms;

    % IMPORTANT: use txRRC.process *inside* the loop
    % so filter state is continuous across frames
    txWave = txRRC.process(up);

    % Normalization (optional)
    txWave = txWave ./ max(abs(txWave)) * 0.8;

    N = numel(txWave);
    n = (0:N-1).' + globalSampleIndex;   % global sample index for this block
    
    txWave = txWave .* exp(1j*2*pi*cfoHz*n/Fs);
    
    globalSampleIndex = globalSampleIndex + N;

    sa(txWave);
    txSink.writeFrame(txWave, struct('FrameIndex', k+1));

    k = k+1;
    if mod(k,50) == 0
        fprintf('TX sent %d frames...\n', k);
    end
end
