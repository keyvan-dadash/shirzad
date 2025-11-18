% RX over UDP: QPSK with repeated-preamble detection (Schmidl-style)
%     + CFO pre-corr + PLL + decision-directed SNR (NO FEC)
clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP receiver support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 9.999e6;     % only used in USRP mode
MasterClockRate = 100e6;
Fs              = 1e6;
Decim           = MasterClockRate/Fs;  %#ok<NASGU>

M   = 4;                      % QPSK
bps = log2(M);
sps = 10; beta = 0.35; span = 10;

% Preamble: two identical halves [a, a]
preambleHalfLen = 32;         % symbols per half
preambleLen     = 2 * preambleHalfLen;   % total preamble length
payloadSyms     = 270;
rxGain_dB       = 0;

SamplesPerFrame = 4000;

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

%% ---------- Payload length & reference bits (NO FEC) ----------
infoBitsLen = payloadSyms * bps;   % for QPSK: 270*2 = 540 bits

rng(1001);
refBits_info = randi([0 1], infoBitsLen, 1);

%% ---------- Preamble (for reference / debugging only) ----------
rng(42);
preBitsHalf = randi([0 1], preambleHalfLen*bps, 1);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];  %#ok<NASGU>  % not used in detector

% Hard demapper
qamDemBits = @(z) demQPSK.demodulateHard(z);

%% ---------- DSP chain ----------
rrcRX  = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    1000, ...
    'MaximumGain_dB',     30, ...
    'AdaptationStepSize', 1e-3, ...
    'TargetPower',        1.0);

dcblock = filters.DcBlocker('Length',64);

% Carrier sync (PLL) at 1 sps
carSyncCoarse = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.01);

carSyncFine = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.002);

carSyncNow = carSyncCoarse;
useFine    = false;

%% ---------- New preamble detector ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  0.2, ...
    'MinWindowPower',   1e-6);

%% ---------- Source & Sink ----------
udpPort = 31000;

% rfSrc = sources.UDPWaveformSource( ...
%     'LocalIPPort',          udpPort, ...
%     'SampleRate',           Fs, ...
%     'MaximumMessageLength', SamplesPerFrame, ...
%     'IsMessageComplex',     true, ...
%     'MessageDataType',      'double', ...
%     'Blocking',             true, ...
%     'TimeoutSeconds',       1.0);

rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        '192.168.10.4', ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

% If you want USRP instead, comment above and use SDRuBasebandSource as before.

paySink = sinks.PayloadCollectorSink();

%% ---------- Spectrum analyzer ----------
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

%% ---------- Receive/decode loop ----------
disp('RX (UDP): waiting for frames…');
yBuf   = complex([]); 
frames = 0;
totErr = 0;
totBits = 0;

while true
    % Pull chunk
    [x, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid
        pause(0.005);
        continue;
    end

    x = dcblock.process(x);
    if ~useFine
        x = agc.process(x);
    end

    % Matched filter at sample rate
    y = rrcRX.process(x);

    sa(y);

    % Accumulate in buffer
    yBuf = [yBuf; y]; %#ok<AGROW>
    maxHold = 2*(preambleLen*sps + payloadSyms*sps) + 8*sps;
    if numel(yBuf) > maxHold
        yBuf = yBuf(end-maxHold+1:end);
    end
    if numel(yBuf) < preambleLen*sps + payloadSyms*sps + 8*sps
        continue;
    end

    %% ======== NEW: Repeated preamble detection + CFO estimate ========
    detRes = preDet.detect(yBuf);
    if ~detRes.Found
        % Uncomment for debugging:
        % fprintf('Detector not found: Metric=%.3f, Pow=%.3f\n', ...
        %         detRes.Metric, detRes.WindowPower);
        continue;
    end

    off          = detRes.SampleOffset;      % sample offset (0..sps-1)
    preStartSym  = detRes.PreambleStartSym;  % 1-based index at symbol-rate
    wSym         = detRes.CfoRadPerSym;      % rad/symbol (CFO)

    % Build symbol-rate stream for this offset
    ySym = yBuf(1+off : sps : end);
    Nsym = numel(ySym);

    payStartS = preStartSym + preambleLen;   % preamble length = 2 * half
    if (payStartS + payloadSyms - 1) > Nsym
        % Not enough symbols yet in buffer
        continue;
    end

    % CFO correction at symbol-rate:
    kSym = (0:Nsym-1).';
    ySym_cfo = ySym .* exp(-1j * wSym * kSym);

    % Carve payload from CFO-corrected symbols
    rxSyms_raw = ySym_cfo(payStartS : payStartS + payloadSyms - 1);

    %% ---- carrier/phase recovery at 1 sps (coarse then fine) ----
    rxSyms_eq = carSyncNow.process(rxSyms_raw);

    %% ---- Resolve ±/±j quadrant using reference bits ----
    G = [1, -1, 1j, -1j];
    errs = zeros(1,4);
    for g = 1:4
        rb = qamDemBits(rxSyms_eq * G(g));
        K  = min(numel(rb), numel(refBits_info));
        errs(g) = mean(rb(1:K) ~= refBits_info(1:K));
    end
    [~, ig] = min(errs);
    rxSyms = rxSyms_eq * G(ig);

    %% ===== Hard-decision demod and BER (NO FEC) =====
    rxBits = qamDemBits(rxSyms);

    errMask = (rxBits(1:infoBitsLen) ~= refBits_info);
    errIdx  = find(errMask);

    fprintf('Error indices this frame: ');
    disp(errIdx.');

    frameBER = mean(rxBits(1:infoBitsLen) ~= refBits_info);
    totErr   = totErr + sum(rxBits(1:infoBitsLen) ~= refBits_info);
    totBits  = totBits + infoBitsLen;
    cumBER   = totErr / max(totBits,1);

    payBits = rxBits(1:infoBitsLen);

    %% ---- decision-directed EVM -> Es/N0 (clean SNR) ----
    hb2 = qamDemBits(rxSyms);
    zh2 = modQPSK.modulate(hb2);
    cHd = (zh2' * rxSyms) / (zh2' * zh2 + eps);
    err = rxSyms - cHd * zh2;
    Es  = mean(abs(cHd * zh2).^2);
    Nv  = mean(abs(err).^2);
    SNRdB = 10*log10(max(Es/Nv, eps));

    %% ---- frame count & sink write ----
    frames = frames + 1;
    paySink.writeFrame(payBits, struct('FrameIndex',frames));

    fprintf(['Frame %d | off=%d | M=%.3f | Pow=%.2f | ' ...
             'BER=%.3e | CUM=%.3e (%d bits) | Es/N0≈%.1f dB | CFO(rad/sym)=%.3g\n'], ...
            frames, off, detRes.Metric, detRes.WindowPower, ...
            frameBER, cumBER, totBits, SNRdB, wSym);

    %% ---- tighten loops & freeze AGC after a few frames ----
    if ~useFine && frames >= 5
        agc.AdaptationStepSize = 1e-9;
        carSyncNow = carSyncFine;
        useFine = true;
    end

    %% ---- drop consumed samples from yBuf ----
    % We consumed up to the end of the payload plus some tail
    needTail_guard = 4*sps;
    % Convert symbol indices back to sample index in yBuf:
    lastSymIdx   = payStartS + payloadSyms - 1;       % in ySym
    lastSampleIx = 1 + off + (lastSymIdx-1)*sps;      % in yBuf
    end_consumed = min(lastSampleIx + needTail_guard - 1, numel(yBuf));
    yBuf = yBuf(end_consumed+1 : end);
end
