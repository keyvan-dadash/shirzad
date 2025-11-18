% RX over USRP/UDP: QPSK with repeated-preamble detection [a,a]
%   + FFT-based coarse CFO estimate (symbol-rate, from known preamble)
%   + CFO correction at SAMPLE RATE (before RRC in the DATA path)
%   + PLL (decision-directed) for fine carrier/phase tracking
%   + decision-directed Es/N0 estimate
%   + Constellation viewer (post-PLL, post-quadrant-fix)
%   NO FEC (uncoded QPSK payload)

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 9.97e6;     % RX center frequency (USRP mode)
MasterClockRate = 100e6;
Fs              = 1e6;      % complex baseband sample rate
Decim           = MasterClockRate/Fs;  %#ok<NASGU>

M   = 4;                    % QPSK
bps = log2(M);
sps = 10;                   % samples per symbol
beta = 0.35;
span = 10;                  % RRC span in symbols

% Preamble: two identical halves [a, a]
preambleHalfLen = 64;       % symbols per half
preambleLen     = 2 * preambleHalfLen;   % total preamble length
payloadSyms     = 270;      % symbols in payload
rxGain_dB       = 0;

SamplesPerFrame = 4000;

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

%% ---------- Payload length & reference bits (NO FEC) ----------
infoBitsLen = payloadSyms * bps;   % for QPSK: 270*2 = 540 bits

% Reference bits (must match TX seed & length)
rng(1001);
refBits_info = randi([0 1], infoBitsLen, 1);

%% ---------- Preamble (known at RX, for CFO etc.) ----------
mseqGen = training.MSequenceGenerator( ...
    'Degree', 9);   % period = 2^9 - 1 = 511 >= preambleHalfLen * bps

% We need preambleHalfLen * bps bits for ONE half (QPSK -> 2 bits/sym)
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);

% Map bits to QPSK using your existing modulator
preSymsHalf = modQPSK.modulate(preBitsHalf);

% Schmidl preamble: [a, a]
preSyms = [preSymsHalf; preSymsHalf];


% Hard demapper
qamDemBits = @(z) demQPSK.demodulateHard(z);

% Symbol rate
Rsym = Fs / sps;  % 1e6 / 10 = 100 ksym/s

%% ---------- DSP chain (detection path) ----------
% This RRC is used on the *raw* (CFO-uncorrected) stream just to detect
% preamble and estimate CFO. Data path will be re-filtered after CFO corr.
rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    1000, ...
    'MaximumGain_dB',     30, ...
    'AdaptationStepSize', 1e-3, ...
    'TargetPower',        1.0);

dcblock = filters.DcBlocker('Length',64);

% Carrier sync (decision-directed PLL) at 1 sps (data path)
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

%% ---------- Repeated preamble detector (Schmidl-style) ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  0.2, ...
    'MinWindowPower',   1e-6);

%% ---------- FFT-based coarse CFO estimator (symbol-rate) ----------
% We use the estimator in *stateless* mode and track CFO in the RX,
% gated by BER and preamble metric.
fftCfoEst = sync.FftCfoEstimator( ...
    'SampleRateSym', Rsym, ...
    'PreambleSyms',  preSyms, ...
    'Nfft',          4096, ...
    'UseHistory',    false, ...
    'NumCandidates', 3);

% CFO tracking (in RX)
cfoInitialized     = false;
fCfoHz_trk         = 0;      % tracked CFO (Hz), used for sample-rate correction
cfoAlpha           = 0.2;    % smoothing step
cfoMaxJumpHz       = 200;    % max CFO change per *trusted* frame
berTrustThresh     = 0.001;   % only use CFO from frames with BER < 15%
metricTrustThresh  = 0.24;   % only trust CFO when preamble metric is decent

%% ---------- Source & Sink ----------
udpPort = 31000; %#ok<NASGU>

% --- USRP source ---
rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        '192.168.10.4', ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

% If you want UDP loopback instead, comment SDRuBasebandSource and use:
% rfSrc = sources.UDPWaveformSource( ...
%     'LocalIPPort',          udpPort, ...
%     'SampleRate',           Fs, ...
%     'MaximumMessageLength', SamplesPerFrame, ...
%     'IsMessageComplex',     true, ...
%     'MessageDataType',      'double', ...
%     'Blocking',             true, ...
%     'TimeoutSeconds',       1.0);

paySink = sinks.PayloadCollectorSink();

%% ---------- Spectrum analyzer ----------
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

%% ---------- Constellation viewer (Comm System Toolbox) ----------
constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-quadrant-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);   % AGC’d baseband samples (before RRC, before CFO corr)
yDetBuf = complex([]);   % matched-filtered samples for detection (after rrcDet)

frames = 0;
totErr = 0;
totBits = 0;

Lpre       = numel(preSyms);       % = preambleLen in symbols
maxHoldSam = 2*(preambleLen*sps + payloadSyms*sps) + 8*sps + span*sps;
rrcSpanSam = span * sps;

while true
    %% ---------- Pull chunk from source ----------
    [xRaw, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid
        pause(0.005);
        continue;
    end

    %% ---------- DC blocker + AGC ----------
    xDC = dcblock.process(xRaw);
    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    % For debug, show spectrum of AGC output
    sa(xAGC);

    %% ---------- Detection path: RRC on AGC output ----------
    yDet = rrcDet.process(xAGC);

    %% ---------- Append to buffers ----------
    xBuf    = [xBuf;    xAGC]; %#ok<AGROW>
    yDetBuf = [yDetBuf; yDet]; %#ok<AGROW>

    % Limit buffer size
    if numel(xBuf) > maxHoldSam
        extra   = numel(xBuf) - maxHoldSam;
        xBuf    = xBuf(   extra+1:end);
        yDetBuf = yDetBuf(extra+1:end);
    end

    % Need at least preamble + payload + some guard
    if numel(xBuf) < preambleLen*sps + payloadSyms*sps + 8*sps
        continue;
    end

    %% ======== Repeated preamble detection (Schmidl metric) ========
    detRes = preDet.detect(yDetBuf);
    if ~detRes.Found
        % Uncomment if you want to watch the metric:
        % fprintf('No preamble: M=%.3f, Pow=%.3g\n', detRes.Metric, detRes.WindowPower);
        continue;
    end

    off         = detRes.SampleOffset;      % sample offset (0..sps-1)
    preStartSym = detRes.PreambleStartSym;  % 1-based index at 1 sps (symbol-rate)

    %% ---------- Build symbol-rate stream from detection path ----------
    ySymDet = yDetBuf(1+off : sps : end);   % 1 sample per symbol (not CFO-corrected)
    NsymDet = numel(ySymDet);

    % Ensure full preamble and payload are present at 1 sps
    if preStartSym + Lpre - 1 > NsymDet
        continue;
    end

    payStartS = preStartSym + preambleLen;
    if payStartS + payloadSyms - 1 > NsymDet
        continue;
    end

    %% === FFT-based coarse CFO estimate at symbol rate (on ySymDet) ===
    [wSym_meas, fCfoHz_meas, cfoPeak] = fftCfoEst.estimate(ySymDet, preStartSym); %#ok<NASGU>

    % --------- Choose CFO to USE for this frame ------------
    % We use the tracked CFO (from previous good frames).
    % Only for the very first accepted frame do we fall back to the raw measurement.
    if cfoInitialized
        fCfoHz_use = fCfoHz_trk;
    else
        % Bootstrap with the first measurement
        fCfoHz_use = fCfoHz_meas;
    end

    wSym_use = 2*pi * fCfoHz_use / Rsym;  % rad/symbol (for info/log)
    wSample  = 2*pi * fCfoHz_use / Fs;    % rad/sample (used for correction)

    %% === CFO correction at SAMPLE RATE (data path, LOCAL SEGMENT) ===
    % Map symbol indices (at 1 sps) to sample indices in xBuf/yDetBuf.
    % Symbol k (1-based in ySymDet) sits at sample:
    %   n_global = 1 + off + (k-1)*sps

    payEndS = payStartS + payloadSyms - 1;

    globalPayStart = 1 + off + (payStartS-1)*sps;
    globalPayEnd   = 1 + off + (payEndS  -1)*sps;

    % Choose a segment around the payload, with RRC span as guard
    segStart = max(1,              globalPayStart - rrcSpanSam);
    segEnd   = min(numel(xBuf),    globalPayEnd   + rrcSpanSam);

    xSeg = xBuf(segStart:segEnd);

    % Local sample indices for the segment
    nSeg     = (0:numel(xSeg)-1).';          % local 0..L-1
    xSeg_cfo = xSeg .* exp(-1j * wSample .* nSeg);

    % Matched filter on CFO-corrected data path (fresh RRC per frame)
    rrcDataLocal = filters.RootRaisedCosineFilter(beta, span, sps);
    ySeg_cfo     = rrcDataLocal.process(xSeg_cfo);

    % Symbol-rate CFO-corrected payload stream:
    % First payload symbol sample (global) -> index in segment:
    payStartSegIdx = globalPayStart - segStart + 1;    % 1-based
    lastPayloadSegIdx = payStartSegIdx + (payloadSyms-1)*sps;

    if lastPayloadSegIdx > numel(ySeg_cfo)
        % Safety check (should not happen if buffers are OK)
        continue;
    end

    rxSyms_raw = ySeg_cfo(payStartSegIdx : sps : lastPayloadSegIdx);

    %% ---- carrier/phase recovery at 1 sps (coarse then fine) ----
    rxSyms_eq = carSyncNow.process(rxSyms_raw);

    %% ---- Resolve ±/±j quadrant using reference bits ----
    G    = [1, -1, 1j, -1j];
    errs = zeros(1,4);
    for g = 1:4
        rb = qamDemBits(rxSyms_eq * G(g));
        K  = min(numel(rb), numel(refBits_info));
        errs(g) = mean(rb(1:K) ~= refBits_info(1:K));
    end
    [~, ig] = min(errs);
    rxSyms = rxSyms_eq * G(ig);

    %% ---- show constellation (post-PLL, post-quadrant-fix) ----
    constDiag(rxSyms);

    %% ===== Hard-decision demod and BER (NO FEC) =====
    rxBits  = qamDemBits(rxSyms);
    errMask = (rxBits(1:infoBitsLen) ~= refBits_info);
    errIdx  = find(errMask);

    fprintf('Error indices this frame: ');
    disp(errIdx.');

    frameBER = mean(errMask);
    totErr   = totErr + sum(errMask);
    totBits  = totBits + infoBitsLen;
    cumBER   = totErr / max(totBits, 1);

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

    fprintf(['Frame %d | off=%d | M=%.3f | Pow=%.3g | ' ...
             'BER=%.3e | CUM=%.3e (%d bits) | Es/N0≈%.1f dB | ' ...
             'CFO_used≈%.1f Hz (%.3g rad/sym)\n'], ...
            frames, off, detRes.Metric, detRes.WindowPower, ...
            frameBER, cumBER, totBits, SNRdB, ...
            fCfoHz_use, wSym_use);

    %% ---- CFO tracking update (BER- and metric-gated) ----
    if detRes.Metric >= metricTrustThresh && frameBER <= berTrustThresh
        if ~cfoInitialized
            % First good frame: initialize tracked CFO from measurement
            fCfoHz_trk   = fCfoHz_meas;
            cfoInitialized = true;
        else
            % Smooth toward measurement, but limit per-frame jump
            df = fCfoHz_meas - fCfoHz_trk;
            if abs(df) > cfoMaxJumpHz
                df = sign(df) * cfoMaxJumpHz;
            end
            fCfoHz_trk = fCfoHz_trk + cfoAlpha * df;
        end
    else
        % Bad frame (high BER or weak metric): do NOT update CFO track
        % (fCfoHz_trk stays as it was)
    end

    %% ---- tighten loops & freeze AGC after a few frames ----
    if ~useFine && frames >= 15
        agc.AdaptationStepSize = 1e-9;  % effectively frozen
        carSyncNow = carSyncFine;       % switch to narrow PLL
        useFine    = true;
    end

    %% ---- drop consumed samples from xBuf & yDetBuf ----
    % We consumed up to the end of the payload plus some guard samples.
    needTail_guard = 4*sps;
    lastSymIdx   = payStartS + payloadSyms - 1;     % in symbol-rate index
    lastSampleIx = 1 + off + (lastSymIdx-1)*sps;    % in sample index
    end_consumed = min(lastSampleIx + needTail_guard - 1, numel(xBuf));

    xBuf    = xBuf(   end_consumed+1 : end);
    yDetBuf = yDetBuf(end_consumed+1 : end);
end
