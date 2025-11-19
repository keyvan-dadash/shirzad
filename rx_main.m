% RX over USRP/UDP: QPSK with repeated-preamble detection [a,a]
%   + FFT-based coarse CFO estimate (symbol-rate, from known preamble)
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + PLL (decision-directed) for fine carrier/phase tracking
%   + decision-directed Es/N0 estimate
%   + Constellation viewer (post-PLL, post-quadrant-fix)
%   NO FEC (uncoded QPSK payload)

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;     % RX center frequency (USRP mode)
MasterClockRate = 100e6;
Fs              = 1e6;        % complex baseband sample rate
Decim           = MasterClockRate/Fs;  %#ok<NASGU>

M   = 4;                      % QPSK
bps = log2(M);
sps = 10;                     % samples per symbol
beta = 0.35;
span = 10;                    % RRC span in symbols

% Preamble: two identical halves [a, a]
preambleHalfLen = 64;         % symbols per half
preambleLen     = 2 * preambleHalfLen;   % total preamble length
payloadSyms     = 270;        % symbols in payload
frameSyms       = preambleLen + payloadSyms;
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

preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];   % [a, a]

% Hard demapper
qamDemBits = @(z) demQPSK.demodulateHard(z);

% Symbol rate
Rsym = Fs / sps;  % 1e6 / 10 = 100 ksym/s

%% ---------- DSP chain (detection path) ----------
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
    'NormalizedLoopBandwidth',0.005);

carSyncNow = carSyncCoarse;
useFine    = false;

%% ---------- Repeated preamble detector (Schmidl-style) ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  0.2, ...
    'MinWindowPower',   1e-7);

%% ---------- FFT-based coarse CFO estimator (symbol-rate) ----------
fftCfoEst = sync.FftCfoEstimator( ...
    'SampleRateSym', Rsym, ...
    'PreambleSyms',  preSyms, ...
    'Nfft',          4096, ...
    'UseHistory',    false, ...
    'NumCandidates', 3);

% CFO tracking (in RX)
cfoInitialized     = false;
fCfoHz_trk         = 0;      % tracked CFO (Hz), used for symbol-rate correction
cfoAlpha           = 0.05;   % smoothing step
cfoMaxJumpHz       = 200;    % max CFO change per *trusted* frame
berTrustThresh     = 0.001;  % only use CFO from frames with BER < threshold
metricTrustThresh  = 0.24;   % only trust CFO when preamble metric is decent

%% ---------- Source & Sink ----------
udpPort = 31000; %#ok<NASGU>

rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        '192.168.10.4', ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

paySink = sinks.PayloadCollectorSink();

%% ---------- Spectrum analyzer ----------
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

%% ---------- Constellation viewer ----------
constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-quadrant-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);   % AGC’d baseband samples (before RRC)
yDetBuf = complex([]);   % matched-filtered samples for detection (after rrcDet)

frames = 0;
totErr = 0;
totBits = 0;

Lpre       = numel(preSyms);       % = preambleLen in symbols
frameSam   = frameSyms * sps;
maxHoldSam = 10*frameSam + 8*sps + span*sps;  % safety limit

while true
    %% ---------- Pull chunk from source ----------
    [xRaw, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid || numel(xRaw) < SamplesPerFrame
        pause(0.5);
        continue;
    end

    %% ---------- DC blocker + AGC ----------
    xDC = dcblock.process(xRaw);
    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    sa(xAGC);

    %% ---------- Detection path: RRC on AGC output ----------
    yDet = rrcDet.process(xAGC);

    %% ---------- Append to buffers ----------
    xBuf    = [xBuf;    xAGC]; %#ok<AGROW>
    yDetBuf = [yDetBuf; yDet]; %#ok<AGROW>

    % Safety: cap buffer size if detector has been failing for a while.
    if numel(xBuf) > maxHoldSam
        extra   = numel(xBuf) - maxHoldSam;
        xBuf(1:extra)    = [];
        yDetBuf(1:extra) = [];
        fprintf('maxHoldSam chop: dropped %d old samples\n', extra);
    end

    fprintf('size y and x: %.4f and %.4f and the input we got %.3f\n', numel(xBuf), numel(yDetBuf), numel(xRaw));

    %% ---------- INNER LOOP: process as many frames as possible ----------
    while true
        % Need at least preamble + payload + some guard at sample rate
        if numel(xBuf) < frameSam + 8*sps
            break;  % not enough samples yet
        end

        % ======== Repeated preamble detection (Schmidl metric) ========
        searchSyms   = frameSyms + 10;       % search only near the beginning
        maxDetectSam = searchSyms * sps;
        yDetSearch   = yDetBuf(1 : min(numel(yDetBuf), maxDetectSam));
        detRes       = preDet.detect(yDetSearch);

        if ~detRes.Found
            % No reliable Schmidl peak in the searched region yet
            % -> wait for more samples from USRP
            fprintf('No preamble: M=%.3f, Pow=%.3g\n', detRes.Metric, detRes.WindowPower);
            break;
        end

        off         = detRes.SampleOffset;      % sample offset (0..sps-1)
        preStartSym = detRes.PreambleStartSym;  % 1-based index at 1 sps

        %% ---------- Build symbol-rate stream from detection path ----------
        ySymDet = yDetBuf(1+off : sps : end);   % 1 sample per symbol
        NsymDet = numel(ySymDet);

        % Ensure full preamble and payload are present at 1 sps
        if preStartSym + Lpre - 1 > NsymDet
            % Not enough 1-sps symbols yet, need more samples from USRP
            break;
        end

        payStartS = preStartSym + preambleLen;
        if payStartS + payloadSyms - 1 > NsymDet
            % Not enough payload symbols yet
            break;
        end

        %% === FFT-based coarse CFO estimate at symbol rate (on ySymDet) ===
        [wSym_meas, fCfoHz_meas, cfoPeak] = fftCfoEst.estimate(ySymDet, preStartSym); %#ok<NASGU>

        if cfoInitialized
            fCfoHz_use = fCfoHz_trk;
        else
            fCfoHz_use = fCfoHz_meas;  % bootstrap
        end

        wSym_use = 2*pi * fCfoHz_use / Rsym;  % rad/symbol

        %% === CFO correction at SYMBOL RATE (1 sps) ===
        nSym      = (0:NsymDet-1).';              % 0..NsymDet-1
        ySym_cfo  = ySymDet .* exp(-1j * wSym_use .* nSym);

        % Preamble sanity check against *known* preamble
        preEndS  = preStartSym + preambleLen - 1;
        candPre  = ySym_cfo(preStartSym:preEndS);
        c        = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);
        if c < 0.7
            fprintf('Low corr with real preamble (c=%.2f), waiting for more samples.\n', c);
            % IMPORTANT: do NOT consume buffer here, otherwise we might
            % discard a real frame. Just break and let more samples arrive.
            break;
        end

        %% ---- Extract payload symbols from CFO-corrected 1-sps stream ----
        payEndS = payStartS + payloadSyms - 1;
        if payEndS > numel(ySym_cfo)
            break;
        end
        rxSyms_raw = ySym_cfo(payStartS : payEndS);

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

        constDiag(rxSyms);

        %% ===== Hard-decision demod and BER (NO FEC) =====
        rxBits  = qamDemBits(rxSyms);
        errMask = (rxBits(1:infoBitsLen) ~= refBits_info);
        errIdx  = find(errMask);

        fprintf("preamble starts at %.2f\n", preStartSym);
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
                fCfoHz_trk    = fCfoHz_meas;
                cfoInitialized = true;
            else
                df = fCfoHz_meas - fCfoHz_trk;
                if abs(df) > cfoMaxJumpHz
                    df = sign(df) * cfoMaxJumpHz;
                end
                fCfoHz_trk = fCfoHz_trk + cfoAlpha * df;
            end
        end

        %% ---- tighten loops & freeze AGC after a few frames ----
        if ~useFine && frames >= 15
            agc.AdaptationStepSize = 1e-9;  % effectively frozen
            carSyncNow = carSyncFine;       % switch to narrow PLL
            useFine    = true;
        end

        %% ---- drop consumed samples from xBuf & yDetBuf ----
        % Symbols we have fully processed: from the very first symbol
        % up to the END of the payload for this decoded frame.
        lastSymIdx   = payStartS + payloadSyms - 1;  % symbol-rate index
        lastSampleIx = 1 + off + (lastSymIdx-1)*sps; % sample index

        needTail_guard = 0;   % you can set a few*sps here if needed
        end_consumed    = min(lastSampleIx + needTail_guard, numel(xBuf));

        fprintf('the consume should be %.3f\n', end_consumed);

        xBuf(1:end_consumed)    = [];
        yDetBuf(1:end_consumed) = [];

        % Now loop again: maybe there is another whole frame already in the buffer
    end  % inner while
end  % outer while
