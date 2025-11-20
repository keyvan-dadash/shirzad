% RX over USRP: QPSK with repeated-preamble detection [a,a]
%   + FFT-based coarse CFO estimate (symbol-rate, from known preamble)
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + PLL (decision-directed) for fine carrier/phase tracking
%   + decision-directed Es/N0 estimate
%   + Message decode using protocol.Datagram (no BER)
%
%   NO FEC (uncoded QPSK payload)

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;     % RX center frequency (USRP mode)
MasterClockRate = 100e6;
Fs              = 1e6;      % complex baseband sample rate
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

%% ---------- Payload + pilot structure (NO FEC) ----------
infoBitsLen  = payloadSyms * bps;   % 540 bits
pilotBitsLen = 100;                 % must match TX
msgCapBits   = infoBitsLen - pilotBitsLen;
msgCapBytes  = msgCapBits/8;        % 55 bytes "protocol payload"

% Datagram header size (for documentation)
hdrBytes        = double(protocol.Datagram.HEADER_BYTES);  % 8
maxProtoPayload = msgCapBytes - hdrBytes;                  % 47 bytes

% Known pilot bits (must match TX)
rng(1001);
pilotBits = randi([0 1], pilotBitsLen, 1); %#ok<NASGU>

%% ---------- Preamble (known at RX, must match TX) ----------
mseqGen = training.MSequenceGenerator('Degree', 9);
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];   % [a, a]
Lpre        = numel(preSyms);

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
    'NormalizedLoopBandwidth',0.002);

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
fCfoHz_trk         = 0;
cfoAlpha           = 0.05;
cfoMaxJumpHz       = 200;
metricTrustThresh  = 0.24;

%% ---------- Source & Sink ----------
rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        '192.168.10.4', ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

paySink = sinks.PayloadCollectorSink();

%% ---------- Writer for decoded messages ----------
msgWriter = io.ConsoleWriter();

%% ---------- Constellation viewer ----------
constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-quadrant-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);   % AGC’d baseband samples (before RRC)
yDetBuf = complex([]);   % matched-filtered samples (after rrcDet)

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 10*frameSam + 8*sps + span*sps;

while true
    %% ---------- Pull chunk from source ----------
    [xRaw, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid
        pause(0.05);
        continue;
    end

    % Handle overrun (short reads etc.) robustly
    if srcInfo.Overrun
        fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
            numel(xRaw), SamplesPerFrame);

        xBuf    = [];
        yDetBuf = [];
        useFine = false;
        agc.AdaptationStepSize = 1e-3;
        cfoInitialized = false;
        fCfoHz_trk     = 0;
        continue;
    end

    %% ---------- DC blocker + AGC ----------
    xDC = dcblock.process(xRaw);
    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    %% ---------- Detection path: RRC on AGC output ----------
    yDet = rrcDet.process(xAGC);

    %% ---------- Append to buffers ----------
    xBuf    = [xBuf;    xAGC]; %#ok<AGROW>
    yDetBuf = [yDetBuf; yDet]; %#ok<AGROW>

    if numel(xBuf) > maxHoldSam
        extra = numel(xBuf) - maxHoldSam;
        xBuf(1:extra)    = [];
        yDetBuf(1:extra) = [];
        fprintf('maxHoldSam chop: dropped %d old samples\n', extra);
    end

    %% ---------- INNER LOOP ----------
    while true
        if numel(xBuf) < frameSam + 8*sps
            break;
        end

        % --- preamble detection ---
        searchSyms   = frameSyms + 10;
        maxDetectSam = searchSyms * sps;
        yDetSearch   = yDetBuf(1 : min(numel(yDetBuf), maxDetectSam));
        detRes       = preDet.detect(yDetSearch);

        if ~detRes.Found
            fprintf('No preamble: M=%.3f, Pow=%.3g\n', ...
                detRes.Metric, detRes.WindowPower);
            break;
        end

        off         = detRes.SampleOffset;
        preStartSym = detRes.PreambleStartSym;

        % --- 1-sps stream ---
        ySymDet = yDetBuf(1+off : sps : end);
        NsymDet = numel(ySymDet);

        if preStartSym + Lpre - 1 > NsymDet
            break;
        end

        payStartS = preStartSym + preambleLen;
        if payStartS + payloadSyms - 1 > NsymDet
            break;
        end

        % --- CFO estimate ---
        [wSym_meas, fCfoHz_meas, ~] = fftCfoEst.estimate(ySymDet, preStartSym); %#ok<NASGU>
        if cfoInitialized
            fCfoHz_use = fCfoHz_trk;
        else
            fCfoHz_use = fCfoHz_meas;
        end
        wSym_use = 2*pi * fCfoHz_use / Rsym;

        % --- CFO correction at 1 sps ---
        nSym     = (0:NsymDet-1).';
        ySym_cfo = ySymDet .* exp(-1j * wSym_use .* nSym);

        % --- preamble sanity check vs known preSyms ---
        preEndS = preStartSym + preambleLen - 1;
        candPre = ySym_cfo(preStartSym:preEndS);
        c       = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);
        if c < 0.7
            fprintf('Low corr with real preamble (c=%.2f), waiting for more samples.\n', c);
            break;
        end

        % --- extract payload symbols ---
        payEndS = payStartS + payloadSyms - 1;
        if payEndS > numel(ySym_cfo)
            break;
        end
        rxSyms_raw = ySym_cfo(payStartS:payEndS);

        % --- carrier/phase recovery ---
        rxSyms_eq = carSyncNow.process(rxSyms_raw);

        % --- Resolve ±/±j quadrant using KNOWN pilot bits ---
        G    = [1, -1, 1j, -1j];
        errs = zeros(1,4);
        for g = 1:4
            rb = qamDemBits(rxSyms_eq * G(g));
            K  = min(numel(rb), pilotBitsLen);
            errs(g) = mean(rb(1:K) ~= pilotBits(1:K));
        end
        [~, ig] = min(errs);
        rxSyms = rxSyms_eq * G(ig);

        % Constellation (scaled) just for visual sanity
        % scale = sqrt(2) / sqrt(mean(abs(rxSyms).^2) + eps);
        % constDiag(scale * rxSyms);

        % --- Es/N0 estimate ---
        hb2 = qamDemBits(rxSyms);
        zh2 = modQPSK.modulate(hb2);
        cHd = (zh2' * rxSyms) / (zh2' * zh2 + eps);
        err = rxSyms - cHd * zh2;
        Es  = mean(abs(cHd * zh2).^2);
        Nv  = mean(abs(err).^2);
        SNRdB = 10*log10(max(Es/Nv, eps));

        frames = frames + 1;

        % --- bits -> bytes (protocol payload) ---
        rxBits = qamDemBits(rxSyms);
        if numel(rxBits) < infoBitsLen
            fprintf('Frame %d: not enough bits (%d < %d)\n', ...
                frames, numel(rxBits), infoBitsLen);
            break;
        end

        dataBits = rxBits(pilotBitsLen+1 : infoBitsLen);   % drop pilot region
        dataBitsMatrix = reshape(dataBits, 8, []).';
        dataBytes      = uint8(bi2de(dataBitsMatrix, 'left-msb'));  % 55 bytes

        % --- parse protocol datagram ---
        [pkt, ok] = protocol.Datagram.fromBytes(dataBytes);

        % ok = true;

        if ~ok
            warning('Frame %d: datagram checksum FAILED (seq=%d). Dropping payload.', ...
                frames, pkt.SeqNum);
            pkt.debugPrint();
        else
            msgBytes = pkt.Payload(1 : pkt.PayloadLen);
            % deliver to Writer (ConsoleWriter prints text for you)
            msgWriter.write(msgBytes);
        end

        % Optionally store bits (even on checksum fail, if you want)
        payBits = dataBits;
        paySink.writeFrame(payBits, struct('FrameIndex', frames));

        fprintf(['Summary: off=%d | M=%.3f | Pow=%.3g | ' ...
                 'Es/N0≈%.1f dB | CFO_used≈%.1f Hz (%.3g rad/sym)\n'], ...
                off, detRes.Metric, detRes.WindowPower, ...
                SNRdB, fCfoHz_use, wSym_use);

        % --- CFO tracking update ---
        if detRes.Metric >= metricTrustThresh
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

        % --- tighten loops & freeze AGC after a few frames ---
        if ~useFine && frames >= 15
            agc.AdaptationStepSize = 1e-9;
            carSyncNow = carSyncFine;
            useFine    = true;
        end

        % --- drop consumed samples ---
        lastSymIdx   = payStartS + payloadSyms - 1;
        lastSampleIx = 1 + off + (lastSymIdx-1)*sps;
        end_consumed = min(lastSampleIx, numel(xBuf));

        xBuf(1:end_consumed)    = [];
        yDetBuf(1:end_consumed) = [];
    end
end
