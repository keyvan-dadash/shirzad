% RX over USRP: QPSK with repeated-preamble detection [a,a]
%   + FFT-based coarse CFO estimate (symbol-rate, from known preamble)
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + PLL (decision-directed) for fine carrier/phase tracking
%   + decision-directed Es/N0 estimate
%   + Convolutional FEC decode (rate 1/2, K=3, truncated)
%   + Message decode using protocol.Datagram
%
%   payloadSyms = 270 (unchanged)

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;
MasterClockRate = 100e6;
Fs              = 1e6;
Decim           = MasterClockRate/Fs;  %#ok<NASGU>

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleHalfLen = 64;
preambleLen     = 2 * preambleHalfLen;
payloadSyms     = 512;        % *** unchanged ***
frameSyms       = preambleLen + payloadSyms;
rxGain_dB       = 0;

SamplesPerFrame = 6420;

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

%% ---------- FEC decoder ----------
dec = fec.ViterbiDecoder.rateHalf_K3();
K   = 3;
Kminus1 = K - 1;

%% ---------- Payload + FEC + protocol structure ----------
infoBitsLen  = payloadSyms * bps;   % 540 bits
pilotBitsLen = 100;                 % must match TX

msgCapBytes    = 40;                                   % total datagram bytes
hdrBytes       = double(protocol.Datagram.HEADER_BYTES); % 8
maxProtoPayload = msgCapBytes - hdrBytes;              % 19 bytes

databitsLen   = 8 * msgCapBytes;   % 216 datagram bits
L_in          = databitsLen + Kminus1;   % 218
codedBitsLen  = 2 * L_in;               % 436
bitsAfterPilot = infoBitsLen - pilotBitsLen;  % 440
padBitsLen    = bitsAfterPilot - codedBitsLen; % 4

if padBitsLen < 0
    error('RX: FEC layout invalid (padBitsLen < 0).');
end

rng(1001);
pilotBits = randi([0 1], pilotBitsLen, 1);

fprintf('RX protocol+FEC:\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  L_in (into enc) : %d bits\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

%% ---------- Preamble (known at RX, must match TX) ----------
mseqGen = training.MSequenceGenerator('Degree', 9);
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];
Lpre        = numel(preSyms);

% Hard demapper
qamDemBits = @(z) demQPSK.demodulateHard(z);

% Symbol rate
Rsym = Fs / sps;

%% ---------- DSP chain (detection path) ----------
rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    1000, ...
    'MaximumGain_dB',     30, ...
    'AdaptationStepSize', 1e-3, ...
    'TargetPower',        1.0);

dcblock = filters.DcBlocker('Length',64);

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

%% ---------- Repeated preamble detector ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  0.2, ...
    'MinWindowPower',   1e-7);

%% ---------- FFT-based coarse CFO estimator ----------
fftCfoEst = sync.FftCfoEstimator( ...
    'SampleRateSym', Rsym, ...
    'PreambleSyms',  preSyms, ...
    'Nfft',          4096, ...
    'UseHistory',    false, ...
    'NumCandidates', 3);

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
msgWriter = io.ConsoleWriter();

constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-quadrant-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);
yDetBuf = complex([]);

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

    %% ---------- Detection path ----------
    yDet = rrcDet.process(xAGC);

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

        ySymDet = yDetBuf(1+off : sps : end);
        NsymDet = numel(ySymDet);

        if preStartSym + Lpre - 1 > NsymDet
            break;
        end

        payStartS = preStartSym + preambleLen;
        if payStartS + payloadSyms - 1 > NsymDet
            break;
        end

        %% ---------- CFO estimate ----------
        [wSym_meas, fCfoHz_meas, ~] = fftCfoEst.estimate(ySymDet, preStartSym); %#ok<NASGU>
        if cfoInitialized
            fCfoHz_use = fCfoHz_trk;
        else
            fCfoHz_use = fCfoHz_meas;
        end
        wSym_use = 2*pi * fCfoHz_use / Rsym;

        nSym     = (0:NsymDet-1).';
        ySym_cfo = ySymDet .* exp(-1j * wSym_use .* nSym);

        preEndS = preStartSym + preambleLen - 1;
        candPre = ySym_cfo(preStartSym:preEndS);
        c       = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);
        if c < 0.7
            fprintf('Low corr with real preamble (c=%.2f), waiting for more samples.\n', c);
            break;
        end

        %% ---------- extract payload symbols ----------
        payEndS = payStartS + payloadSyms - 1;
        if payEndS > numel(ySym_cfo)
            break;
        end
        rxSyms_raw = ySym_cfo(payStartS:payEndS);

        %% ---------- carrier/phase recovery ----------
        rxSyms_eq = carSyncNow.process(rxSyms_raw);

        G    = [1, -1, 1j, -1j];
        errs = zeros(1,4);
        for g = 1:4
            rb = qamDemBits(rxSyms_eq * G(g));
            Kc = min(numel(rb), pilotBitsLen);
            errs(g) = mean(rb(1:Kc) ~= pilotBits(1:Kc));
        end
        [~, ig] = min(errs);
        rxSyms = rxSyms_eq * G(ig);

        constDiag(rxSyms);

        %% ---------- Es/N0 estimate ----------
        hb2 = qamDemBits(rxSyms);
        zh2 = modQPSK.modulate(hb2);
        cHd = (zh2' * rxSyms) / (zh2' * zh2 + eps);
        err = rxSyms - cHd * zh2;
        Es  = mean(abs(cHd * zh2).^2);
        Nv  = mean(abs(err).^2);
        SNRdB = 10*log10(max(Es/Nv, eps));

        frames = frames + 1;

        %% ---------- bits -> FEC decode -> datagram bytes ----------
        rxBits = qamDemBits(rxSyms);
        if numel(rxBits) < infoBitsLen
            fprintf('Frame %d: not enough bits (%d < %d)\n', ...
                frames, numel(rxBits), infoBitsLen);
            break;
        end

        % [pilot | codedBits(436) | padBits(4)]
        codedBits = rxBits(pilotBitsLen+1 : pilotBitsLen+codedBitsLen);
        if numel(codedBits) < codedBitsLen
            fprintf('Frame %d: not enough coded bits (%d < %d)\n', ...
                frames, numel(codedBits), codedBitsLen);
            break;
        end

        uBits_hat = dec.decode(logical(codedBits));
        uBits_hat = double(uBits_hat(:));

        if numel(uBits_hat) < databitsLen
            warning('Frame %d: decoded bits %d < required data bits %d', ...
                frames, numel(uBits_hat), databitsLen);
            break;
        end

        % For truncated decoding with dummy bits at the start,
        % the 216 output bits correspond to the original datagram bits.
        dataBits = uBits_hat(1:databitsLen);

        dataBitsMatrix = reshape(dataBits, 8, []).';
        dataBytes      = uint8(bi2de(dataBitsMatrix, 'left-msb'));  % 27 bytes

        %% ---------- parse protocol datagram ----------
        [pkt, ok] = protocol.Datagram.fromBytes(dataBytes);

        if ~ok
            warning('Frame %d: datagram checksum FAILED (seq=%d). Dropping payload.', ...
                frames, pkt.SeqNum);
            pkt.debugPrint();
        else
            msgBytes = pkt.Payload(1 : pkt.PayloadLen);
            msgWriter.write(msgBytes);
        end

        payBits = dataBits;
        paySink.writeFrame(payBits, struct('FrameIndex', frames));

        fprintf(['Summary: off=%d | M=%.3f | Pow=%.3g | ' ...
                 'Es/N0≈%.1f dB | CFO_used≈%.1f Hz (%.3g rad/sym)\n'], ...
                off, detRes.Metric, detRes.WindowPower, ...
                SNRdB, fCfoHz_use, wSym_use);

        %% ---------- CFO tracking update ----------
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

        %% ---------- tighten loops & freeze AGC ----------
        if ~useFine && frames >= 15
            agc.AdaptationStepSize = 1e-9;
            carSyncNow = carSyncFine;
            useFine    = true;
        end

        %% ---------- drop consumed samples ----------
        lastSymIdx   = payStartS + payloadSyms - 1;
        lastSampleIx = 1 + off + (lastSymIdx-1)*sps;
        end_consumed = min(lastSampleIx, numel(xBuf));

        xBuf(1:end_consumed)    = [];
        yDetBuf(1:end_consumed) = [];
    end
end
