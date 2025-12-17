clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ===================== Load capture =====================
% Point this to your saved capture MAT (created by rx_capture_offline)
capMatPath = 'rx_capture_20251217_154331.mat';   % <-- CHANGE ME

S = load(capMatPath);

% Required fields from capture script
assert(isfield(S,'binFile') && isfield(S,'chunkLen') && isfield(S,'totalChunks'), ...
    'Capture MAT missing required fields (binFile/chunkLen/totalChunks).');

binFile = S.binFile;
fid = fopen(binFile, 'rb');
assert(fid > 0, 'Could not open BIN file: %s', binFile);

cleanupObj = onCleanup(@() fclose(fid));

% Use captured metadata when present
if isfield(S,'cfg')
    cfg = S.cfg;
else
    cfg = phyAppConfig();  % fallback
end

if isfield(S,'Fs'); Fs = double(S.Fs); else; Fs = double(cfg.Link.MasterClockRate/cfg.Link.Decim); end
if isfield(S,'fc'); fc = double(S.fc); else; fc = double(cfg.Link.fcRx); end
if isfield(S,'SamplesPerFrame'); SamplesPerFrame = double(S.SamplesPerFrame); else; SamplesPerFrame = double(cfg.Link.SamplesPerFrame); end

fprintf('OFFLINE RX: using capture\n');
fprintf('  MAT: %s\n', capMatPath);
fprintf('  BIN: %s\n', binFile);
fprintf('  Fs=%.3f Hz | fc=%.3f Hz | SamplesPerFrame=%d\n', Fs, fc, SamplesPerFrame);
fprintf('  chunks=%d\n', double(S.totalChunks));

%% ===================== RX params (unchanged) =====================
MasterClockRate = cfg.Link.MasterClockRate; %#ok<NASGU>
Decim           = cfg.Link.Decim;           %#ok<NASGU>
rxGain_dB       = cfg.Link.RxGain_dB;       %#ok<NASGU>

sps  = cfg.Link.Sps;
beta = cfg.Link.RrcBeta;
span = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;

modu = modulators.getMmodulator(cfg.Modulation.Name);
dem  = demodulators.getDemodulator(cfg.Modulation.Name);
qamDemBits = @(z) dem.demodulateHard(z); %#ok<NASGU>

M   = modu.M;
bps = modu.BitsPerSymbol;

enc = fec.ConvEncoder.rateHalf_K3();
dec = fec.ViterbiDecoder.rateHalf_K3(); %#ok<NASGU>
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
fecDecodeFcn  = @(codedBits) double(fec.viterbi_k3_mex(logical(codedBits)));

msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen;

pre = protocol.Preamble.fromMSequence( ...
    modu, ...
    preambleHalfLen, ...
    'Degree', cfg.Frame.MseqDegree, ...
    'Seed',   cfg.Frame.MseqSeed);

pay = protocol.Payload(modu, dem, ...
              payloadSyms, msgCapBytes, pilotBitsLen, ...
              fecEncodeFcn, fecDecodeFcn);

fr = protocol.Frame(pre, pay);

preambleLen = fr.NumPreambleSymbols;
frameSyms   = fr.NumFrameSymbols;

assert(preambleLen == 2*preambleHalfLen, 'RX: preambleLen mismatch.');

infoBitsLen  = fr.Payload.InfoBitsLen;  %#ok<NASGU>
databitsLen  = fr.Payload.DataBitsLen;
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;
pilotBits    = fr.Payload.PilotBits;
pilotBitsLen = numel(pilotBits);

hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes; %#ok<NASGU>

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2*L_in, 'RX: codedBitsLen mismatch TX.');

fprintf('RX protocol+FEC:\n');
fprintf('  Modulation      : %s (M=%d, bps=%.1f)\n', cfg.Modulation.Name, M, bps);
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  L_in (into enc) : %d bits\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

preSyms = fr.Preamble.Symbols;
Lpre    = fr.NumPreambleSymbols;

Rsym = Fs / sps;

%% ===================== DSP chain (unchanged) =====================
rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    cfg.Agc.AveragingLength, ...
    'MaximumGain_dB',     cfg.Agc.MaximumGain_dB, ...
    'AdaptationStepSize', cfg.Agc.AdaptationStepSize, ...
    'TargetPower',        cfg.Agc.TargetPower);

dcblock = filters.FastDcBlocker('Length', 8192);

carSyncCoarse = sync.CPPDecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
    'NormalizedLoopBandwidth',cfg.CarrierSync.CoarseLoopBandwidthNorm);

carSyncFine = sync.CPPDecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
    'NormalizedLoopBandwidth',cfg.CarrierSync.FineLoopBandwidthNorm);

carSyncNow = carSyncCoarse;
useFine    = false;

preDet = sync.CPPCandidateRepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  cfg.PreambleDetector.MetricThreshold, ...
    'MinWindowPower',   cfg.PreambleDetector.MinWindowPower);

cfoInitialized     = false;
fCfoHz_trk         = 0;
cfoAlpha           = cfg.Cfo.TrackAlpha;
cfoMaxJumpHz       = cfg.Cfo.MaxJumpHz;
metricTrustThresh  = cfg.Cfo.MetricTrustThreshold;

lastCfoRadPerSymDet = NaN;
cfoWarnThreshRad    = 0.2;

paySink = sinks.CppPayloadCollectorSink('NumThreads', 4);
for kW = 1:numel(cfg.Rx.StreamWriters)
    spec = cfg.Rx.StreamWriters(kW);

    closeOnEnd = true;
    if isfield(spec, 'CloseOnEnd')
        closeOnEnd = logical(spec.CloseOnEnd);
    end

    extra = {};
    if isfield(spec, 'WorkerType')
        extra = {'WorkerType', spec.WorkerType};
    end

    paySink.registerWriter(spec.StreamId, spec.Writer, ...
                           'CloseOnEnd', closeOnEnd, extra{:});
end

constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-phase-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

disp('OFFLINE RX: decoding captured samples…');

xBuf    = complex([]);
yDetBuf = complex([]);

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 100*frameSam + 8*sps + span*sps;

% (spectrum analyzer optional for offline; keep if you like)
sa = spectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (offline, post DC/AGC)');

coarseBuff = [];
buffLen    = cfg.Cfo.SuperCoarseBuffLen;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

snrPrintEvery = 50;
snrCount      = 0;

snr_dB_hist   = [];
esn0_dB_hist  = [];
ebn0_dB_hist  = [];

Rcode = cfg.Fec.Rate;

%% ===================== Main OFFLINE loop (chunks) =====================
for chunk = 1:double(S.totalChunks)

    N = double(S.chunkLen(chunk));
    if N <= 0
        continue;
    end

    % Read 2xN float32 values: [I; Q]
    iq = fread(fid, [2 N], 'float32');
    if size(iq,2) ~= N
        fprintf('EOF/short read at chunk %d (wanted %d complex samples, got %d)\n', ...
            chunk, N, size(iq,2));
        break;
    end

    xRaw = complex(iq(1,:).', iq(2,:).');     % N x 1

    % No USRP overruns offline
    over = false; %#ok<NASGU>

    sa(xRaw);

    %% ---------- Super-coarse CFO (sample-rate FFT) ----------
    if isSuperCoarseReady && superCoarseFreq ~= 0
        Nn = numel(xRaw);
        n = (0:Nn-1).' + sampleIndex;
        xRaw = xRaw .* exp(-1j * 2*pi*superCoarseFreq/Fs .* n);
        sampleIndex = sampleIndex + Nn;
    end

    if ~isSuperCoarseReady
        coarseBuff = [coarseBuff; xRaw];

        if numel(coarseBuff) > buffLen
            XCfo = coarseBuff(1:buffLen);

            w          = hann(buffLen);
            fft_result = fftshift(fft(XCfo)); %#ok<NASGU>

            [~, peak]  = max(abs(fft_result));
            peak_new   = peak - (buffLen / 2 + 1);

            superCoarseFreq = peak_new * Fs / buffLen;
            fprintf('Super-coarse CFO ~ %.3f Hz (old/new bins %d/%d)\n', ...
                    superCoarseFreq, peak, peak_new);
            isSuperCoarseReady = true;

            coarseBuff = [];
        end
    end

    %% ---------- DC blocker + AGC ----------
    xDC = dcblock.process(xRaw);

    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    %% ---------- Detection path: RRC ----------
    yDet = rrcDet.process(xAGC);

    xBuf    = [xBuf;    xAGC];
    yDetBuf = [yDetBuf; yDet];

    if numel(xBuf) > maxHoldSam
        extra = numel(xBuf) - maxHoldSam;
        xBuf(1:extra)    = [];
        yDetBuf(1:extra) = [];
        fprintf('maxHoldSam chop: dropped %d old samples\n', extra);
    end

    %% ---------- Process frames inside buffer ----------
    while true
        if numel(yDetBuf) < frameSam
            break;
        end

        candList = preDet.detectCandidates(yDetBuf);

        if isempty(candList)
            keepSam = Lpre * sps;
            if numel(yDetBuf) <= keepSam
                break;
            end
            dropSamples = numel(yDetBuf) - keepSam;
            dropSamples = min(dropSamples, numel(xBuf));

            xBuf(1:dropSamples)    = [];
            yDetBuf(1:dropSamples) = [];
            continue;
        end

        offsAll    = [candList.SampleOffset];
        uniqueOffs = unique(offsAll);

        ySymDetCell = cell(1, sps);
        NsymDetVec  = zeros(1, sps);
        nSymCell    = cell(1, sps);

        for uu = 1:numel(uniqueOffs)
            off = uniqueOffs(uu);
            ySym = yDetBuf(1+off : sps : end);
            ySymDetCell{off+1} = ySym;
            NsymDetVec(off+1)  = numel(ySym);
            nSymCell{off+1}    = (0:NsymDetVec(off+1)-1).';
        end

        wSym_sc_all = [candList.CfoRadPerSym];
        met_all     = [candList.Metric];
        pow_all     = [candList.WindowPower];

        if all(pow_all == 0)
            score = met_all;
        else
            p_norm = pow_all / max(pow_all);
            score  = met_all .* p_norm;
        end

        [~, idxBest]     = max(score);
        wSym_sc_best     = wSym_sc_all(idxBest);
        fCfoHz_meas_best = wSym_sc_best * Rsym / (2*pi);

        if cfoInitialized
            fCfoHz_use = fCfoHz_trk;
        else
            fCfoHz_use = fCfoHz_meas_best;
        end

        wSym_use = 2*pi * fCfoHz_use / Rsym;

        ySymCfoCell = cell(1, sps);
        for uu = 1:numel(uniqueOffs)
            off = uniqueOffs(uu);
            ySym = ySymDetCell{off+1};
            if isempty(ySym); continue; end
            nSym = nSymCell{off+1};
            ySymCfoCell{off+1} = ySym .* exp(-1j * wSym_use .* nSym);
        end

        preStartSym = [candList.PreambleStartSym];
        offs        = [candList.SampleOffset];

        payStartS = preStartSym + preambleLen;
        payEndS   = payStartS  + payloadSyms - 1;
        preEndS   = preStartSym + preambleLen - 1;

        NsymPerCand = NsymDetVec(offs + 1);

        hasFullPre    = (preEndS  <= NsymPerCand);
        hasFullPay    = (payEndS  <= NsymPerCand);
        hasFullFrame  = hasFullPre & hasFullPay;

        lastSampleIx = 1 + offs + (payEndS - 1) * sps;

        hasFullFrame = hasFullFrame & (lastSampleIx <= numel(yDetBuf));

        candIdxValid = find(hasFullFrame);
        if isempty(candIdxValid)
            break;
        end

        acceptedIdx        = [];
        acceptedLastSample = [];
        acceptedOffsets    = [];
        acceptedPayStart   = [];
        acceptedPayEnd     = [];
        acceptedWSym_sc    = [];
        acceptedMetric     = [];
        acceptedTheta      = [];
        acceptedSNRdB      = [];
        acceptedEsN0dB     = [];
        acceptedEbN0dB     = [];

        for jj = 1:numel(candIdxValid)
            ic   = candIdxValid(jj);
            cand = candList(ic);

            off         = cand.SampleOffset;
            preStart    = cand.PreambleStartSym;
            payStartS_i = payStartS(ic);
            payEndS_i   = payEndS(ic);
            preEndS_i   = preEndS(ic);
            lastSampleIx_i = lastSampleIx(ic);

            ySym_cfo = ySymCfoCell{off+1};
            NsymDet  = NsymDetVec(off+1);

            if preEndS_i > NsymDet || payEndS_i > NsymDet
                continue;
            end

            candPre = ySym_cfo(preStart:preEndS_i);
            h = preSyms' * candPre;

            cCorr = abs(h) / (norm(candPre)*norm(preSyms) + eps);
            if cCorr < 0.7
                continue;
            end

            % ---------- preamble-based Es/N0 (same as your code) ----------
            x = preSyms(:);
            y = candPre(:);

            a_hat = (x' * y) / (x' * x + eps);
            e     = y - a_hat * x;

            Ps = mean(abs(a_hat*x).^2);
            Pn = mean(abs(e).^2) + eps;

            esn0_lin = Ps / Pn;
            esn0_dB  = 10*log10(esn0_lin);

            snr_dB   = esn0_dB;
            ebn0_dB  = esn0_dB - 10*log10(bps * Rcode);

            acceptedIdx(end+1)        = ic;
            acceptedLastSample(end+1) = lastSampleIx_i;
            acceptedOffsets(end+1)    = off;
            acceptedPayStart(end+1)   = payStartS_i;
            acceptedPayEnd(end+1)     = payEndS_i;
            acceptedWSym_sc(end+1)    = cand.CfoRadPerSym;
            acceptedMetric(end+1)     = cand.Metric;
            acceptedTheta(end+1)      = angle(h);
            acceptedSNRdB(end+1)      = snr_dB;
            acceptedEsN0dB(end+1)     = esn0_dB;
            acceptedEbN0dB(end+1)     = ebn0_dB;
        end

        if isempty(acceptedIdx)
            break;
        end

        nAcc = numel(acceptedIdx);
        bigPayRaw   = complex([]);
        segStartIdx = zeros(nAcc,1);
        segLen      = zeros(nAcc,1);

        for k = 1:nAcc
            off   = acceptedOffsets(k);
            ySymC = ySymCfoCell{off+1};
            s0    = acceptedPayStart(k);
            s1    = acceptedPayEnd(k);
            theta = acceptedTheta(k);

            seg = ySymC(s0:s1) .* exp(-1j*theta);
            segStartIdx(k) = numel(bigPayRaw) + 1;
            segLen(k)      = numel(seg);
            bigPayRaw      = [bigPayRaw; seg]; %#ok<AGROW>
        end

        bigPayEq = carSyncNow.process(bigPayRaw);

        maxDropSamples = 0;

        for k = 1:nAcc
            ic = acceptedIdx(k);

            idx0 = segStartIdx(k);
            idx1 = idx0 + segLen(k) - 1;
            rxSyms_eq = bigPayEq(idx0:idx1);

            [rxSyms, rotIdx, rotErrs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<ASGLU>

            constDiag(rxSyms);

            frames = frames + 1;

            % ---------- print SNR/EsN0/EbN0 ----------
            snrCount = snrCount + 1;
            snr_dB_hist(snrCount)  = acceptedSNRdB(k);
            esn0_dB_hist(snrCount) = acceptedEsN0dB(k);
            ebn0_dB_hist(snrCount) = acceptedEbN0dB(k);

            if snrCount >= snrPrintEvery && mod(snrCount, snrPrintEvery) == 0
                fprintf('SNR≈%.2f dB | Es/N0≈%.2f dB | Eb/N0≈%.2f dB (avg over last %d)\n', ...
                    mean(snr_dB_hist(end-snrPrintEvery+1:end)), ...
                    mean(esn0_dB_hist(end-snrPrintEvery+1:end)), ...
                    mean(ebn0_dB_hist(end-snrPrintEvery+1:end)), ...
                    snrPrintEvery);
            end

            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>
            paySink.writeFrame(codedBits, struct('FrameIndex', frames));

            % ---------- CFO tracking ----------
            wSym_sc_frame = acceptedWSym_sc(k);
            fCfoHz_meas   = wSym_sc_frame * Rsym / (2*pi);

            if ~isnan(lastCfoRadPerSymDet)
                delta = angle(exp(1j*(wSym_sc_frame - lastCfoRadPerSymDet)));
                if abs(delta) > cfoWarnThreshRad
                    fprintf('Warning: large change in S&C CFO between frames: Δw=%.3g rad/sym\n', delta);
                end
            end
            lastCfoRadPerSymDet = wSym_sc_frame;

            if acceptedMetric(k) >= metricTrustThresh
                if ~cfoInitialized
                    fCfoHz_trk     = fCfoHz_meas;
                    cfoInitialized = true;
                else
                    df = fCfoHz_meas - fCfoHz_trk;
                    if abs(df) > cfoMaxJumpHz
                        df = sign(df) * cfoMaxJumpHz;
                    end
                    fCfoHz_trk = fCfoHz_trk + cfoAlpha * df;
                end
            end

            % ---------- tighten loops & freeze AGC ----------
            if ~useFine && frames >= cfg.CarrierSync.SwitchToFineAfterFrames
                agc.AdaptationStepSize = 1e-9;
                carSyncNow = carSyncFine;
                useFine    = true;
                carSyncFine.reset(carSyncCoarse.phase, carSyncCoarse.freq);
                fprintf('changing to fine\n');
            end

            if acceptedLastSample(k) > maxDropSamples
                maxDropSamples = acceptedLastSample(k);
            end
        end

        if maxDropSamples == 0
            break;
        end

        dropSamples = min(maxDropSamples, numel(yDetBuf));
        dropSamples = min(dropSamples, numel(xBuf));

        xBuf(1:dropSamples)    = [];
        yDetBuf(1:dropSamples) = [];
    end
end

fprintf('OFFLINE RX DONE. Total decoded frames=%d\n', frames);
