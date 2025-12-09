function rx_benchmark_circbuf_no_usrp()
%RX_BENCHMARK_CIRCBUF_NO_USRP
%   Offline RX throughput benchmark using your CircularComplexBuffer-based
%   RX implementation (no USRP, no plots).
%
%   Pipeline:
%     1) Build PHY config / frame / FEC as usual.
%     2) Offline-generate TX frames (same layout as your TX script).
%     3) Feed concatenated TX waveform into this RX chain:
%          - Super-coarse CFO
%          - DC-blocker, AGC, RRC
%          - CPPCandidateRepeatedPreambleDetector
%          - CFO correction
%          - CPPDecisionDirectedCarrierSync (coarse/fine)
%          - FEC + C++ payload sink
%     4) Measure frames/s and PHY / APP data rate.

clear; clc;
profile off;
profile clear;

assert(exist('dsp.UDPReceiver','class')==8, ...
    'Install "DSP System Toolbox" for UDP/USRP support.');

%% ---------- PHY config ----------
cfg = phyAppConfig();

fc              = cfg.Link.fcRx;
MasterClockRate = cfg.Link.MasterClockRate;
Decim           = cfg.Link.Decim;
Fs              = MasterClockRate/Decim;

sps  = cfg.Link.Sps;
beta = cfg.Link.RrcBeta;
span = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;
rxGain_dB       = cfg.Link.RxGain_dB; %#ok<NASGU>

SamplesPerFrame = cfg.Link.SamplesPerFrame;

%% ---------- Mod / FEC / Frame ----------
modu = modulators.getMmodulator(cfg.Modulation.Name);
dem  = demodulators.getDemodulator(cfg.Modulation.Name);

M   = modu.M;
bps = modu.BitsPerSymbol;

enc = fec.ConvEncoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
% fecEncodeFcn = @(dataBits) fec.puncture78_k3(enc.encode(logical(dataBits), true));

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
pilotBitsLen = numel(pilotBits); %#ok<NASGU>

hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2*L_in, 'RX: codedBitsLen mismatch TX.');
% expectedCoded = (8/7) * L_in;  % 7/8 effective rate
% 
% if abs(double(codedBitsLen) - expectedCoded) > 1e-6
%     error('TX: codedBitsLen (%d) != (8/7)*(databitsLen+%d)=%g.', ...
%         codedBitsLen, Kminus1, expectedCoded);
% end

fprintf('RX BENCH (CircularBuffer, no USRP):\n');
fprintf('  Modulation      : %s (M=%d, bps=%.1f)\n', ...
    cfg.Modulation.Name, M, bps);
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  L_in (into enc) : %d bits\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

preSyms = fr.Preamble.Symbols;
Lpre    = fr.NumPreambleSymbols;

Rsym = Fs / sps;

%% =======================================================================
%   OFFLINE TX GENERATION (baseband)
% ========================================================================

scrSeedBits = logical([ ...
        1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ...
    ]);


% Datagram source (same as TX)
dgramSrc = sources.DatagramSource(msgCapBytes, cfg.Tx.StreamSpecs);

% TX RRC filter
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

frameSamUp = frameSyms * sps;   % samples per TX frame

NframesGen   = 5000;   % total TX frames generated offline
framesTarget = 4000;   % frames to decode for RX benchmark

fprintf('\nGenerating %d TX frames offline...\n', NframesGen);

txWaveAll = complex(zeros(frameSamUp * NframesGen, 1));
widx = 1;

% constDiag = comm.ConstellationDiagram( ...
%     'SamplesPerSymbol', 1, ...
%     'Name', 'RX Constellation (post-PLL, post-phase-fix)', ...
%     'XLimits', [-2 2], ...
%     'YLimits', [-2 2]);


for n = 1:NframesGen
    [protoBytesTX, ~] = dgramSrc.readFrame();
    protoBytesTX = scrambler.scrambleBytes(protoBytesTX, scrSeedBits);
    [frmSymsRawTX, ~] = fr.encode(protoBytesTX);

    pilotAmp = cfg.Frame.PilotAmpOffset;
    frmSymsTX = pilotAmp + frmSymsRawTX;

    up = zeros(numel(frmSymsTX)*sps, 1);
    up(1:sps:end) = frmSymsTX;

    txOut = txRRC.process(up);

    txWaveAll(widx : widx + frameSamUp - 1) = txOut;
    widx = widx + frameSamUp;
end

txWaveAll = single(txWaveAll);

totalSamples = numel(txWaveAll);
readOffset   = 0;

fprintf('Offline TX buffer: %d samples (%.3f s @ Fs=%.0f Hz)\n\n', ...
    totalSamples, totalSamples/Fs, Fs);

%% =======================================================================
%   RX CHAIN (your CircularComplexBuffer-based implementation)
% ========================================================================

rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    cfg.Agc.AveragingLength, ...
    'MaximumGain_dB',     cfg.Agc.MaximumGain_dB, ...
    'AdaptationStepSize', cfg.Agc.AdaptationStepSize, ...
    'TargetPower',        cfg.Agc.TargetPower);

dcblock = filters.FastDcBlocker('Length',2048);

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

% C++ payload sink
paySink = sinks.CppPayloadCollectorSink('NumThreads', 8);
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

% (constellation diagram is unused in benchmark)
% constDiag = comm.ConstellationDiagram(...)

disp('RX BENCH: processing offline data (Circular buffer, no USRP)...');

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 100*frameSam + 8*sps + span*sps;

% Circular buffer for filtered samples
yDetBuf = utils.CircularComplexBuffer(maxHoldSam);

coarseBuff = [];
buffLen    = cfg.Cfo.SuperCoarseBuffLen;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

done = false;

% profile on;   % if you want profiling, uncomment

%% ---------- Benchmark timer ----------
% profile clear;
% profile on;
low_corr= 0;
tStart = tic;

%% ---------- Main RX loop (offline source instead of rfSrc) ----------
while ~done
    % Emulate rfSrc.readFrame() from offline buffer
    if readOffset >= totalSamples
        fprintf('RX BENCH: ran out of offline samples at %d.\n', readOffset);
        break;
    end

    Nread = min(SamplesPerFrame, totalSamples - readOffset);
    xRaw  = txWaveAll(readOffset+1 : readOffset+Nread);
    len   = Nread; %#ok<NASGU>
    over  = false; %#ok<NASGU>
    readOffset = readOffset + Nread;

    % --- Super-coarse CFO ---
    if isSuperCoarseReady && superCoarseFreq ~= 0
        N = numel(xRaw);
        n = (0:N-1).' + sampleIndex;
        xRaw = xRaw .* exp(-1j * 2*pi*superCoarseFreq/Fs .* n);
        sampleIndex = sampleIndex + N;
    end

    if ~isSuperCoarseReady
        coarseBuff = [coarseBuff; xRaw];
        if numel(coarseBuff) > buffLen
            XCfo = coarseBuff(1:buffLen);
            w          = hann(buffLen);
            fft_result = fftshift(fft(XCfo .* w));
            [~, peak]  = max(abs(fft_result));
            peak_new   = peak - (buffLen / 2 + 1);
            superCoarseFreq = peak_new * Fs / buffLen;
            fprintf('Super-coarse CFO (offline) ~ %.3f Hz (old/new bins %d/%d)\n', ...
                    superCoarseFreq, peak, peak_new);
            isSuperCoarseReady = true;
            coarseBuff = [];
        end
    end

    % --- DC + AGC + RRC ---
    xDC = dcblock.process(xRaw);
    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end
    yDet = rrcDet.process(xAGC);

    % Append into circular buffer (auto-drops oldest if needed)
    yDetBuf.append(yDet);

    %% ---------- Process all complete frames in yDetBuf ----------
    while true
        if yDetBuf.Length < frameSam
            break;
        end

        % Snapshot current buffer contents
        yDetVec = yDetBuf.toVector();

        candList = preDet.detectCandidates(yDetVec);

        if isempty(candList)
            keepSam = Lpre * sps;
            if yDetBuf.Length <= keepSam
                break;
            end
            dropSamples = yDetBuf.Length - keepSam;
            yDetBuf.dropFirst(dropSamples);
            continue;
        end

        offsAll    = [candList.SampleOffset];
        uniqueOffs = unique(offsAll);

        ySymDetCell = cell(1, sps);
        NsymDetVec  = zeros(1, sps);
        nSymCell    = cell(1, sps);

        for uu = 1:numel(uniqueOffs)
            off = uniqueOffs(uu);
            ySym = yDetVec(1+off : sps : end);
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

        [~, idxBest]   = max(score);
        wSym_sc_best   = wSym_sc_all(idxBest);
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
            if isempty(ySym)
                continue;
            end
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

        numSam = yDetBuf.Length;
        hasFullFrame = hasFullFrame & (lastSampleIx <= numSam);

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
            cCorr   = abs(candPre' * preSyms) / ...
                      (norm(candPre)*norm(preSyms) + eps);

            if cCorr < 0.7
                low_corr = low_corr + 1;
                continue;
            end

            acceptedIdx(end+1)        = ic;
            acceptedLastSample(end+1) = lastSampleIx_i;
            acceptedOffsets(end+1)    = off;
            acceptedPayStart(end+1)   = payStartS_i;
            acceptedPayEnd(end+1)     = payEndS_i;
            acceptedWSym_sc(end+1)    = cand.CfoRadPerSym;
            acceptedMetric(end+1)     = cand.Metric;
        end

        if isempty(acceptedIdx)
            break;
        end

        nAcc = numel(acceptedIdx);
        bigPayRaw    = complex([]);
        segStartIdx  = zeros(nAcc,1);
        segLen       = zeros(nAcc,1);

        for k = 1:nAcc
            off   = acceptedOffsets(k);
            ySymC = ySymCfoCell{off+1};
            s0    = acceptedPayStart(k);
            s1    = acceptedPayEnd(k);

            seg   = ySymC(s0:s1);
            segStartIdx(k) = numel(bigPayRaw) + 1;
            segLen(k)      = numel(seg);
            bigPayRaw      = [bigPayRaw; seg];
        end

        % Here
        % maxDropSamples = max(acceptedLastSample(:));
        % dropSamples = min(maxDropSamples, yDetBuf.Length);
        % yDetBuf.dropFirst(dropSamples);
        % frames = frames + nAcc;
        % continue;


        bigPayEq = carSyncNow.process(bigPayRaw);

        maxDropSamples = 0;

        for k = 1:nAcc
            ic = acceptedIdx(k);

            idx0 = segStartIdx(k);
            idx1 = idx0 + segLen(k) - 1;
            rxSyms_eq = bigPayEq(idx0:idx1);

            [rxSyms, rotIdx, rotErrs] = ...
                dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<NASGU>

            frames = frames + 1;

            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>

            paySink.writeFrame(codedBits, struct('FrameIndex', frames));

            wSym_sc_frame = acceptedWSym_sc(k);
            fCfoHz_meas   = wSym_sc_frame * Rsym / (2*pi);

            if ~isnan(lastCfoRadPerSymDet)
                delta = angle(exp(1j * ...
                    (wSym_sc_frame - lastCfoRadPerSymDet)));
                if abs(delta) > cfoWarnThreshRad
                    fprintf(['Warning: large change in S&C CFO between frames: ' ...
                             'Δw=%.3g rad/sym\n'], delta);
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

            if ~useFine && frames >= cfg.CarrierSync.SwitchToFineAfterFrames
                agc.AdaptationStepSize = 1e-9;
                carSyncNow = carSyncFine;
                carSyncFine.reset(carSyncCoarse.phase, carSyncCoarse.freq);
                useFine    = true;
                fprintf('Switching to fine carrier loop.\n');
            end

            lastSamThis = acceptedLastSample(k);
            if lastSamThis > maxDropSamples
                maxDropSamples = lastSamThis;
            end

            if frames >= framesTarget
                done = true;
            end
        end

        if maxDropSamples == 0
            break;
        end

        dropSamples = min(maxDropSamples, yDetBuf.Length);
        yDetBuf.dropFirst(dropSamples);

        if done
            break;
        end
    end

    if done
        break;
    end
end

elapsed = toc(tStart);

fprintf('skipped things: %d\n', low_corr);
% profile off;
% profile viewer;

%% ---------- Throughput summary ----------
fprintf('\nRX BENCH RESULTS (Circular buffer, no USRP):\n');
fprintf('  Frames decoded  : %d (target=%d)\n', frames, framesTarget);
fprintf('  Elapsed time    : %.3f s\n', elapsed);

if elapsed > 0 && frames > 0
    framesPerSec = frames / elapsed;

    % 1) Over-the-air PHY bits (includes preamble, payload, FEC etc via symbols)
    phyBitsPerFrame_air = frameSyms * bps;
    phyRateAir_kBps = (framesPerSec * phyBitsPerFrame_air) / 8 / 1024;

    % 2) Coded payload bits only (excludes preamble, includes FEC redundancy)
    phyBitsPerFrame_coded = codedBitsLen;   % from fr.Payload.CodedBitsLen
    phyRateCoded_kBps = (framesPerSec * phyBitsPerFrame_coded) / 8 / 1024;

    % 3) Net datagram bytes (header + payload) after FEC/pilot constraints
    netDatagramBytesPerFrame = msgCapBytes;
    netDatagramRate_kBps = (framesPerSec * netDatagramBytesPerFrame) / 1024;

    % 4) Max application payload bytes (datagram payload only)
    appPayloadBytesPerFrame = maxProtoPayload;
    appRate_kBps = (framesPerSec * appPayloadBytesPerFrame) / 1024;

    % 5) Symbols per second (total & payload-only)
    symsPerSec_total   = framesPerSec * frameSyms;
    symsPerSec_payload = framesPerSec * payloadSyms;

    fprintf('Frames per sec    : %.1f frames/s\n', framesPerSec);
    fprintf('Symbols per sec   : %.1f sym/s (total), %.1f sym/s (payload)\n', ...
            symsPerSec_total, symsPerSec_payload);

    fprintf('PHY air rate      : %.1f kB/s (frameSyms=%d, bps=%.1f)\n', ...
            phyRateAir_kBps, frameSyms, bps);
    fprintf('PHY coded rate    : %.1f kB/s (codedBitsLen=%d)\n', ...
            phyRateCoded_kBps, phyBitsPerFrame_coded);
    fprintf('Datagram rate     : %.1f kB/s (MsgCap=%dB)\n', ...
            netDatagramRate_kBps, netDatagramBytesPerFrame);
    fprintf('Max APP rate      : %.1f kB/s (payload<=%dB)\n', ...
            appRate_kBps, appPayloadBytesPerFrame);
else
    fprintf('Not enough frames / time to compute throughput.\n');
end

end
