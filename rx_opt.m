clear; clc;
profile off;
profile clear;

assert(exist('dsp.UDPReceiver','class')==8, 'Install "DSP System Toolbox" for UDP/USRP support.');

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
rxGain_dB       = cfg.Link.RxGain_dB;

SamplesPerFrame = cfg.Link.SamplesPerFrame;

modu = modulators.getMmodulator(cfg.Modulation.Name);
dem  = demodulators.getDemodulator(cfg.Modulation.Name);

M   = modu.M;
bps = modu.BitsPerSymbol;

enc = fec.ConvEncoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
% fecEncodeFcn = @(dataBits) ...
%     fec.puncture78_k3( enc.encode(logical(dataBits), true) );
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

infoBitsLen  = fr.Payload.InfoBitsLen;
databitsLen  = fr.Payload.DataBitsLen;
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;
pilotBits    = fr.Payload.PilotBits;
pilotBitsLen = numel(pilotBits);

hdrBytes        = single(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2*L_in, 'RX: codedBitsLen mismatch TX.');
% assert(codedBitsLen == floor(8/7*L_in), 'RX: codedBitsLen mismatch TX.');

fprintf('RX protocol+FEC:\n');
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

rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

dc3 = dsp.DCBlocker(Algorithm="Subtract mean");

agc = gain.SimpleAgc( ...
    'AveragingLength',    cfg.Agc.AveragingLength, ...
    'MaximumGain_dB',     cfg.Agc.MaximumGain_dB, ...
    'AdaptationStepSize', cfg.Agc.AdaptationStepSize, ...
    'TargetPower',        cfg.Agc.TargetPower);

dcblock = filters.FastDcBlocker('Length', 16538);

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

rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        cfg.SDR.RxIPAddress, ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame, ...
      'TransportDataType', 'int8', ...
      'OutputDataType', 'single');

constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-phase-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

disp('RX: waiting for frames…');

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 500*frameSam + 8*sps + span*sps;

% Preallocated circular buffer for filtered samples
yDetBuf = utils.CircularComplexBuffer(maxHoldSam);

coarseBuff = [];
buffLen    = cfg.Cfo.SuperCoarseBuffLen;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

% for faster cfo
dphi            = 0;                % phase step per sample
cfoPhasorFrame  = [];               % template phasor for one frame
cfoZ0           = 1;                % starting phasor for current chunk
cfoZstepFrame   = 1;                % phase jump per full SamplesPerFrame

profile off;
% profile clear;
% profile on;

while true
    [xRaw, len, over] = rfSrc.readFrame();

    if over
        fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
            numel(xRaw), SamplesPerFrame);
        % profile off;
        continue;
    end

    if isSuperCoarseReady && superCoarseFreq ~= 0
        N = numel(xRaw);
        xRaw = xRaw .* (cfoZ0 * cfoPhasorFrame(1:N));
        cfoZ0 = cfoZ0 * exp(-1j * dphi * N);
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
            fprintf('Super-coarse CFO ~ %.3f Hz (old/new bins %d/%d)\n', ...
                    superCoarseFreq, peak, peak_new);

            dphi = 2*pi*superCoarseFreq/Fs;

            cfoPhasorFrame = exp(-1j * dphi * (0:SamplesPerFrame-1).');
            
            cfoZ0 = 1;
            
            cfoZstepFrame = exp(-1j * dphi * SamplesPerFrame);

            isSuperCoarseReady = true;
            coarseBuff = [];
        end
    end

    xDC = dcblock.process(xRaw);
    % xDC = dc3(xRaw);

    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    yDet = rrcDet.process(xAGC);

    % Append filtered samples into circular buffer (auto-drops oldest if needed)
    yDetBuf.append(yDet);

    while true
        % Need at least one full frame worth of samples
        if yDetBuf.Length < frameSam
            break;
        end

        % Snapshot current buffer contents as contiguous vector
        yDetVec = yDetBuf.toVector();

        candList = preDet.detectCandidates(yDetVec);

        if isempty(candList)
            % Keep only last Lpre*sps samples, drop older ones
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
            cCorr   = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);

            if cCorr < 0.7
                % fprintf('shiiit\n');
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

        % nAcc = numel(acceptedIdx);
        % bigPayRaw    = complex([]);
        % segStartIdx  = zeros(nAcc,1);
        % segLen       = zeros(nAcc,1);
        % 
        % for k = 1:nAcc
        %     off   = acceptedOffsets(k);
        %     ySymC = ySymCfoCell{off+1};
        %     s0    = acceptedPayStart(k);
        %     s1    = acceptedPayEnd(k);
        % 
        %     seg   = ySymC(s0:s1);
        %     segStartIdx(k) = numel(bigPayRaw) + 1;
        %     segLen(k)      = numel(seg);
        %     bigPayRaw      = [bigPayRaw; seg];
        % end

        nAcc = numel(acceptedIdx);
        segLenPerFrame = payloadSyms;          % constant
        segLen         = repmat(segLenPerFrame, nAcc, 1);
        totalLen       = nAcc * segLenPerFrame;
        
        bigPayRaw   = complex(zeros(totalLen,1, 'single'));
        segStartIdx = (0:nAcc-1).' * segLenPerFrame + 1;
        
        for k = 1:nAcc
            off   = acceptedOffsets(k);
            ySymC = ySymCfoCell{off+1};
            s0    = acceptedPayStart(k);
            s1    = s0 + segLenPerFrame - 1;
        
            idx0 = segStartIdx(k);
            idx1 = idx0 + segLenPerFrame - 1;
        
            bigPayRaw(idx0:idx1) = ySymC(s0:s1);
        end

        bigPayEq = carSyncNow.process(bigPayRaw);

        maxDropSamples = 0;

        for k = 1:nAcc
            ic = acceptedIdx(k);

            idx0 = segStartIdx(k);
            idx1 = idx0 + segLen(k) - 1;
            rxSyms_eq = bigPayEq(idx0:idx1);

            [rxSyms, rotIdx, rotErrs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<ASGLU>

            % constDiag(rxSyms);

            frames = frames + 1;

            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>

            paySink.writeFrame(codedBits, struct('FrameIndex', frames));

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

            if ~useFine && frames >= cfg.CarrierSync.SwitchToFineAfterFrames
                agc.AdaptationStepSize = 1e-9;
                carSyncNow = carSyncFine;
                carSyncFine.reset(carSyncCoarse.phase, carSyncCoarse.freq);
                useFine    = true;
            end

            lastSamThis = acceptedLastSample(k);
            if lastSamThis > maxDropSamples
                maxDropSamples = lastSamThis;
            end
        end

        if maxDropSamples == 0
            break;
        end

        dropSamples = min(maxDropSamples, yDetBuf.Length);
        yDetBuf.dropFirst(dropSamples);
    end
end
