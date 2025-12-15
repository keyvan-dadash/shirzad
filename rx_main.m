% RX over USRP: generic M-QAM / M-PSK link with:
%   + Super-coarse CFO via FFT at sample-rate
%   + Schmidl & Cox detector (repeated preamble [a,a])
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + decision-directed PLL (CPPDecisionDirectedCarrierSync)
%   + decision-directed Es/N0 estimate
%   + Convolutional FEC decode (rate 1/2, K=3, terminated)
%   + Datagram decode handled by C++ payload worker via coded bits
%
% Frame: [Preamble | Payload] where Payload is protocol.Payload.

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- Load shared config ----------
cfg = phyAppConfig();

%% ---------- User/link params ----------
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

%% ---------- Mod / Demod ----------
modu = modulators.getMmodulator(cfg.Modulation.Name);
dem  = demodulators.getDemodulator(cfg.Modulation.Name);
qamDemBits = @(z) dem.demodulateHard(z);

M   = modu.M;
bps = modu.BitsPerSymbol;

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();   % only for layout consistency
dec = fec.ViterbiDecoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

% FEC encode: used only to set up Payload layout (same as TX)
fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
fecDecodeFcn  = @(codedBits) double(fec.viterbi_k3_mex(logical(codedBits)));

%% ---------- Build Frame / Payload / Preamble ----------
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

preambleLen = fr.NumPreambleSymbols;   % = 2 * preambleHalfLen
frameSyms   = fr.NumFrameSymbols;

% Consistency check
assert(preambleLen == 2*preambleHalfLen, 'RX: preambleLen mismatch.');

% Sizes for convenience / logging
infoBitsLen  = fr.Payload.InfoBitsLen;
databitsLen  = fr.Payload.DataBitsLen;
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;
pilotBits    = fr.Payload.PilotBits;
pilotBitsLen = numel(pilotBits);

hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2*L_in, 'RX: codedBitsLen mismatch TX.');

fprintf('RX protocol+FEC:\n');
fprintf('  Modulation      : %s (M=%d, bps=%.1f)\n', ...
    cfg.Modulation.Name, M, bps);
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  L_in (into enc) : %d bits\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

%% ---------- Preamble symbols ----------
preSyms = fr.Preamble.Symbols;
Lpre    = fr.NumPreambleSymbols;

% Symbol rate
Rsym = Fs / sps;

%% ---------- DSP chain (detection path) ----------
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

% carSyncCoarse = sync.DecisionDirectedCarrierSync( ...
%     'ModulationOrder',        M, ...
%     'SamplesPerSymbol',       1, ...
%     'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
%     'NormalizedLoopBandwidth',cfg.CarrierSync.CoarseLoopBandwidthNorm);
% 
% carSyncFine = sync.DecisionDirectedCarrierSync( ...
%     'ModulationOrder',        M, ...
%     'SamplesPerSymbol',       1, ...
%     'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
%     'NormalizedLoopBandwidth',cfg.CarrierSync.FineLoopBandwidthNorm);

carSyncNow = carSyncCoarse;
useFine    = false;

%% ---------- Repeated preamble detector (C++ multi-candidate) ----------
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

% Track residual CFO from S&C per frame (rad/sym) to warn on big jumps
lastCfoRadPerSymDet = NaN;
cfoWarnThreshRad    = 0.2;  % rad/sym, tweak if needed

%% ---------- C++ payload sink ----------
paySink = sinks.CppPayloadCollectorSink('NumThreads', 4);

% Register workers from config (mirrors previous PayloadCollectorSink logic)
for kW = 1:numel(cfg.Rx.StreamWriters)
    spec = cfg.Rx.StreamWriters(kW);

    closeOnEnd = true;
    if isfield(spec, 'CloseOnEnd')
        closeOnEnd = logical(spec.CloseOnEnd);
    end

    extra = {};
    if isfield(spec, 'WorkerType')
        % If you configured explicit worker types in cfg, honour them
        extra = {'WorkerType', spec.WorkerType};
    end

    % writer object is only used to infer workerType (console/file)
    paySink.registerWriter(spec.StreamId, spec.Writer, ...
                           'CloseOnEnd', closeOnEnd, extra{:});
end

%% ---------- Source ----------
rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        cfg.SDR.RxIPAddress, ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame, ...
      'TransportDataType', 'int8', ...
      'OutputDataType', 'single');

%% ---------- (Optional) FFT-based CFO estimator (symbol-rate) ----------
% Currently unused, but kept for future experiments.
% fftCfoEst = sync.FftCfoEstimator( ... %#ok<NASGU>
%     'SampleRateSym', Rsym, ...
%     'PreambleSyms',  preSyms, ...
%     'Nfft',          8192, ...
%     'NumCandidates', 3);

constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-phase-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);   % post-AGC samples
yDetBuf = complex([]);   % post-RRC samples

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 100*frameSam + 8*sps + span*sps;

sa = spectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

coarseBuff = [];
buffLen    = cfg.Cfo.SuperCoarseBuffLen;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

prof = utils.EventProfiler();

snrPrintEvery = 50;                 % print every N accepted frames
snrCount      = 0;

snr_dB_hist   = [];                 % store per frame (same as EsN0 here)
esn0_dB_hist  = [];
ebn0_dB_hist  = [];

Rcode = cfg.Fec.Rate;               % coding rate

%% ---------- Main RX loop ----------
while true
    %% ---------- Pull chunk from source ----------
    % prof.start('readFrame');
    [xRaw, len, over] = rfSrc.readFrame();
    % prof.stop('readFrame');

    if over
        fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
            numel(xRaw), SamplesPerFrame);
        continue;
    end

    % continue;

    sa(xRaw);
    % continue;
    % fprintf('the size of buff is: %d\n', numel(xRaw));

    %% ---------- Super-coarse CFO (sample-rate FFT) ----------
    if isSuperCoarseReady && superCoarseFreq ~= 0
        N = numel(xRaw);
        n = (0:N-1).' + sampleIndex;   % global sample index
        xRaw = xRaw .* exp(-1j * 2*pi*superCoarseFreq/Fs .* n);
        sampleIndex = sampleIndex + N;
    end

     % sa(xRaw);

    if ~isSuperCoarseReady
        coarseBuff = [coarseBuff; xRaw];

        if numel(coarseBuff) > buffLen
            XCfo = coarseBuff(1:buffLen);

            w          = hann(buffLen);
            fft_result = fftshift(fft(XCfo));

            fft_2 = abs(fft_result);
            [sort_fft, z] = sort(fft_2,'descend');

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
    % prof.start('dcblock');
    xDC = dcblock.process(xRaw);
    % prof.stop('dcblock');
    % xDC = xRaw;

    if ~useFine
        % prof.start('agc');
        xAGC = agc.process(xDC);
        % prof.stop('agc');
    else
        xAGC = xDC;
    end

    % xAGC = xDC;

    %% ---------- Detection path: RRC ----------
    prof.start('rrc');
    yDet = rrcDet.process(xAGC);
    prof.stop('rrc');

    xBuf    = [xBuf;    xAGC];
    yDetBuf = [yDetBuf; yDet];

    if numel(xBuf) > maxHoldSam
        extra = numel(xBuf) - maxHoldSam;
        xBuf(1:extra)    = [];
        yDetBuf(1:extra) = [];
        fprintf('maxHoldSam chop: dropped %d old samples\n', extra);
    end

    %% ---------- Process all complete frames currently in yDetBuf ----------
    while true
        % Need at least enough samples to ever contain a full frame
        if numel(yDetBuf) < frameSam
            break;
        end

        % prof.start("batch");

        % Full-buffer multi-candidate Schmidl & Cox (C++), no cache
        % prof.start('preambleDetect');
        candList = preDet.detectCandidates(yDetBuf);
        % prof.stop('preambleDetect');

        if isempty(candList)
            fprintf('sfsfsd');
            % No possible preamble anywhere in the buffer.
            % Safe slide: keep only last Lpre symbols worth of samples.
            keepSam = Lpre * sps;
            if numel(yDetBuf) <= keepSam
                break;  % wait for more samples
            end

            dropSamples = numel(yDetBuf) - keepSam;
            dropSamples = min(dropSamples, numel(xBuf));

            xBuf(1:dropSamples)    = [];
            yDetBuf(1:dropSamples) = [];

            continue;   % try again with new buffer head
        end

        % candList is already sorted by StartSample in the detector

        %% ---------- Precompute symbol-rate streams per offset ----------
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

        %% ---------- Choose global CFO for this chunk ----------
        wSym_sc_all = [candList.CfoRadPerSym];
        met_all     = [candList.Metric];
        pow_all     = [candList.WindowPower];
        
        % disp(wSym_sc_all * Rsym / (2*pi));

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

        fprintf('freq is %.3f\n', fCfoHz_use);
        wSym_use = 2*pi * fCfoHz_use / Rsym;

        %% ---------- CFO correction once per offset (vectorized) ----------
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

        %% ---------- Vectorized frame-completeness checks ----------
        preStartSym = [candList.PreambleStartSym];      % 1-based
        offs        = [candList.SampleOffset];

        payStartS = preStartSym + preambleLen;
        payEndS   = payStartS  + payloadSyms - 1;
        preEndS   = preStartSym + preambleLen - 1;

        % Map NsymDet per candidate via offset
        NsymPerCand = NsymDetVec(offs + 1);

        hasFullPre    = (preEndS  <= NsymPerCand);
        hasFullPay    = (payEndS  <= NsymPerCand);
        hasFullFrame  = hasFullPre & hasFullPay;

        % last sample index for frame end (vectorized)
        lastSymIdx   = payEndS;   % symbol index of last payload symbol
        lastSampleIx = 1 + offs + (lastSymIdx - 1) * sps;

        % Also ensure sample-level consistency
        numSam = numel(yDetBuf);
        hasFullFrame = hasFullFrame & (lastSampleIx <= numSam);

        candIdxValid = find(hasFullFrame);
        if isempty(candIdxValid)
            % We have candidates but none have full payload yet; wait for more.
            % fprintf('shooooot %d\n', numel(yDetBuf));
            break;
        end

        %% ---------- Per-frame preamble validation & frame selection ----------
        acceptedIdx        = [];
        acceptedLastSample = [];
        acceptedOffsets    = [];
        acceptedPayStart   = [];
        acceptedPayEnd     = [];
        acceptedWSym_sc    = [];
        acceptedMetric     = [];
        acceptedTheta      = [];
        acceptedSNRdB  = [];
        acceptedEsN0dB = [];
        acceptedEbN0dB = [];

        for jj = 1:numel(candIdxValid)
            ic   = candIdxValid(jj);    % index into candList
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
                % Should not happen if hasFullFrame was true, but guard anyway
                continue;
            end

            % prof.start('frameProcess');

            % Validate preamble via correlation with known preamble
            candPre = ySym_cfo(preStart:preEndS_i);
            h = preSyms' * candPre;

            cCorr   = abs(h) / (norm(candPre)*norm(preSyms) + eps);

            if cCorr < 0.7
                % false alarm: skip this candidate but do NOT drop samples
                % prof.stop('frameProcess');
                % fprintf(['Low corr with real preamble (c=%.2f), ' ...
                %          'skipping candidate at StartSample=%d.\n'], ...
                %         cCorr, cand.StartSample);
                continue;
            end
%% Calculate SNR
            x = preSyms(:);        % known reference (Nx1)
            y = candPre(:);        % received preamble (Nx1)
            
            % LS complex gain estimate: y ≈ a*x + n
            a_hat = (x' * y) / (x' * x + eps);
            
            % Error (noise+impairments not explained by a_hat*x)
            e = y - a_hat * x;
            
            Ps = mean(abs(a_hat*x).^2);         % estimated signal power
            Pn = mean(abs(e).^2) + eps;         % estimated noise/error power
            
            esn0_lin = Ps / Pn;
            esn0_dB  = 10*log10(esn0_lin);
            
            % At 1 sample/symbol after matched filter, SNR ≈ Es/N0
            snr_dB = esn0_dB;
            
            % Eb/N0 (information-bit Eb) for coded system:
            %   Eb = Es / (bps * Rcode)  => Eb/N0 = Es/N0 - 10log10(bps*Rcode)
            ebn0_dB = esn0_dB - 10*log10(bps * Rcode);

            % If we get here, we accept this frame for PLL/decoding
            acceptedIdx(end+1)        = ic;              %#ok<AGROW>
            acceptedLastSample(end+1) = lastSampleIx_i;  %#ok<AGROW>
            acceptedOffsets(end+1)    = off;             %#ok<AGROW>
            acceptedPayStart(end+1)   = payStartS_i;     %#ok<AGROW>
            acceptedPayEnd(end+1)     = payEndS_i;       %#ok<AGROW>
            acceptedWSym_sc(end+1)    = cand.CfoRadPerSym; %#ok<AGROW>
            acceptedMetric(end+1)     = cand.Metric;     %#ok<AGROW>
            acceptedTheta(end+1)      = angle(h);
            acceptedSNRdB(end+1)  = snr_dB;
            acceptedEsN0dB(end+1) = esn0_dB;
            acceptedEbN0dB(end+1) = ebn0_dB;

            % prof.stop('frameProcess');
        end

        if isempty(acceptedIdx)
            % No accepted frames this pass (all false alarms or partials)
            % fprintf('shoooooooooot2\n');
            break;
        end

        %% ---------- Concatenate payload symbols for PLL (vectorized) ----------
        nAcc = numel(acceptedIdx);
        bigPayRaw    = complex([]);
        segStartIdx  = zeros(nAcc,1);
        segLen       = zeros(nAcc,1);

        for k = 1:nAcc
            off   = acceptedOffsets(k);
            ySymC = ySymCfoCell{off+1};
            s0    = acceptedPayStart(k);
            s1    = acceptedPayEnd(k);
            theta = acceptedTheta(k);

            % seg   = ySymC(s0:s1);
            seg = ySymC(s0:s1) .* exp(-1j*theta);
            segStartIdx(k) = numel(bigPayRaw) + 1;
            segLen(k)      = numel(seg);
            bigPayRaw      = [bigPayRaw; seg]; %#ok<AGROW>
        end

        %% ---------- Run PLL once over all accepted payload symbols ----------
        % prof.start('pll');
        bigPayEq = carSyncNow.process(bigPayRaw);
        % prof.stop('pll');

        %% ---------- Per-frame post-PLL processing ----------
        maxDropSamples = 0;

        for k = 1:nAcc
            ic = acceptedIdx(k);

            % Extract equalized payload segment for this frame
            idx0 = segStartIdx(k);
            idx1 = idx0 + segLen(k) - 1;
            rxSyms_eq = bigPayEq(idx0:idx1);
            %

            %% ---------- frame-level processing ----------
            % prof.start('frameProcess2');

            % Generic phase ambiguity resolver using pilot bits
            % prof.start('phaseAmbig');
            [rxSyms, rotIdx, rotErrs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<NASGU>
            % prof.stop('phaseAmbig');

            constDiag(rxSyms);

            frames = frames + 1;

            snrCount = snrCount + 1; 

            snr_dB_hist(snrCount)  = acceptedSNRdB(k);
            esn0_dB_hist(snrCount) = acceptedEsN0dB(k);
            ebn0_dB_hist(snrCount) = acceptedEbN0dB(k);
            
            if mod(snrCount, snrPrintEvery) == 0
                fprintf('SNR≈%.2f dB | Es/N0≈%.2f dB | Eb/N0≈%.2f dB (avg over last %d)\n', ...
                    mean(snr_dB_hist(end-snrPrintEvery+1:end)), ...
                    mean(esn0_dB_hist(end-snrPrintEvery+1:end)), ...
                    mean(ebn0_dB_hist(end-snrPrintEvery+1:end)), ...
                    snrPrintEvery);
            end

            %% ---------- Payload decode: symbols -> coded bits ----------
            % t0 = tic;
            % prof.start('payloadDecode');
            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>
            % prof.stop('payloadDecode');
            % fprintf('time is: %.4f\n', toc(t0));
            % codedBits: logical/double 0/1, length = codedBitsLen

            %% ---------- Deliver coded bits to C++ payload sink ----------
            % prof.start('sink');
            paySink.writeFrame(codedBits, struct('FrameIndex', frames));
            % prof.stop('sink');

            %% ---------- CFO tracking & residual CFO warning ----------
            wSym_sc_frame = acceptedWSym_sc(k);
            fCfoHz_meas   = wSym_sc_frame * Rsym / (2*pi);

            % Warn if residual CFO per symbol jumps a lot frame-to-frame
            if ~isnan(lastCfoRadPerSymDet)
                % wrap difference into [-pi, pi]
                delta = angle(exp(1j*(wSym_sc_frame - lastCfoRadPerSymDet)));
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

            %% ---------- tighten loops & freeze AGC ----------
            if ~useFine && frames >= cfg.CarrierSync.SwitchToFineAfterFrames
                agc.AdaptationStepSize = 1e-9;
                carSyncNow = carSyncFine;
                useFine    = true;
                carSyncFine.reset(carSyncCoarse.phase, carSyncCoarse.freq);
                fprintf('changing to fine\n');
            end

            % prof.stop('frameProcess2');

            %% ---------- track max consumed samples up to last processed frame ----------
            lastSamThis = acceptedLastSample(k);
            if lastSamThis > maxDropSamples
                maxDropSamples = lastSamThis;
            end
        end % per accepted frame
        % prof.stop("batch");

        if maxDropSamples == 0
            % Shouldn't normally happen here, but guard anyway
            fprintf('shot4');
            break;
        end

        %% ---------- drop consumed samples up to last processed frame ----------
        dropSamples = min(maxDropSamples, numel(yDetBuf));
        dropSamples = min(dropSamples, numel(xBuf));

        xBuf(1:dropSamples)    = [];
        yDetBuf(1:dropSamples) = [];

        % prof.stop("batch");

        %% ---------- periodic profiler print ----------
        % if frames >= old_frames + 100
        %     fprintf(['\n=== EventProfiler summary after %d frames and current ' ...
        %         ' batch of %d ===\n'], frames, nAcc);
        %     prof.print();
        %     prof.reset();
        %     fprintf('\n');
        %     old_frames = frames;
        % end
    end
end
