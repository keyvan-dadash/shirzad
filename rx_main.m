% RX over USRP: generic M-QAM / M-PSK link with:
%   + Super-coarse CFO via FFT at sample-rate
%   + Schmidl & Cox detector (repeated preamble [a,a])
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + decision-directed PLL (DecisionDirectedCarrierSync)
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
dem = demodulators.getDemodulator(cfg.Modulation.Name);
qamDemBits = @(z) dem.demodulateHard(z);

M   = modu.M;
bps = modu.BitsPerSymbol;

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();   % only for layout consistency
dec = fec.ViterbiDecoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

% FEC encode: used only to set up Payload layout (same as TX)
fecEncodeFcn = @(dataBits) enc.encode(logical(dataBits), true);

fecDecodeFcn = @(codedBits) double(fec.viterbi_k3_mex(logical(codedBits)));

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

dcblock = filters.FastDcBlocker('Length',1024);

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
      'SamplesPerFrame',  SamplesPerFrame);

%% ---------- C++ payload worker setup ----------
% Here we assume you manage C++ stream workers separately via
% utils.payload_worker_mex('add_worker', streamId, 'console'/'file') etc.
% This RX just feeds coded bits into the C++ backend.
% If you want, you can also create a CppPayloadCollectorSink wrapper instead.

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
maxHoldSam = 30*frameSam + 8*sps + span*sps;

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

while true
    %% ---------- Pull chunk from source ----------
    prof.start('readFrame');
    [xRaw, srcInfo] = rfSrc.readFrame();
    prof.stop('readFrame');

    % fprintf('new read\n');
    if ~srcInfo.IsValid
        pause(0.05);
        continue;
    end

    if srcInfo.Overrun
        fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
            numel(xRaw), SamplesPerFrame);
        return;
    end

    % sa(xRaw);
    % fprintf('the size of buff is: %d\n', numel(xRaw));

    %% ---------- Super-coarse CFO (sample-rate FFT) ----------
    if isSuperCoarseReady && superCoarseFreq ~= 0
        N = numel(xRaw);
        n = (0:N-1).' + sampleIndex;   % global sample index
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
            fprintf('Super-coarse CFO ~ %.3f Hz (old/new bins %d/%d)\n', ...
                    superCoarseFreq, peak, peak_new);
            isSuperCoarseReady = true;

            coarseBuff = [];
        end
    end

    %% ---------- DC blocker + AGC ----------
    prof.start('dcblock');
    xDC = dcblock.process(xRaw);
    prof.stop('dcblock');

    if ~useFine
        prof.start('agc');
        xAGC = agc.process(xDC);
        prof.stop('agc');
    else
        xAGC = xDC;
    end

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

        % Full-buffer multi-candidate Schmidl & Cox (C++), no cache
        prof.start('preambleDetect');
        candList = preDet.detectCandidates(yDetBuf);
        prof.stop('preambleDetect');

        if isempty(candList)
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
        maxDropSamples = 0;

        for ic = 1:numel(candList)
            cand        = candList(ic);
            off         = cand.SampleOffset;
            preStartSym = cand.PreambleStartSym;

            % Symbol-rate stream from yDet at this offset
            ySymDet = yDetBuf(1+off : sps : end);
            NsymDet = numel(ySymDet);

            % Ensure preamble fully inside symbol buffer
            if preStartSym + Lpre - 1 > NsymDet
                % Not enough symbols yet (preamble crosses end of buffer)
                continue;
            end

            payStartS = preStartSym + preambleLen;
            payEndS   = payStartS + payloadSyms - 1;
            if payEndS > NsymDet
                % Payload not fully in buffer yet for this candidate
                % (and any later one), leave it for next chunk.
                break;
            end

            prof.start('frameProcess');

            %% ---------- CFO from S&C + tracking ----------
            wSym_sc     = cand.CfoRadPerSym;
            fCfoHz_meas = wSym_sc * Rsym / (2*pi);

            % Warn if residual CFO per symbol jumps a lot frame-to-frame
            if ~isnan(lastCfoRadPerSymDet)
                % wrap difference into [-pi, pi]
                delta = angle(exp(1j*(wSym_sc - lastCfoRadPerSymDet)));
                if abs(delta) > cfoWarnThreshRad
                    fprintf('Warning: large change in S&C CFO between frames: Δw=%.3g rad/sym\n', delta);
                end
            end
            lastCfoRadPerSymDet = wSym_sc;

            if cfoInitialized
                fCfoHz_use = fCfoHz_trk;
            else
                fCfoHz_use = fCfoHz_meas;
            end
            wSym_use = 2*pi * fCfoHz_use / Rsym;

            nSym     = (0:NsymDet-1).';
            ySym_cfo = ySymDet .* exp(-1j * wSym_use .* nSym);

            % Validate preamble via correlation with known preamble
            preEndS = preStartSym + preambleLen - 1;
            candPre = ySym_cfo(preStartSym:preEndS);
            cCorr   = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);

            if cCorr < 0.7
                % false alarm: skip this candidate but do NOT drop samples
                prof.stop('frameProcess');

                fprintf('Low corr with real preamble (c=%.2f), skipping candidate at StartSample=%d.\n', ...
                        cCorr, cand.StartSample);
                continue;
            end

            %% ---------- extract payload symbols ----------
            rxSyms_raw = ySym_cfo(payStartS:payEndS);

            %% ---------- carrier/phase recovery ----------
            prof.start('pll');
            rxSyms_eq = carSyncNow.process(rxSyms_raw);
            prof.stop('pll');

            % Generic phase ambiguity resolver using pilot bits
            prof.start('phaseAmbig');
            [rxSyms, rotIdx, rotErrs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<NASGU>
            prof.stop('phaseAmbig');

            % constDiag(rxSyms .* 10);

            frames = frames + 1;

            %% ---------- Payload decode: symbols -> coded bits ----------
            % t0 = tic;
            prof.start('payloadDecode');
            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>
            prof.stop('payloadDecode');
            % fprintf('time is: %.4f\n', toc(t0));
            % codedBits: logical/double 0/1, length = codedBitsLen

            prof.start('sink');
            paySink.writeFrame(codedBits, struct('FrameIndex', frames));
            prof.stop('sink');

            % Optional: log summary if you want
            % fprintf(['Summary: M=%.3f | ' ...
            %          'Es/N0≈%.1f dB | CFO_used≈%.1f Hz (%.3g rad/sym)\n'], ...
            %         cand.Metric, SNRdB, fCfoHz_use, wSym_use);

            %% ---------- CFO tracking update ----------
            if cand.Metric >= metricTrustThresh
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
            end

            % Done with this frame’s processing
            prof.stop('frameProcess');

            %% ---------- track max consumed samples for this batch ----------
            % last symbol (payload end) in symbol index:
            lastSymIdx   = payEndS;
            % sample index in yDetBuf (1-based):
            lastSampleIx = 1 + off + (lastSymIdx-1)*sps;

            if lastSampleIx > maxDropSamples
                maxDropSamples = lastSampleIx;
            end
        end % for each candidate

        if maxDropSamples == 0
            % No complete frame processed in this pass (e.g. only partial one
            % at the end). Wait for more samples.
            break;
        end

        %% ---------- drop consumed samples up to last processed frame ----------
        dropSamples = min(maxDropSamples, numel(yDetBuf));
        dropSamples = min(dropSamples, numel(xBuf));

        xBuf(1:dropSamples)    = [];
        yDetBuf(1:dropSamples) = [];

        %% ---------- periodic profiler print ----------
        % if mod(frames, 100) == 0
        %     fprintf('\n=== EventProfiler summary after %d frames ===\n', frames);
        %     prof.print();
        %     fprintf('\n');
        % end
    end
end
