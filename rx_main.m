% RX over USRP: generic M-QAM / M-PSK link with:
%   + Super-coarse CFO via FFT at sample-rate
%   + Schmidl & Cox detector (repeated preamble [a,a])
%   + CFO correction at SYMBOL RATE (1 sps, post-RRC)
%   + decision-directed PLL (DecisionDirectedCarrierSync)
%   + decision-directed Es/N0 estimate
%   + Convolutional FEC decode (rate 1/2, K=3, terminated)
%   + Datagram decode handled by sinks.PayloadCollectorSink via io.Writer
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
mod = modulators.getMmodulator(cfg.Modulation.Name);
dem = demodulators.getDemodulator(cfg.Modulation.Name);
qamDemBits = @(z) dem.demodulateHard(z);

M   = mod.M;
bps = mod.BitsPerSymbol;

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc = fec.ConvEncoder.rateHalf_K3();   % only for layout consistency
dec = fec.ViterbiDecoder.rateHalf_K3();
K   = cfg.Fec.ConstraintLength;
Kminus1 = K - 1;

% FEC encode: used only to set up Payload layout (same as TX)
fecEncodeFcn = @(dataBits) enc.encode(logical(dataBits), true);

% FEC decode: prefer MEX if available, else MATLAB Viterbi
if cfg.Fec.UseMexK3 && exist('fec.viterbi_k3_mex','file')
    fecDecodeFcn = @(codedBits) double(fec.viterbi_k3_mex(logical(codedBits)));
else
    fecDecodeFcn = @(codedBits) dec.decode(double(codedBits));
end

%% ---------- Build Frame / Payload / Preamble ----------
msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen;

pre = protocol.Preamble.fromMSequence( ...
    mod, ...
    preambleHalfLen, ...
    'Degree', cfg.Frame.MseqDegree, ...
    'Seed',   cfg.Frame.MseqSeed);

pay = protocol.Payload(mod, dem, ...
              payloadSyms, msgCapBytes, pilotBitsLen, ...
              fecEncodeFcn, fecDecodeFcn);

fr = protocol.Frame(pre, pay);

preambleLen = fr.NumPreambleSymbols;
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

dcblock = filters.DcBlocker('Length',2048);

carSyncCoarse = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
    'NormalizedLoopBandwidth',cfg.CarrierSync.CoarseLoopBandwidthNorm);

carSyncFine = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
    'NormalizedLoopBandwidth',cfg.CarrierSync.FineLoopBandwidthNorm);

carSyncNow = carSyncCoarse;
useFine    = false;

%% ---------- Repeated preamble detector ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  cfg.PreambleDetector.MetricThreshold, ...
    'MinWindowPower',   cfg.PreambleDetector.MinWindowPower);

%% ---------- (Optional) FFT-based CFO estimator (symbol-rate) ----------
fftCfoEst = sync.FftCfoEstimator( ... %#ok<NASGU>
    'SampleRateSym', Rsym, ...
    'PreambleSyms',  preSyms, ...
    'Nfft',          8192, ...
    'NumCandidates', 3);  % not currently used

cfoInitialized     = false;
fCfoHz_trk         = 0;
cfoAlpha           = cfg.Cfo.TrackAlpha;
cfoMaxJumpHz       = cfg.Cfo.MaxJumpHz;
metricTrustThresh  = cfg.Cfo.MetricTrustThreshold;

%% ---------- Source & Sinks ----------
rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        cfg.SDR.RxIPAddress, ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

% Datagram sink / demux
paySink = sinks.PayloadCollectorSink();

% Register writers from config
for kW = 1:numel(cfg.Rx.StreamWriters)
    spec = cfg.Rx.StreamWriters(kW);
    closeOnEnd = true;
    if isfield(spec, 'CloseOnEnd')
        closeOnEnd = logical(spec.CloseOnEnd);
    end
    paySink.registerWriter(spec.StreamId, spec.Writer, 'CloseOnEnd', closeOnEnd);
end

constDiag = comm.ConstellationDiagram( ...
    'SamplesPerSymbol', 1, ...
    'Name', 'RX Constellation (post-PLL, post-phase-fix)', ...
    'XLimits', [-2 2], ...
    'YLimits', [-2 2]);

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');
xBuf    = complex([]);
yDetBuf = complex([]);

frames = 0;

frameSam   = frameSyms * sps;
maxHoldSam = 10*frameSam + 8*sps + span*sps;

sa = spectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

coarseBuff = [];
buffLen    = cfg.Cfo.SuperCoarseBuffLen;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

while true
    %% ---------- Pull chunk from source ----------
    tStart = tic; %#ok<NASGU>
    [xRaw, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid
        pause(0.05);
        continue;
    end

    if srcInfo.Overrun
        fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
            numel(xRaw), SamplesPerFrame);
        return;   % for now, bail out on overrun
    end

    % sa(xRaw);

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

    %% ---------- INNER LOOP ----------
    while true
        if numel(xBuf) < frameSam + 8*sps
            break;
        end

        searchSyms   = frameSyms + 10;
        maxDetectSam = searchSyms * sps;
        yDetSearch   = yDetBuf(1 : min(numel(yDetBuf), maxDetectSam));
        detRes       = preDet.detectFast(yDetSearch);

        if ~detRes.Found
            % fprintf('No preamble: M=%.3f, Pow=%.3g\n', ...
            %     detRes.Metric, detRes.WindowPower);

            dropSamples = min((searchSyms - preambleHalfLen) * sps, numel(xBuf));
        
            xBuf(1:dropSamples)    = [];
            yDetBuf(1:dropSamples) = [];
            break;
        end

        off         = detRes.SampleOffset;
        preStartSym = detRes.PreambleStartSym;
        wSym_sc     = detRes.CfoRadPerSym;      % rad/sym from Schmidl
        fCfoHz_meas = wSym_sc * Rsym / (2*pi);

        % Symbol-rate stream from yDet
        ySymDet = yDetBuf(1+off : sps : end);
        NsymDet = numel(ySymDet);

        if preStartSym + Lpre - 1 > NsymDet
            break;
        end

        payStartS = preStartSym + preambleLen;
        if payStartS + payloadSyms - 1 > NsymDet
            break;
        end

        %% ---------- CFO apply at symbol-rate ----------
        if cfoInitialized
            fCfoHz_use = fCfoHz_trk;
        else
            fCfoHz_use = fCfoHz_meas;
        end
        wSym_use = 2*pi * fCfoHz_use / Rsym;

        nSym     = (0:NsymDet-1).';
        ySym_cfo = ySymDet .* exp(-1j * wSym_use .* nSym);

        % Validate preamble via correlation
        preEndS = preStartSym + preambleLen - 1;
        candPre = ySym_cfo(preStartSym:preEndS);
        c       = abs(candPre' * preSyms) / (norm(candPre)*norm(preSyms) + eps);
        if c < 0.7
            fprintf('Low corr with real preamble (c=%.2f), waiting for more samples.\n', c);
            lastFalseSym    = preStartSym + preambleLen + payloadSyms - 1;
            lastFalseSym = min(lastFalseSym, searchSyms - preambleLen);
            lastFalseSample = off + (lastFalseSym - 1) * sps;
            dropSamples     = min(lastFalseSample, numel(xBuf));

            fprintf('total size of buf: %d, and we are gonna drop %d\n', numel(xBuf), dropSamples);
        
            xBuf(1:dropSamples)    = [];
            yDetBuf(1:dropSamples) = [];
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

        % Generic phase ambiguity resolver using pilot bits
        [rxSyms, rotIdx, rotErrs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits);
        % fprintf('the error is: %3f\n', rotErrs);

        % constDiag(rxSyms .* 10);

        %% ---------- Es/N0 estimate ----------
        hb2 = qamDemBits(rxSyms);
        zh2 = mod.modulate(hb2);
        cHd = (zh2' * rxSyms) / (zh2' * zh2 + eps);
        err = rxSyms - cHd * zh2;
        Es  = mean(abs(cHd * zh2).^2);
        Nv  = mean(abs(err).^2);
        SNRdB = 10*log10(max(Es/Nv, eps));

        frames = frames + 1;

        %% ---------- Payload decode: symbols -> datagram bytes ----------
        [dataBytes, payInfo] = fr.decodeFromPayload(rxSyms); %#ok<NASGU>
        % dataBytes: uint8 column, length = msgCapBytes (one datagram)

        %% ---------- Deliver datagram to PayloadCollectorSink ----------
        paySink.writeFrame(dataBytes, struct('FrameIndex', frames));

        % fprintf(['Summary: off=%d | M=%.3f | ' ...
        %          'Es/N0≈%.1f dB | CFO_used≈%.1f Hz (%.3g rad/sym)\n'], ...
        %         off, detRes.Metric, SNRdB, fCfoHz_use, wSym_use);

        %% ---------- CFO tracking update ----------
        if detRes.Metric >= metricTrustThresh
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

        %% ---------- drop consumed samples ----------
        lastSymIdx   = payStartS + payloadSyms - 1;
        lastSampleIx = 1 + off + (lastSymIdx-1)*sps;
        end_consumed = min(lastSampleIx, numel(xBuf));

        xBuf(1:end_consumed)    = [];
        yDetBuf(1:end_consumed) = [];
    end
end
