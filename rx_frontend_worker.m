function rx_frontend_worker(paySymQueue)
    % ==== all your config stuff here ====
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
    M    = modu.M;
    bps  = modu.BitsPerSymbol; %#ok<NASGU>

    enc = fec.ConvEncoder.rateHalf_K3();
    K   = cfg.Fec.ConstraintLength;
    Kminus1 = K - 1; %#ok<NASGU>

    msgCapBytes  = cfg.Frame.MsgCapBytes;
    pilotBitsLen = cfg.Frame.PilotBitsLen;

    pre = protocol.Preamble.fromMSequence( ...
        modu, ...
        preambleHalfLen, ...
        'Degree', cfg.Frame.MseqDegree, ...
        'Seed',   cfg.Frame.MseqSeed);

    dem_dummy = demodulators.getDemodulator(cfg.Modulation.Name); % only for Frame
    fecEncodeFcn   = @(dataBits) enc.encode(logical(dataBits), true);
    fecDecodeDummy = @(codedBits) error('decode not used in front-end');

    pay = protocol.Payload(modu, dem_dummy, ...
                  payloadSyms, msgCapBytes, pilotBitsLen, ...
                  fecEncodeFcn, fecDecodeDummy);

    fr = protocol.Frame(pre, pay);
    preSyms      = fr.Preamble.Symbols;
    Lpre         = fr.NumPreambleSymbols;
    preambleLen  = fr.NumPreambleSymbols;
    frameSyms    = fr.NumFrameSymbols;
    Rsym         = Fs / sps;

    % --- front-end DSP objects ---
    rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);
    agc = gain.SimpleAgc( ...
        'AveragingLength',    cfg.Agc.AveragingLength, ...
        'MaximumGain_dB',     cfg.Agc.MaximumGain_dB, ...
        'AdaptationStepSize', cfg.Agc.AdaptationStepSize, ...
        'TargetPower',        cfg.Agc.TargetPower);
    dcblock = filters.FastDcBlocker('Length',1024);

    preDet = sync.CPPCandidateRepeatedPreambleDetector( ...
        'SamplesPerSymbol', sps, ...
        'PreambleHalfLen',  preambleHalfLen, ...
        'MetricThreshold',  cfg.PreambleDetector.MetricThreshold, ...
        'MinWindowPower',   cfg.PreambleDetector.MinWindowPower);

    % CFO tracking state
    cfoInitialized      = false;
    fCfoHz_trk          = 0;
    cfoAlpha            = cfg.Cfo.TrackAlpha;
    cfoMaxJumpHz        = cfg.Cfo.MaxJumpHz;
    metricTrustThresh   = cfg.Cfo.MetricTrustThreshold;
    lastCfoRadPerSymDet = NaN;
    cfoWarnThreshRad    = 0.2;

    rfSrc = sources.SDRuBasebandSource( ...
          'IPAddress',        cfg.SDR.RxIPAddress, ...
          'CenterFrequency',  fc, ...
          'MasterClockRate',  MasterClockRate, ...
          'DecimationFactor', Decim, ...
          'Gain',             rxGain_dB, ...
          'SamplesPerFrame',  SamplesPerFrame, ...
          'TransportDataType', 'int8', ...
          'OutputDataType',   'single');

    disp('Front-end worker: waiting for frames...');

    frames = 0;
    frameSam   = frameSyms * sps;
    maxHoldSam = 200*frameSam + 8*sps + span*sps;
    yDetBuf    = utils.CircularComplexBuffer(maxHoldSam);

    % super-coarse CFO state
    coarseBuff        = [];
    buffLen           = cfg.Cfo.SuperCoarseBuffLen;
    superCoarseFreq   = 0;
    isSuperCoarseReady = false;
    sampleIndex       = 0;

    while true
        [xRaw, len, over] = rfSrc.readFrame(); %#ok<NASGU>
        if over
            fprintf('Overrun/short read (%d < %d), skipping chunk\n', ...
                    numel(xRaw), SamplesPerFrame);
            continue;
        end

        % --- super coarse CFO on time-domain samples ---
        if isSuperCoarseReady && superCoarseFreq ~= 0
            N = numel(xRaw);
            n = (0:N-1).' + sampleIndex;
            xRaw = xRaw .* exp(-1j * 2*pi*superCoarseFreq/Fs .* n);
            sampleIndex = sampleIndex + N;
        end

        if ~isSuperCoarseReady
            coarseBuff = [coarseBuff; xRaw]; %#ok<AGROW>
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

        % --- DC + AGC + RRC ---
        xDC  = dcblock.process(xRaw);
        xAGC = agc.process(xDC);
        yDet = rrcDet.process(xAGC);

        yDetBuf.append(yDet);

        % --- Process as many complete frames as possible ---
        while true
            if yDetBuf.Length < frameSam
                break;
            end

            yDetVec  = yDetBuf.toVector();
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

            [~, idxBest]     = max(score);
            wSym_sc_best     = wSym_sc_all(idxBest);
            fCfoHz_meas_best = wSym_sc_best * Rsym / (2*pi);

            if cfoInitialized
                fCfoHz_use = fCfoHz_trk;
            else
                fCfoHz_use = fCfoHz_meas_best;
            end
            wSym_use = 2*pi * fCfoHz_use / Rsym;

            % symbol-rate CFO correction
            ySymCfoCell = cell(1, sps);
            for uu = 1:numel(uniqueOffs)
                off = uniqueOffs(uu);
                ySym = ySymDetCell{off+1};
                if isempty(ySym), continue; end
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
            numSam       = yDetBuf.Length;
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
                payStartS_i = payStartS(ic);
                payEndS_i   = payEndS(ic);
                preEndS_i   = preEndS(ic);
                lastSampleIx_i = lastSampleIx(ic);

                ySym_cfo = ySymCfoCell{off+1};
                NsymDet  = NsymDetVec(off+1);

                if preEndS_i > NsymDet || payEndS_i > NsymDet
                    continue;
                end

                candPre = ySym_cfo(preStartSym(ic):preEndS_i);
                cCorr   = abs(candPre' * preSyms) / ...
                          (norm(candPre)*norm(preSyms) + eps);

                if cCorr < 0.7
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

            % === Batch all accepted frames into ONE job ===
            nAcc = numel(acceptedIdx);
            bigPayRaw   = complex([]);
            segStartIdx = zeros(nAcc, 1, 'int32');
            segLen      = zeros(nAcc, 1, 'int32');
            frameIdxVec = zeros(nAcc, 1, 'int32');

            maxDropSamples = 0;

            for k = 1:nAcc
                off   = acceptedOffsets(k);
                ySymC = ySymCfoCell{off+1};
                s0    = acceptedPayStart(k);
                s1    = acceptedPayEnd(k);

                seg   = ySymC(s0:s1);

                segStartIdx(k) = numel(bigPayRaw) + 1;
                segLen(k)      = numel(seg);
                bigPayRaw      = [bigPayRaw; seg]; %#ok<AGROW>

                % --- CFO tracking exactly as before ---
                wSym_sc_frame = acceptedWSym_sc(k);
                fCfoHz_meas   = wSym_sc_frame * Rsym / (2*pi);

                if ~isnan(lastCfoRadPerSymDet)
                    delta = angle(exp(1j*(wSym_sc_frame - lastCfoRadPerSymDet)));
                    if abs(delta) > cfoWarnThreshRad
                        fprintf(['Warning: large change in S&C CFO: ' ...
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

                frames = frames + 1;
                frameIdxVec(k) = frames;

                lastSamThis = acceptedLastSample(k);
                if lastSamThis > maxDropSamples
                    maxDropSamples = lastSamThis;
                end
            end

            % --- Send ONE job containing many frames ---
            job = struct();
            job.PayloadSyms = bigPayRaw;    % concatenated payload symbols
            job.SegStartIdx = segStartIdx;  % per-frame start index
            job.SegLen      = segLen;       % per-frame length
            job.FrameIndex  = frameIdxVec;  % frame indices for labeling

            send(paySymQueue, job);

            if maxDropSamples == 0
                break;
            end
            dropSamples = min(maxDropSamples, yDetBuf.Length);
            yDetBuf.dropFirst(dropSamples);
        end
    end
end
