% RX over USRP: QPSK with repeated-preamble detection [a,a]
%   + Schmidl-based CFO estimate (symbol-rate, from repeated preamble)
%   + PLL for fine carrier/phase tracking
%   + Convolutional FEC decode
%   + File transfer over protocol.Datagram + mini file-chunk header

clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP/USRP support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;
MasterClockRate = 100e6;
Decim           = 64;
Fs              = MasterClockRate/Decim;

M   = 4;  bps = log2(M);
sps = 10; beta = 0.35; span = 10;

preambleHalfLen = 128;
preambleLen     = 2 * preambleHalfLen;

% Bigger payload: 2048
payloadSyms     = 2048;
frameSyms       = preambleLen + payloadSyms;
rxGain_dB       = 0;

SamplesPerFrame = 24000;   % ~1 frame per read

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

%% FEC decoder
dec = fec.ViterbiDecoder.rateHalf_K3();
K   = 3;
Kminus1 = K - 1;

%% Protocol sizes (MUST MATCH TX)
infoBitsLen  = payloadSyms * bps;   % 2048*2 = 4096 bits
pilotBitsLen = 100;

msgCapBytes     = 248;
hdrBytes        = double(protocol.Datagram.HEADER_BYTES); % 8
maxProtoPayload = msgCapBytes - hdrBytes;                 % 240

databitsLen    = 8 * msgCapBytes;          % 1984 bits
L_in           = databitsLen + Kminus1;    % 1986
codedBitsLen   = 2 * L_in;                 % 3972 bits
bitsAfterPilot = infoBitsLen - pilotBitsLen;  % 3996
padBitsLen     = bitsAfterPilot - codedBitsLen; % 24 bits

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

%% Preamble
mseqGen = training.MSequenceGenerator('Degree', 9);
preBitsHalf = mseqGen.generateBits(preambleHalfLen * bps);
preSymsHalf = modQPSK.modulate(preBitsHalf);
preSyms     = [preSymsHalf; preSymsHalf];
Lpre        = numel(preSyms);

qamDemBits = @(z) demQPSK.demodulateHard(z);
Rsym = Fs / sps;

%% ---------- DSP chain ----------
rrcDet = filters.RootRaisedCosineFilter(beta, span, sps);

agc = gain.SimpleAgc( ...
    'AveragingLength',    1000, ...
    'MaximumGain_dB',     30, ...
    'AdaptationStepSize', 1e-3, ...
    'TargetPower',        1.0);

dcblock = filters.DcBlocker('Length',1024);

carSyncCoarse = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.2);

carSyncFine = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.01);

carSyncNow = carSyncCoarse;
useFine    = false;

%% ---------- Repeated preamble detector ----------
preDet = sync.RepeatedPreambleDetector( ...
    'SamplesPerSymbol', sps, ...
    'PreambleHalfLen',  preambleHalfLen, ...
    'MetricThreshold',  0.2, ...
    'MinWindowPower',   1e-7);

%% ---------- Source & Sinks ----------
rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        '192.168.10.4', ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame);

paySink = sinks.PayloadCollectorSink();

fileId    = uint8(1);
outFile   = 'U:\Chalmers\MCC125\codes\shirzad\test5.pdf';  % use local disk for speed if possible
fileWriter = io.FileWriter(outFile);
assembler  = filetransfer.FileAssembler(fileId, fileWriter);

% constDiag = comm.ConstellationDiagram( ...
%     'SamplesPerSymbol', 1, ...
%     'Name', 'RX Constellation (post-PLL, post-quadrant-fix)', ...
%     'XLimits', [-2 2], ...
%     'YLimits', [-2 2]);

%% ---------- Debug flags ----------
DEBUG_GENERAL = false;   % overall status, CFO, Es/N0, overrun messages
DEBUG_SYNC    = false;  % "No preamble" spam
DEBUG_FILE    = false;   % file-chunk / assembler progress
DEBUG_TIMING  = false;   % timing breakdown prints

%% ---------- Buffers & counters ----------
disp('RX: waiting for frames…');

frameSam   = frameSyms * sps;
maxHoldSam = 10*frameSam + 8*sps + span*sps;

% Fixed-size sliding buffers
xBuf    = complex(zeros(maxHoldSam,1));
yDetBuf = complex(zeros(maxHoldSam,1));
bufLen  = 0;      % number of valid samples in xBuf(1:bufLen) / yDetBuf(1:bufLen)

sa = spectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, ...
    'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

coarseBuff = [];
buffLenCfo = 16384;

superCoarseFreq    = 0;
isSuperCoarseReady = false;
sampleIndex        = 0;

cfoInitialized     = false;
fCfoHz_trk         = 0;
cfoAlpha           = 0.05;
cfoMaxJumpHz       = 200;
metricTrustThresh  = 0.24;

frames          = 0;
noPreambleCount = 0;

%% ---------- Timing accumulators ----------
timeRead     = 0;
timeSuperCfo = 0;
timeFront    = 0;
timeDetect   = 0;
timeFec      = 0;

% Detect/PLL breakdown
timePre      = 0;   % preDet.detect()
timeCfo      = 0;   % symbol-rate CFO + preamble corr + Es/N0 estimate
timePLL      = 0;   % carrier PLL + quadrant search

readCalls    = 0;

minOff = 10000000000;

%% ================== MAIN RX LOOP ==================
while true
    %% ---------- Pull chunk from source ----------
    t0 = tic;
    [xRaw, srcInfo] = rfSrc.readFrame();
    timeRead = timeRead + toc(t0);
    readCalls = readCalls + 1;

    if ~srcInfo.IsValid
        pause(0.05);
        continue;
    end

    if srcInfo.Overrun
        if DEBUG_GENERAL
            fprintf('Overrun/short read (%d < %d), resetting RX state\n', ...
                numel(xRaw), SamplesPerFrame);
        end

        bufLen = 0;
        useFine = false;
        agc.AdaptationStepSize = 1e-3;
        cfoInitialized = false;
        fCfoHz_trk     = 0;
        coarseBuff = [];
        isSuperCoarseReady = false;
        superCoarseFreq = 0;
        sampleIndex = 0;
        continue;
    end

    %% ---------- Super-coarse CFO (once) ----------
    if ~isSuperCoarseReady
        tSC = tic;
        coarseBuff = [coarseBuff; xRaw];

        if numel(coarseBuff) > buffLenCfo
            XCfo = coarseBuff(1:buffLenCfo);

            w = hann(buffLenCfo);
            fft_result = fftshift(fft(XCfo .* w));

            [~, peak] = max(abs(fft_result));
            peak_new = peak - (buffLenCfo / 2 + 1);
            superCoarseFreq = peak_new * Fs / buffLenCfo;

            if DEBUG_GENERAL
                fprintf('Super-coarse CFO ~ %.3f Hz (old/new bins %d/%d)\n', ...
                    superCoarseFreq, peak, peak_new);
            end

            isSuperCoarseReady = true;
            coarseBuff = [];
        end
        timeSuperCfo = timeSuperCfo + toc(tSC);
    end

    %% ---------- Apply super-coarse CFO ----------
    tFrontStart = tic;

    if isSuperCoarseReady && superCoarseFreq ~= 0
        N = numel(xRaw);
        n = (0:N-1).' + sampleIndex;
        xRaw = xRaw .* exp(-1j * 2*pi*superCoarseFreq/Fs .* n);
        sampleIndex = sampleIndex + N;
    end

    % sa(xRaw);

    %% ---------- DC blocker + AGC + RRC ----------
    xDC = dcblock.process(xRaw);
    if ~useFine
        xAGC = agc.process(xDC);
    else
        xAGC = xDC;
    end

    yDet = rrcDet.process(xAGC);

    % ---------- Update fixed-size buffers (xBuf, yDetBuf) ----------
    Nnew = numel(xAGC);  % == numel(yDet)

    if Nnew >= maxHoldSam
        % Keep only the most recent maxHoldSam samples
        xBuf(:)    = xAGC(end-maxHoldSam+1:end);
        yDetBuf(:) = yDet(end-maxHoldSam+1:end);
        bufLen     = maxHoldSam;
    else
        total = bufLen + Nnew;
        if total <= maxHoldSam
            % Just append
            xBuf(bufLen+1:total)    = xAGC;
            yDetBuf(bufLen+1:total) = yDet;
            bufLen = total;
        else
            % Need to drop oldest 'drop' samples
            drop   = total - maxHoldSam;
            remain = bufLen - drop;
            if remain > 0
                xBuf(1:remain)    = xBuf(drop+1:bufLen);
                yDetBuf(1:remain) = yDetBuf(drop+1:bufLen);
            end
            % Append new samples
            xBuf(remain+1:remain+Nnew)    = xAGC;
            yDetBuf(remain+1:remain+Nnew) = yDet;
            bufLen = maxHoldSam;
        end
    end

    timeFront = timeFront + toc(tFrontStart);

    %% ---------- INNER LOOP ----------
    while true
        if bufLen < frameSam + 8*sps
            break;
        end

        % Search window for preamble & payload
        searchSyms   = numel(preSyms)*2 + 10;   % small lookahead window
        maxDetectSam = searchSyms * sps;
        Ndet = min(bufLen, maxDetectSam);

        tDetStart = tic;

        % ---------- preamble detector ----------
        yDetSearch = yDetBuf(1:4000);
        tPreStart  = tic;
        detRes     = preDet.detectFast(yDetSearch);
        timePre    = timePre + toc(tPreStart);

        if ~detRes.Found
            noPreambleCount = noPreambleCount + 1;
            if DEBUG_SYNC && mod(noPreambleCount, 100) == 0
                fprintf('No preamble: M=%.3f, Pow=%.3g (count=%d)\n', ...
                    detRes.Metric, detRes.WindowPower, noPreambleCount);
            end
            timeDetect = timeDetect + toc(tDetStart);
            break;
        end

        % ---------- CFO / preamble handling ----------
        tCfoStart  = tic;

        off         = detRes.SampleOffset;
        preStartSym = detRes.PreambleStartSym;
        wSym_sc     = detRes.CfoRadPerSym;
        fCfoHz_meas = wSym_sc * Rsym / (2*pi);

        % Symbol-rate stream from yDet buffer
        ySymDet = yDetBuf(1+off : sps : bufLen);
        NsymDet = numel(ySymDet);

        if preStartSym + Lpre - 1 > NsymDet
            % not enough symbols yet to hold full preamble
            timeDetect = timeDetect + toc(tDetStart);
            timeCfo    = timeCfo + toc(tCfoStart);
            break;
        end

        payStartS = preStartSym + preambleLen;
        if payStartS + payloadSyms - 1 > NsymDet
            % not enough symbols yet to hold payload
            timeDetect = timeDetect + toc(tDetStart);
            timeCfo    = timeCfo + toc(tCfoStart);
            break;
        end

        % CFO estimate / apply (symbol-rate)
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
            % low correlation with real preamble
            timeDetect = timeDetect + toc(tDetStart);
            timeCfo    = timeCfo + toc(tCfoStart);
            break;
        end

        % ---------- Es/N0 estimate from corrected preamble ----------
        % Model candPre ≈ h * preSyms + noise, solve LS for h
        h_hat  = (preSyms' * candPre) / (preSyms' * preSyms);
        sig    = h_hat * preSyms;
        noise  = candPre - sig;

        Es_hat = mean(abs(sig).^2);
        N0_hat = mean(abs(noise).^2) + eps;
        EsN0_dB = 10*log10(Es_hat / N0_hat);

        if DEBUG_GENERAL
            fprintf(['Frame %d (candidate): detM=%.3f, corr=%.3f, ' ...
                     'CFO=%.1f Hz, Es/N0≈%.1f dB\n'], ...
                frames+1, detRes.Metric, c, fCfoHz_meas, EsN0_dB);
        end

        % ---------- extract payload symbols ----------
        payEndS = payStartS + payloadSyms - 1;
        if payEndS > numel(ySym_cfo)
            timeDetect = timeDetect + toc(tDetStart);
            timeCfo    = timeCfo + toc(tCfoStart);
            break;
        end
        rxSyms_raw = ySym_cfo(payStartS:payEndS);

        timeCfo = timeCfo + toc(tCfoStart);

        % ---------- PLL + quadrant search ----------
        tPLLStart = tic;
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
        timePLL = timePLL + toc(tPLLStart);

        % constDiag(rxSyms);

        timeDetect = timeDetect + toc(tDetStart);

        %% ---------- FEC decode + file assembly timing ----------
        tFecStart = tic;

        frames = frames + 1;

        rxBits = qamDemBits(rxSyms);
        if numel(rxBits) < infoBitsLen
            if DEBUG_GENERAL
                fprintf('Frame %d: not enough bits (%d < %d)\n', ...
                    frames, numel(rxBits), infoBitsLen);
            end
            timeFec = timeFec + toc(tFecStart);
            break;
        end

        % [pilot | codedBits | padBits]
        codedBits = rxBits(pilotBitsLen+1 : pilotBitsLen+codedBitsLen);
        if numel(codedBits) < codedBitsLen
            if DEBUG_GENERAL
                fprintf('Frame %d: not enough coded bits (%d < %d)\n', ...
                    frames, numel(codedBits), codedBitsLen);
            end
            timeFec = timeFec + toc(tFecStart);
            break;
        end

        % uBits_hat = dec.decode(logical(codedBits));
        uBits_hat = fec.viterbi_k3_mex(logical(codedBits));
        uBits_hat = double(uBits_hat(:));

        if numel(uBits_hat) < databitsLen
            if DEBUG_GENERAL
                warning('Frame %d: decoded bits %d < required data bits %d', ...
                    frames, numel(uBits_hat), databitsLen);
            end
            timeFec = timeFec + toc(tFecStart);
            break;
        end

        dataBits = uBits_hat(1:databitsLen);
        dataBitsMatrix = reshape(dataBits, 8, []).';
        dataBytes      = uint8(bi2de(dataBitsMatrix, 'left-msb'));

        %% ---------- parse protocol datagram ----------
        [pkt, ok] = protocol.Datagram.fromBytes(dataBytes);

        if ~ok
            % if DEBUG_GENERAL
            %     warning('Frame %d: datagram checksum FAILED (seq=%d). Dropping payload.', ...
            %         frames, pkt.SeqNum);
            % end
            % pkt.debugPrint();
        else
            % Interpret pkt.Payload as FILE-CHUNK payload
            pay = pkt.Payload(1 : pkt.PayloadLen);
            try
                [meta, chunkData] = filetransfer.FileChunk.decode(pay);
                if meta.FileId == assembler.FileId
                    assembler.acceptChunk(meta.Offset, meta.TotalSize, meta.IsLast, chunkData);

                    [offTot, tot, hasTotal, bufBytes] = assembler.status();

                    if meta.Offset < minOff
                        minOff = meta.Offset;
                    end

                    if DEBUG_FILE || mod(frames,500) == 0
                        fprintf('Chunk off=%u len=%d isLast=%d | written=%u, buffered=%u, minOff=%u\n', ...
                            meta.Offset, numel(chunkData), meta.IsLast, offTot, bufBytes, minOff);

                        if hasTotal
                            fprintf('File progress: %d / %d bytes (%.1f%%), buffered=%d bytes\n', ...
                                offTot, tot, 100*double(offTot)/double(tot), bufBytes);
                        else
                            fprintf('File progress: %d bytes written so far, %d bytes buffered (total unknown)\n', ...
                                offTot, bufBytes);
                        end
                    end

                    if assembler.isComplete()
                        [offTot, tot, hasTotal] = assembler.status();
                        fprintf('File transfer COMPLETE: %d/%d bytes (HasTotal=%d)\n', ...
                            offTot, tot, hasTotal);
                        assembler.Writer.close();
                        timeFec = timeFec + toc(tFecStart);
                        return;
                    end
                else
                    if DEBUG_FILE
                        fprintf('Frame %d: unknown FileId=%d, ignoring.\n', ...
                            frames, meta.FileId);
                    end
                end
            catch ME
                if DEBUG_GENERAL
                    warning('Frame %d: failed to decode file chunk: %s', ...
                        frames, ME.message);
                end
            end
        end

        paySink.writeFrame(dataBits, struct('FrameIndex', frames));

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

        timeFec = timeFec + toc(tFecStart);

        %% ---------- drop consumed samples from buffers ----------
        lastSymIdx   = payStartS + payloadSyms - 1;
        lastSampleIx = 1 + off + (lastSymIdx-1)*sps;  % 1-based index in yDetBuf/xBuf

        end_consumed = min(lastSampleIx, bufLen);
        remain = bufLen - end_consumed;
        if remain > 0
            xBuf(1:remain)    = xBuf(end_consumed+1:bufLen);
            yDetBuf(1:remain) = yDetBuf(end_consumed+1:bufLen);
        end
        bufLen = remain;

        %% ---------- periodic timing report (AVERAGES) ----------
        if frames > 0 && mod(frames,50) == 0 && DEBUG_TIMING
            % Averages (not cumulative totals)
            avgRead  = timeRead     / max(readCalls, 1);
            avgSCFO  = timeSuperCfo / max(readCalls, 1);
            avgFront = timeFront    / max(frames,    1);
            avgDet   = timeDetect   / max(frames,    1);
            avgFec   = timeFec      / max(frames,    1);

            avgPre   = timePre      / max(frames,    1);
            avgCfo   = timeCfo      / max(frames,    1);
            avgPLL   = timePLL      / max(frames,    1);

            fprintf(['Timing (AVERAGE) over %d frames, %d reads:\n' ...
                     '  readFrame     : %.6f s / read\n' ...
                     '  superCFO FFT  : %.6f s / read\n' ...
                     '  front-end     : %.6f s / frame\n' ...
                     '  detect/PLL    : %.6f s / frame\n' ...
                     '  FEC+file      : %.6f s / frame\n'], ...
                frames, readCalls, ...
                avgRead, avgSCFO, avgFront, avgDet, avgFec);

            fprintf(['Detect breakdown (AVERAGE per frame):\n' ...
                     '  preDet.detect : %.6f s\n' ...
                     '  CFO/preamble  : %.6f s\n' ...
                     '  PLL+quadrant  : %.6f s\n'], ...
                avgPre, avgCfo, avgPLL);
        end
    end
end
