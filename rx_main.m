% RX over UDP: QPSK/16-QAM with symbol-rate preamble detection
%     + CFO/phase pre-corr + tight PLL + FEC (rate-1/2) + decision-directed SNR
clear; clc;

% For UDP loopback we need DSP System Toolbox (dsp.UDPReceiver)
assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP receiver support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;     % kept for consistency (not used in UDP mode)
MasterClockRate = 100e6;   % same
Fs              = 1e6;     % symbol-rate * sps
Decim           = MasterClockRate/Fs;  %#ok<NASGU>  % not used in UDP mode

M   = 4;                      % 4=QPSK, 16=16-QAM
bps = log2(M);
sps = 10; beta = 0.35; span = 10;     % keep sps=10 to match TX

% IMPORTANT: these must match TX settings
preambleLen = 128;            % symbols
payloadSyms = 270;            % symbols/frame (reduced to fit one UDP packet)
rxGain_dB   = 3;              %#ok<NASGU>  % not used here, but kept for symmetry

SamplesPerFrame = 4000;      % max UDP samples we are willing to accept

%% ---------- FEC (rate 1/2, K=7) ----------
useFEC = true;                              % toggle on/off
R    = 1/2;
trel = poly2trellis(7,[171 133]);
vitDec = comm.ViterbiDecoder( ...
    'TrellisStructure', trel, ...
    'InputFormat','Unquantized', ...        % LLRs: + => '1',  - => '0'
    'TracebackDepth', 48, ...
    'TerminationMethod','Truncated');
tb = vitDec.TracebackDepth;

% Reference *info* bits (same seed as TX) for BER display only:
infoBitsLen = round(payloadSyms*bps*R);     % for QPSK: 272*2*1/2 = 272
rng(1001); refBits_info = randi([0 1], infoBitsLen, 1);
encRefBits  = convenc(refBits_info, trel);  % used ONLY to pick the correct quadrant

%% ---------- Known preamble (match TX exactly) ----------
rng(42); preBits = randi([0 1], preambleLen*bps, 1);
preSyms = qammod(preBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);
EpreS   = sum(abs(preSyms).^2);

% Demapper (hard bits)
qamDemBits = @(z) qamdemod(z, M, 'gray', 'OutputType','bit', 'UnitAveragePower', true);

%% ---------- DSP chain ----------
rrcRX  = comm.RaisedCosineReceiveFilter( ...
    'RolloffFactor',beta,'FilterSpanInSymbols',span, ...
    'InputSamplesPerSymbol',sps,'DecimationFactor',1);
agc     = comm.AGC('AveragingLength',1000,'MaximumGain',30,'AdaptationStepSize',1e-3);
dcblock = dsp.DCBlocker('Length',64);

% Carrier sync: coarse (startup) and fine (steady-state) at 1 sps
carSyncCoarse = comm.CarrierSynchronizer( ...
    'Modulation','QAM','SamplesPerSymbol',1, ...
    'DampingFactor',0.707,'NormalizedLoopBandwidth',0.01);
carSyncFine   = comm.CarrierSynchronizer( ...
    'Modulation','QAM','SamplesPerSymbol',1, ...
    'DampingFactor',0.707,'NormalizedLoopBandwidth',0.002);   % tighter
carSyncNow = carSyncCoarse; useFine = false;

%% ---------- Source & Sink (UDP instead of USRP) ----------
udpPort = 31000;    % must match TX UDP sink

% RF source: UDPWaveformSource (complex double, blocking)
rfSrc = sources.UDPWaveformSource( ...
    'LocalIPPort',          udpPort, ...
    'SampleRate',           Fs, ...
    'MaximumMessageLength', SamplesPerFrame, ...
    'IsMessageComplex',     true, ...
    'MessageDataType',      'double', ...
    'Blocking',             true, ...
    'TimeoutSeconds',       1.0);

% Payload sink: collects decoded payload bits
paySink = sinks.PayloadCollectorSink();

%% ---------- Spectrum analyzer (optional) ----------
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, 'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

%% ---------- Symbol-rate detector thresholds ----------
MU_THR_S  = 0.75;    % normalized corr peak @ 1 sps
PSR_THR_S = 2.0;     % peak-to-sidelobe ratio
POW_THR_S = 0.95;    % corr-window power / median

%% ---------- Receive/decode loop ----------
disp('RX (UDP): waiting for frames…');
yBuf   = complex([]); 
frames = 0;
totErr  = 0;
totBits = 0;

while true
    % Pull chunk from Source (UDP)
    [x, srcInfo] = rfSrc.readFrame();
    if ~srcInfo.IsValid
        pause(0.005);
        continue;
    end

    x = dcblock(x);
    if ~useFine, x = agc(x); end
    sa(x);

    % Matched filter (still at sps)
    y = rrcRX(x);

    % Accumulate & bound buffer at sample rate
    yBuf = [yBuf; y]; %#ok<AGROW>
    maxHold = 2*(preambleLen*sps + payloadSyms*sps) + 8*sps;
    if numel(yBuf) > maxHold, yBuf = yBuf(end-maxHold+1:end); end
    if numel(yBuf) < preambleLen*sps + payloadSyms*sps + 8*sps, continue; end

    % ================= SYMBOL-RATE DETECTION (same logic) =================
    bestFound = false;
    best = struct('off',0,'pk',-inf,'idx',0,'PSR',0,'pow',0);

    for off = 0:(sps-1)
        ySym = yBuf(1+off : sps : end);
        if numel(ySym) < preambleLen + payloadSyms + 8, continue; end

        rS  = filter(conj(flipud(preSyms)), 1, ySym);               % correlation @ 1 sps
        eyS = filter(ones(preambleLen,1), 1, abs(ySym).^2);         % window power
        muS = (abs(rS).^2) ./ max(EpreS * (eyS), eps);              % 0..1

        [pkS, idxS] = max(muS);

        guardS = max(4, round(0.3*preambleLen));
        mu2S   = muS; mu2S(max(1,idxS-guardS):min(end,idxS+guardS)) = 0;
        PSRS      = pkS / max(max(mu2S), eps);
        winPowS   = eyS(idxS) / preambleLen;
        noiseMedS = median(eyS) / preambleLen;
        powRatioS = winPowS / max(noiseMedS, eps);

        if pkS > best.pk
            bestFound = true;
            best.off  = off;
            best.pk   = pkS;
            best.idx  = idxS;
            best.PSR  = PSRS;
            best.pow  = powRatioS;
        end
    end

    if ~bestFound, continue; end

    % If you see mu~0.98 but PSR~1, you can relax PSR gating while debugging
    if ~(best.pk > MU_THR_S && best.pow > POW_THR_S && best.PSR > PSR_THR_S)
        fprintf('mu=%.2f PSR=%.2f Pow=%.2f\n', best.pk, best.PSR, best.pow);
        continue;
    end

    % ---- carve payload (symbol-rate indices) ----
    off  = best.off;
    idxS = best.idx;                     % end of preamble in ySym
    ySym = yBuf(1+off : sps : end);

    payStartS = idxS + 1;                % first payload symbol (1 sps index)
    if (payStartS + payloadSyms - 1) > numel(ySym), continue; end

    % ---------- preamble-based phase & CFO pre-correction ----------
    preStartS = idxS - preambleLen + 1;
    if preStartS < 1, continue; end
    rxPre  = ySym(preStartS:idxS);                      % received preamble @ 1 sps
    phi    = unwrap(angle(rxPre .* conj(preSyms(:))));  % per-symbol phase error
    npre   = (0:preambleLen-1).';
    pp     = polyfit(npre, phi, 1);                     % phi ≈ wSym*n + phi0
    wSym   = pp(1);                                     % rad/symbol (CFO)
    phi0   = pp(2);                                     % constant phase

    m            = (payStartS : payStartS + payloadSyms - 1).' - preStartS;
    rxSyms_raw   = ySym(payStartS : payStartS + payloadSyms - 1) .* exp(-1j*(phi0 + wSym*m));

    % ---- carrier/phase recovery at 1 sps (coarse then fine) ----
    rxSyms_eq = carSyncNow(rxSyms_raw);

    % ---- Resolve ±/±j quadrant using coded hard bits vs reference coding ----
    G = [1, -1, 1j, -1j];
    errs = zeros(1,4);
    for g = 1:4
        rb = qamDemBits(rxSyms_eq * G(g));                      % coded hard bits
        K  = min(numel(rb), numel(encRefBits));
        errs(g) = mean(rb(1:K) ~= encRefBits(1:K));
    end
    [~, ig] = min(errs);
    rxSyms = rxSyms_eq * G(ig);

    % ===== Soft LLRs for Viterbi (if FEC enabled) =====
    if useFEC
        % Decision-directed noise variance for LLRs
        hb  = qamDemBits(rxSyms);
        zh  = qammod(hb, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);
        cH  = (zh' * rxSyms) / (zh' * zh + eps);
        eV  = rxSyms - cH * zh;
        varComplex  = max(mean(abs(eV).^2), 1e-8);     % complex noise variance
        noiseVarLLR = varComplex / 2;                  % per dimension

        llr = qamdemod(rxSyms, M, 'gray', 'OutputType','approxllr', ...
                       'UnitAveragePower', true, 'NoiseVariance', noiseVarLLR);
        % approxllr = log(P(b=1)/P(b=0)) => positive=1, negative=0

        reset(vitDec);
        decBits = vitDec(llr);
        if numel(decBits) < tb + infoBitsLen, continue; end
        decBits = decBits(tb+1 : tb + infoBitsLen);            % drop ramp-in

        frameBER = mean(decBits ~= refBits_info);
        totErr   = totErr + sum(decBits ~= refBits_info);
        totBits  = totBits + numel(refBits_info);

        payBits = decBits;
    else
        rxBits = qamDemBits(rxSyms);
        frameBER = mean(rxBits(1:infoBitsLen) ~= refBits_info);
        totErr   = totErr + sum(rxBits(1:infoBitsLen) ~= refBits_info);
        totBits  = totBits + infoBitsLen;

        payBits = rxBits(1:infoBitsLen);
    end
    cumBER  = totErr / max(totBits,1);

    % ---- decision-directed EVM -> Es/N0 (clean SNR) ----
    hb2 = qamDemBits(rxSyms);
    zh2 = qammod(hb2, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);
    cHd = (zh2' * rxSyms) / (zh2' * zh2 + eps);
    err = rxSyms - cHd * zh2;
    Es  = mean(abs(cHd * zh2).^2);
    Nv  = mean(abs(err).^2);
    SNRdB = 10*log10(max(Es/Nv, eps));      % ≈ Es/N0 in dB

    % ---- frame count & sink write ----
    frames = frames + 1;
    paySink.writeFrame(payBits, struct('FrameIndex',frames));

    fprintf(['Frame %d | off=%d | mu=%.2f | PSR=%.2f | PowRatio=%.2f | ' ...
             'BER=%.3e | CUM=%.3e (%d bits) | Es/N0≈%.1f dB | CFO(rad/sym)=%.3g\n'], ...
            frames, off, best.pk, best.PSR, best.pow, ...
            frameBER, cumBER, totBits, SNRdB, wSym);

    % ---- tighten loops & freeze AGC after a few frames ----
    if ~useFine && frames >= 5
        agc.AdaptationStepSize = 1e-9;   % tiny > 0; ≈ frozen
        carSyncNow = carSyncFine;        % switch to narrow PLL
        useFine = true;
    end

    % ---- drop consumed samples from yBuf (SAMPLE-RATE indices) ----
    needTail_guard = 4*sps;
    end_consumed = 1 + off + (idxS + payloadSyms)*sps + needTail_guard - 1;
    end_consumed = min(end_consumed, numel(yBuf));
    yBuf = yBuf(end_consumed+1 : end);
end
