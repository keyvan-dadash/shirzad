% RX over UDP: QPSK with symbol-rate preamble detection
%     + CFO/phase pre-corr + tight PLL + decision-directed SNR (NO FEC)
clear; clc;

% For UDP loopback we need DSP System Toolbox (dsp.UDPReceiver)
assert(exist('dsp.UDPReceiver','class')==8, ...
  ['Install "DSP System Toolbox" for UDP receiver support.']);

%% ---------- User/link params (MUST MATCH TX) ----------
fc              = 10e6;     % kept for consistency (not used in UDP mode)
MasterClockRate = 100e6;   % same
Fs              = 1e6;     % symbol-rate * sps
Decim           = MasterClockRate/Fs;  %#ok<NASGU>  % not used in UDP mode

M   = 4;                      % 4=QPSK
bps = log2(M);
sps = 10; beta = 0.35; span = 10;     % keep sps=10 to match TX

% IMPORTANT: these must match TX settings
preambleLen = 128;            % symbols
payloadSyms = 270;            % symbols/frame
rxGain_dB   = 3;              %#ok<NASGU>

SamplesPerFrame = 4000;      % max UDP samples we are willing to accept

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

%% ---------- Payload length & reference bits (NO FEC) ----------
% Uncoded bits per frame:
infoBitsLen = payloadSyms * bps;   % for QPSK: 270*2 = 540 bits

% Reference *info* bits (same seed as TX) for BER & quadrant resolution:
rng(1001); 
refBits_info = randi([0 1], infoBitsLen, 1);
% No encRefBits, no convolutional coding here.

%% ---------- Known preamble (match TX exactly) ----------
rng(42); preBits = randi([0 1], preambleLen*bps, 1);
preSyms = modQPSK.modulate(preBits);
EpreS   = sum(abs(preSyms).^2);

% Demapper (hard bits)
qamDemBits = @(z) demQPSK.demodulateHard(z);

%% ---------- DSP chain ----------
rrcRX  = filters.RootRaisedCosineFilter(beta, span, sps);
agc = gain.SimpleAgc( ...
    'AveragingLength',    1000, ...
    'MaximumGain_dB',     30, ...
    'AdaptationStepSize', 1e-3, ...
    'TargetPower',        1.0);
dcblock = dsp.DCBlocker('Length',64);

% Carrier sync: coarse (startup) and fine (steady-state) at 1 sps
carSyncCoarse = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.01);

carSyncFine = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.002);

carSyncNow = carSyncCoarse;
useFine    = false;

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
PSR_THR_S = 0.99;    % peak-to-sidelobe ratio
POW_THR_S = 0.97;    % corr-window power / median

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
    if ~useFine
        x = agc.process(x);
    end
    sa(x);

    % Matched filter (still at sps)
    y = rrcRX.process(x);

    % Accumulate & bound buffer at sample rate
    yBuf = [yBuf; y]; %#ok<AGROW>
    maxHold = 2*(preambleLen*sps + payloadSyms*sps) + 8*sps;
    if numel(yBuf) > maxHold, yBuf = yBuf(end-maxHold+1:end); end
    if numel(yBuf) < preambleLen*sps + payloadSyms*sps + 8*sps, continue; end

    % ================= SYMBOL-RATE DETECTION =================
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
    rxSyms_eq = carSyncNow.process(rxSyms_raw);

    % ---- Resolve ±/±j quadrant using hard bits vs reference bits ----
    G = [1, -1, 1j, -1j];
    errs = zeros(1,4);
    for g = 1:4
        rb = qamDemBits(rxSyms_eq * G(g));         % hard bits for this quadrant
        K  = min(numel(rb), numel(refBits_info));
        errs(g) = mean(rb(1:K) ~= refBits_info(1:K));
    end
    [~, ig] = min(errs);
    rxSyms = rxSyms_eq * G(ig);

    % ===== Hard-decision demod and BER (NO FEC) =====
    rxBits = qamDemBits(rxSyms);

    errMask = (rxBits(1:infoBitsLen) ~= refBits_info);
    errIdx  = find(errMask);
    
    
    fprintf('Error indices this frame: ');
    disp(errIdx.');
    

    frameBER = mean(rxBits(1:infoBitsLen) ~= refBits_info);
    totErr   = totErr + sum(rxBits(1:infoBitsLen) ~= refBits_info);
    totBits  = totBits + infoBitsLen;
    cumBER   = totErr / max(totBits,1);

    % Payload bits passed to sink
    payBits = rxBits(1:infoBitsLen);

    % ---- decision-directed EVM -> Es/N0 (clean SNR) ----
    hb2 = qamDemBits(rxSyms);
    zh2 = modQPSK.modulate(hb2);
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
