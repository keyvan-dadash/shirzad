% RX: QPSK/16-QAM over USRP N210 with symbol-rate preamble detection + CFO/phase pre-corr + tight PLL + cumulative BER + EVM
clear; clc;

assert(exist('comm.SDRuReceiver','class')==8, ...
  ['Install "Communications Toolbox Support Package for USRP Radio". ', ...
   'Home > Add-Ons > Get Hardware Support Packages.']);

%% ---------- User/Radio params (MUST MATCH TX) ----------
fc              = 10e6;     % use 915e6 for SBX/WBX/UBX; 10e6 only for LFRX/LFTX
MasterClockRate = 100e6;
Fs              = 1e6;
Decim           = MasterClockRate/Fs;  assert(round(Decim)==Decim,'Decim must be integer');

M   = 4;                      % 4=QPSK, 16=16-QAM
bps = log2(M);
sps = 8;  beta = 0.35; span = 10;

preambleLen = 128;            % symbols
payloadSyms = 2000;           % symbols/frame
rxGain_dB   = 0;

SamplesPerFrame = 32768;

%% ---------- Known sequences (match TX exactly) ----------
rng(1001); refBits = randi([0 1], payloadSyms*bps, 1);  % payload reference bits
rng(42);   preBits = randi([0 1], preambleLen*bps, 1);  % preamble bits
preSyms = qammod(preBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

% Demapper handle
qamDem = @(z) qamdemod(z, M, 'gray', 'OutputType','bit', 'UnitAveragePower', true);

% (small speed-up) reference constellation for EVM once
refSymsConst = qammod(refBits, M, 'gray', 'InputType','bit', 'UnitAveragePower', true);

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
carSyncNow = carSyncCoarse;
useFine = false;

%% ---------- Radio ----------
rx = comm.SDRuReceiver('Platform','N200/N210/USRP2', ...
    'IPAddress','192.168.10.4', ...
    'CenterFrequency',fc, ...
    'MasterClockRate',MasterClockRate, ...
    'DecimationFactor',Decim, ...
    'Gain',rxGain_dB, ...
    'SamplesPerFrame',SamplesPerFrame, ...
    'OutputDataType','double', ...
    'TransportDataType','int16', ...
    'EnableBurstMode',false);
try, rx.Antenna = 'RX2'; end   % change to 'TX/RX' if that’s your cable

for i=1:10, step(rx); end   % warm up & flush

%% ---------- Spectrum analyzer (only visual) ----------
sa = dsp.SpectrumAnalyzer('SampleRate',Fs, ...
    'PlotAsTwoSidedSpectrum',true, 'SpectrumType','Power density', ...
    'Title','RX spectrum (post DC/AGC)');

%% ---------- Symbol-rate detector thresholds ----------
MU_THR_S  = 0.75;    % normalized corr peak at 1 sps
PSR_THR_S = 2.0;     % peak-to-sidelobe ratio
POW_THR_S = 0.95;    % corr-window power / median

EpreS = sum(abs(preSyms).^2);   % energy of symbol-rate preamble

%% ---------- Receive/decode loop ----------
disp('RX: waiting for frames…');
yBuf   = complex([]); 
frames = 0;

% === use normal variables instead of 'persistent' (scripts can't declare persistent) ===
totErr  = 0;
totBits = 0;

while true
    % --- pull chunk ---
    [x,len] = rx(); if len==0, pause(0.005); continue; end

    % front-end conditioning
    x = dcblock(x);
    x = agc(x);

    % spectrum view (cheap)
    sa(x);

    % matched filter (still at sps)
    y = rrcRX(x);

    % accumulate & bound buffer at sample rate
    yBuf = [yBuf; y]; %#ok<AGROW>
    maxHold = 2*(preambleLen*sps + payloadSyms*sps) + 8*sps;
    if numel(yBuf) > maxHold, yBuf = yBuf(end-maxHold+1:end); end
    if numel(yBuf) < preambleLen*sps + payloadSyms*sps + 8*sps, continue; end

    % ================= SYMBOL-RATE DETECTION =================
    bestFound = false;
    best = struct('off',0,'pk',-inf,'idx',0,'PSR',0,'pow',0);

    for off = 0:(sps-1)
        % Build symbol-rate stream for this phase
        ySym = yBuf(1+off : sps : end);
        if numel(ySym) < preambleLen + payloadSyms + 8, continue; end

        % Normalized correlation at symbol rate (end-aligned)
        rS  = filter(conj(flipud(preSyms)), 1, ySym);
        eyS = filter(ones(preambleLen,1), 1, abs(ySym).^2);
        muS = (abs(rS).^2) ./ max(EpreS * (eyS), eps);

        [pkS, idxS] = max(muS);

        % PSR & power gates at symbol rate
        guardS = max(4, round(0.3*preambleLen));
        mu2S = muS; mu2S(max(1,idxS-guardS):min(end,idxS+guardS)) = 0;
        PSRS      = pkS / max(max(mu2S), eps);
        winPowS   = eyS(idxS) / preambleLen;
        noiseMedS = median(eyS) / preambleLen;
        powRatioS = winPowS / max(noiseMedS, eps);

        % Keep best candidate across timing phases
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
    if ~(best.pk > MU_THR_S && best.PSR > PSR_THR_S && best.pow > POW_THR_S)
        continue;  % not a valid preamble yet
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

    % ---- carrier/phase recovery at 1 sps (coarse then fine after a few frames) ----
    rxSyms_eq = carSyncNow(rxSyms_raw);

    % ---- IQ quadrant/polarity snap (±0/90/180/270°) ----
    G = [1, -1, 1j, -1j];
    errs = zeros(1,4);
    for g = 1:4
        rb = qamDem(rxSyms_eq * G(g));
        errs(g) = mean(rb ~= refBits);
    end
    [frameBER, ig] = min(errs);
    rxSyms = rxSyms_eq * G(ig);
    rxBits = qamDem(rxSyms);

    % ---- cumulative BER & EVM-based SNR ----
    e       = sum(rxBits ~= refBits);
    totErr  = totErr  + e;
    totBits = totBits + numel(refBits);
    cumBER  = totErr / totBits;

    evmRMS  = sqrt(mean(abs(rxSyms - refSymsConst).^2) / mean(abs(refSymsConst).^2));
    SNRdB   = -20*log10(max(evmRMS, eps));     % ≈ Es/N0 (dB)

    frames = frames + 1;
    fprintf(['Frame %d | off=%d | mu=%.2f | PSR=%.2f | PowRatio=%.2f | ' ...
             'BER=%.3e | CUM=%.3e (%d bits) | Es/N0≈%.1f dB | CFO(rad/sym)=%.3g\n'], ...
            frames, off, best.pk, best.PSR, best.pow, frameBER, cumBER, totBits, SNRdB, wSym);

    % ---- tighten loops & freeze AGC after a few frames ----
    if ~useFine && frames >= 5
        agc.AdaptationStepSize = 1e-9;   % must be > 0; tiny ≈ frozen (tunable)
        carSyncNow = carSyncFine;        % switch to narrow PLL
        useFine = true;
    end

    % ---- drop consumed samples from yBuf (SAMPLE-RATE indices) ----
    % ySym = yBuf(1+off : sps : end).  Symbol index (idxS+payloadSyms) maps to:
    % sample index = 1 + off + (idxS + payloadSyms)*sps
    needTail_guard = 4*sps;  % small guard at sps
    end_consumed = 1 + off + (idxS + payloadSyms)*sps + needTail_guard - 1;
    end_consumed = min(end_consumed, numel(yBuf));
    yBuf = yBuf(end_consumed+1 : end);
end
