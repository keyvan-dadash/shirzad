function tx_benchmark_no_usrp()
clear; clc;

cfg = phyAppConfig();

MasterClockRate = cfg.Link.MasterClockRate;
Interp          = cfg.Link.Interp;
Fs              = MasterClockRate / Interp;

sps   = cfg.Link.Sps;
beta  = cfg.Link.RrcBeta;
span  = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;

msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen;

modulator   = modulators.getMmodulator(cfg.Modulation.Name);
demodulator = demodulators.getDemodulator(cfg.Modulation.Name);

M   = modulator.M;
bps = modulator.BitsPerSymbol;

enc      = fec.ConvEncoder.rateHalf_K3();
K        = cfg.Fec.ConstraintLength;
Kminus1  = K - 1;

fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
fecDecodeDummy = @(codedBits) error('Payload.decode is not used in TX.');

pre = protocol.Preamble.fromMSequence( ...
    modulator, ...
    preambleHalfLen, ...
    'Degree', cfg.Frame.MseqDegree, ...
    'Seed',   cfg.Frame.MseqSeed);

pay = protocol.Payload(modulator, demodulator, ...
              payloadSyms, msgCapBytes, pilotBitsLen, ...
              fecEncodeFcn, fecDecodeDummy);

fr = protocol.Frame(pre, pay);
preambleLen = fr.NumPreambleSymbols;

hdrBytes        = double(protocol.Datagram.HEADER_BYTES);
maxProtoPayload = msgCapBytes - hdrBytes;

databitsLen  = fr.Payload.DataBitsLen;
codedBitsLen = fr.Payload.CodedBitsLen;
padBitsLen   = fr.Payload.PadBitsLen;

L_in = databitsLen + Kminus1;
assert(codedBitsLen == 2 * L_in, ...
    'TX: codedBitsLen (%d) != 2*(databitsLen+%d)=%d.', ...
    codedBitsLen, Kminus1, 2*L_in);

fprintf('TX BENCH (no USRP, pre-encoded frames): protocol+FEC layout\n');
fprintf('  datagram bytes  : %d (header=%d, payload<=%d)\n', ...
    msgCapBytes, hdrBytes, maxProtoPayload);
fprintf('  databitsLen     : %d bits\n', databitsLen);
fprintf('  T = L_in        : %d time steps\n', L_in);
fprintf('  codedBitsLen    : %d bits, padBits=%d\n', codedBitsLen, padBitsLen);

dgramSrc = sources.DatagramSource(msgCapBytes, cfg.Tx.StreamSpecs);

txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

NsymFrame = preambleLen + payloadSyms;
NsampUp   = NsymFrame * sps;

fprintf('Frame: %d symbols, %d samples (upsampled)\n', NsymFrame, NsampUp);

NBUF_FRAMES   = 1000;
preFrames     = cell(NBUF_FRAMES,1);
pilotAmp      = cfg.Frame.PilotAmpOffset;

fprintf('Pre-encoding %d frames (full FEC + framing)...\n', NBUF_FRAMES);

for k = 1:NBUF_FRAMES
    [protoBytes, ~] = dgramSrc.readFrame(); 

    [frmSyms_raw, ~] = fr.encode(protoBytes);

    frmSyms = pilotAmp + frmSyms_raw;
    preFrames{k} = single(frmSyms);
end

fprintf('Pre-encoding done.\n');

NFRAMES_BENCH = 10000;

fprintf('Benchmarking %d frames using %d pre-encoded frames (circular reuse)...\n', ...
        NFRAMES_BENCH, NBUF_FRAMES);

tic;
for n = 1:NFRAMES_BENCH
    idx = mod(n-1, NBUF_FRAMES) + 1;
    frmSyms = preFrames{idx};

    up = complex(zeros(numel(frmSyms)*sps, 1, 'single'));
    up(1:sps:end) = frmSyms;

    txWave = txRRC.process(up);
end
elapsed = toc;

framesPerSec = NFRAMES_BENCH / elapsed;

phyBytesPerFrame = msgCapBytes;
phyRate_kBps     = (framesPerSec * phyBytesPerFrame) / 1024;

maxAppPayloadBytes = maxProtoPayload;
maxAppRate_kBps    = (framesPerSec * maxAppPayloadBytes) / 1024;

fprintf('\nTX BENCH RESULTS (no USRP, pre-encoded frames):\n');
fprintf('  Frames processed : %d\n', NFRAMES_BENCH);
fprintf('  Elapsed time     : %.3f s\n', elapsed);
fprintf('  Frames per second: %.1f frames/s\n', framesPerSec);
fprintf('  PHY rate         : %.1f kB/s (MsgCap=%dB)\n', ...
        phyRate_kBps, phyBytesPerFrame);
fprintf('  Max APP rate     : %.1f kB/s (payload<=%dB)\n', ...
        maxAppRate_kBps, maxAppPayloadBytes);

end
