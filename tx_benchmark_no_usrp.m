function tx_benchmark_no_usrp()
%TX_BENCHMARK_NO_USRP  Measure TX-side throughput with pre-encoded frames.
%
% Idea:
%   1) Use existing DatagramSource + Frame(FEC) pipeline OFFLINE to build
%      a buffer of pre-encoded symbol frames (frmSyms).
%   2) In the timed loop, only do:
%          - pick pre-encoded frmSyms (circular)
%          - upsample
%          - RRC filtering
%      (no per-frame convolutional encoding or protocol mapping).
%
% This approximates what you'd get if you did all FEC+framing once into
% a big buffer and then streamed from that buffer in real time.

clear; clc;

%% ---------- Load shared config ----------
cfg = phyAppConfig();

%% ---------- Link / PHY params ----------
MasterClockRate = cfg.Link.MasterClockRate;
Interp          = cfg.Link.Interp;
Fs              = MasterClockRate / Interp; %#ok<NASGU>

sps   = cfg.Link.Sps;
beta  = cfg.Link.RrcBeta;
span  = cfg.Link.RrcSpan;

preambleHalfLen = cfg.Frame.PreambleHalfLen;
payloadSyms     = cfg.Frame.PayloadSyms;

msgCapBytes  = cfg.Frame.MsgCapBytes;
pilotBitsLen = cfg.Frame.PilotBitsLen; %#ok<NASGU>

%% ---------- Modulator / Demodulator (for layout) ----------
modulator   = modulators.getMmodulator(cfg.Modulation.Name);
demodulator = demodulators.getDemodulator(cfg.Modulation.Name);   % not strictly used in TX

M   = modulator.M; %#ok<NASGU>
bps = modulator.BitsPerSymbol; %#ok<NASGU>

%% ---------- FEC (rate 1/2, K=3, TERMINATED) ----------
enc      = fec.ConvEncoder.rateHalf_K3();
K        = cfg.Fec.ConstraintLength;
Kminus1  = K - 1;

fecEncodeFcn  = @(dataBits) enc.encode(logical(dataBits), true);
fecDecodeDummy = @(codedBits) error('Payload.decode is not used in TX.');

%% ---------- Payload / Frame structure ----------
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

%% ---------- Protocol / FEC layout printout ----------
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

%% ---------- Datagram source (application → datagram bytes) ----------
dgramSrc = sources.DatagramSource(msgCapBytes, cfg.Tx.StreamSpecs);

%% ---------- RRC filter (streaming) ----------
txRRC = filters.RootRaisedCosineFilter(beta, span, sps);

%% ---------- Frame sizes ----------
NsymFrame = preambleLen + payloadSyms;   % symbols per frame
NsampUp   = NsymFrame * sps;             % samples per frame (pre-RRC)

fprintf('Frame: %d symbols, %d samples (upsampled)\n', NsymFrame, NsampUp);

%% ---------- Pre-encode frames into a buffer (OFFLINE) ----------
% We do full fr.encode (including convolutional FEC) only here.
% In the timed loop we will ONLY reuse these frmSyms.
%
% NBUF_FRAMES controls memory usage:
%   - each frame has NsymFrame complex samples
%   - double: 16 bytes per complex => ~16 * NsymFrame bytes per frame
%   - single: 8 bytes per complex  => ~8  * NsymFrame bytes per frame
%
% For NsymFrame=6256 and NBUF_FRAMES=1000:
%   single: 8 * 6256 * 1000 ≈ 50 MB

NBUF_FRAMES   = 1000;    % how many distinct frames to pre-encode
preFrames     = cell(NBUF_FRAMES,1);
pilotAmp      = cfg.Frame.PilotAmpOffset;

fprintf('Pre-encoding %d frames (full FEC + framing)...\n', NBUF_FRAMES);

for k = 1:NBUF_FRAMES
    [protoBytes, ~] = dgramSrc.readFrame();   % datagram bytes (header+payload)

    % Full PHY encode (includes convolutional FEC, mapping, pilot placement)
    [frmSyms_raw, ~] = fr.encode(protoBytes);

    % Add optional pilot DC offset, then store as single to save RAM
    frmSyms = pilotAmp + frmSyms_raw;
    preFrames{k} = single(frmSyms);   % store as complex single
end

fprintf('Pre-encoding done.\n');

%% ---------- Benchmark parameters ----------
NFRAMES_BENCH = 10000;   % frames processed in the TIMED loop

fprintf('Benchmarking %d frames using %d pre-encoded frames (circular reuse)...\n', ...
        NFRAMES_BENCH, NBUF_FRAMES);

%% ---------- Timed run (ONLY upsample + RRC) ----------
tic;
for n = 1:NFRAMES_BENCH
    % Circular index into pre-encoded frames
    idx = mod(n-1, NBUF_FRAMES) + 1;
    frmSyms = preFrames{idx};   % complex single

    % Upsample
    up = complex(zeros(numel(frmSyms)*sps, 1, 'single'));
    up(1:sps:end) = frmSyms;

    % RRC filter
    txWave = txRRC.process(up); %#ok<NASGU>
end
elapsed = toc;

%% ---------- Throughput summary ----------
framesPerSec = NFRAMES_BENCH / elapsed;

% "PHY" here is at the datagram (MsgCapBytes) level,
% i.e. per-frame bytes after FEC + framing constraints, but before pilots.
phyBytesPerFrame = msgCapBytes;             % header + payload
phyRate_kBps     = (framesPerSec * phyBytesPerFrame) / 1024;

% Max application payload (datagram payload only)
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
