function cfg = phyAppConfig()
%PHYAPPCONFIG  Shared PHY + application configuration for TX & RX.
%
%   cfg = phyAppConfig();
%
% Edit this file to change:
%   - modulation type (QPSK / 16-QAM / 32-QAM, ...)
%   - frame layout (preamble, payload symbols, pilot bits, datagram size)
%   - AGC, CFO, carrier sync tuning
%   - SDR IPs, gains
%   - application readers/writers per StreamId

%% ---------- Link / USRP params ----------
cfg.Link.fcTx            = 25e6;     % TX center frequency
cfg.Link.fcRx            = 25e6 + 1e6;    % RX center frequency (can be offset)
cfg.Link.MasterClockRate = 100e6;
cfg.Link.Interp          = 12;
cfg.Link.Decim           = 12;
cfg.Link.Fs              = cfg.Link.MasterClockRate / cfg.Link.Decim;  % RX Fs

cfg.Link.Sps       = 4;            % samples per symbol
cfg.Link.RrcBeta   = 0.8;          % RRC roll-off
cfg.Link.RrcSpan   = 10;            % RRC span (symbols)
cfg.Link.TxGain_dB = 0;
cfg.Link.RxGain_dB = 0;

%% ---------- Modulation ----------
% Name is used with getMmodulator/getDemodulator
% Supported by your helper: 'qpsk', '16-qam', '32-qam'
cfg.Modulation.Name = '16-qam';

bps = 0;

switch cfg.Modulation.Name
    case 'qpsk'
        bps = 2;
    case '16-qam'
        bps = 4;
    case '64-qam'
        bps = 6;
    case '256-qam'
        bps = 8;
end

%% ---------- FEC (Conv, rate 1/2, K=3) ----------
cfg.Fec.ConstraintLength = 3;       % K
cfg.Fec.Rate             = 1/2;
cfg.Fec.UseMexK3         = true;    % RX: try fec.viterbi_k3_mex if available

%% ---------- Frame / protocol layout ----------

% Ethernet uses 1500 bytes payload
cfg.ethernetPayloadLenght = 1500;
numOfPayloadSyms = cfg.ethernetPayloadLenght * 8 / bps;

cfg.Frame.PreambleHalfLen = 128;    % half-length (a), total [a a] = 256
cfg.Frame.PayloadSyms     = numOfPayloadSyms;    % number of payload symbols per frame

totalSym = numOfPayloadSyms + 2 * cfg.Frame.PreambleHalfLen;
fprintf('total symbol cnt and samples: %d and %d\n', totalSym, totalSym * cfg.Link.Sps);

cfg.Frame.PilotBitsLen    = 16;    % number of known pilot bits
cfg.Frame.MsgCapBytes     = cfg.ethernetPayloadLenght * cfg.Fec.Rate ...
    - cfg.Frame.PilotBitsLen/8;     % datagram bytes (header+payload)
cfg.Frame.MseqDegree      = 11;      % for m-sequence preamble
cfg.Frame.MseqSeed        = 1001;   % RNG seed used in training generator

cfg.Link.SamplesPerFrame = (2*cfg.Frame.PreambleHalfLen + cfg.Frame.PayloadSyms) * 64 ...
    * cfg.Link.Sps;    % SDRu RX frame size

cfg.Frame.PilotAmpOffset  = 0.1;    % DC offset added to frame symbols (TX)

%% ---------- AGC ----------
cfg.Agc.AveragingLength    = 1000;
cfg.Agc.MaximumGain_dB     = 30;
cfg.Agc.AdaptationStepSize = 1e-3;
cfg.Agc.TargetPower        = 1.0;

%% ---------- Carrier / phase sync (DecisionDirectedCarrierSync) ----------
cfg.CarrierSync.DampingFactor           = 0.4;
cfg.CarrierSync.CoarseLoopBandwidthNorm = 0.01;   % normalized to symbol rate
cfg.CarrierSync.FineLoopBandwidthNorm   = 0.001;
cfg.CarrierSync.SwitchToFineAfterFrames = 20;    % after N good frames

%% ---------- Preamble detector (Schmidl & Cox-style) ----------
cfg.PreambleDetector.MetricThreshold = 0.2;
cfg.PreambleDetector.MinWindowPower  = 5e-3;

%% ---------- CFO tracking ----------
cfg.Cfo.SuperCoarseBuffLen   = 16384;   % samples for FFT-based super-coarse CFO
cfg.Cfo.TrackAlpha           = 0.1;    % IIR smoothing for CFO
cfg.Cfo.MaxJumpHz            = 200;     % limit per update
cfg.Cfo.MetricTrustThreshold = 0.24;    % only update CFO if metric >= this

%% ---------- SDR IPs ----------
cfg.SDR.TxIPAddress = '192.168.10.5';
cfg.SDR.RxIPAddress = '192.168.10.4';

%% ---------- Application / Streams (TX side) ----------
% You can add multiple streams here (round-robin DatagramSource).
% Each entry must have fields:
%   StreamId (uint8), Reader (io.Reader subclass)

cfg.Tx.StreamSpecs = struct([]);

% Example: Stream 0 sends a repeating text message via FixedMessageReader
% cfg.Tx.StreamSpecs(1).StreamId = uint8(0);
% cfg.Tx.StreamSpecs(1).Reader   = io.FixedMessageReader('Hello from TX via USRP!', true);

% cfg.Tx.StreamSpecs(1).Reader = io.FileChunkReader(...
%     'U:\Chalmers\MCC125\codes\shirzad\test1.rar', ...
%     uint8(1), 700, true);

fileName    = 'U:\Chalmers\MCC125\codes\shirzad\hoho.pdf';
% fileName    = 'U:\Chalmers\MCC125\codes\shirzad\test1.rar';
baseStream  = 0;      % streams will be 0,1,2,3
baseFileId  = 1;      % fileIds will be 1,2,3,4
maxDataBytes = 700;
numStreams   = 4;
loop         = false; % no looping over file

cfg = addParallelFileStreams(cfg, fileName, baseStream, baseFileId, maxDataBytes, numStreams, loop);

%% ---------- Application / Streams (RX side) ----------
% Each entry for RX side must have:
%   StreamId (uint8), Writer (io.Writer subclass), CloseOnEnd (logical)
%
% You can register e.g. file writers, video decoders, etc., here.

cfg.Rx.StreamWriters = struct([]);

% Example: Stream 0 -> console output
for i = 1:numStreams
    cfg.Rx.StreamWriters(i).StreamId   = uint8(i - 1);
    cfg.Rx.StreamWriters(i).Writer     = io.FileChunkWriter(...
        filetransfer.FileAssembler(uint8(1), ...
        io.FileWriter('U:\Chalmers\MCC125\codes\shirzad\test12.rar')), 200);
    % cfg.Rx.StreamWriters(1).Writer = io.ConsoleWriter();
    cfg.Rx.StreamWriters(i).CloseOnEnd = false;
end

end
