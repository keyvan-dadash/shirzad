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
cfg.Link.fcRx            = 25e6;    % RX center frequency (can be offset)
cfg.Link.MasterClockRate = 100e6;
cfg.Link.Interp          = 12;
cfg.Link.Decim           = 12;
cfg.Link.Fs              = cfg.Link.MasterClockRate / cfg.Link.Decim;  % RX Fs

cfg.Link.Sps       = 4;            % samples per symbol
cfg.Link.RrcBeta   = 0.8;          % RRC roll-off
cfg.Link.RrcSpan   = 10;            % RRC span (symbols)
cfg.Link.TxGain_dB = 0;
cfg.Link.RxGain_dB = 1;

%% ---------- Modulation ----------
% Name is used with getMmodulator/getDemodulator
% Supported by your helper: 'qpsk', '16-qam', '32-qam'
cfg.Modulation.Name = 'qpsk';

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
cfg.CarrierSync.DampingFactor           = 0.9;
cfg.CarrierSync.CoarseLoopBandwidthNorm = 0.02;   % normalized to symbol rate
cfg.CarrierSync.FineLoopBandwidthNorm   = 0.008;
cfg.CarrierSync.SwitchToFineAfterFrames = 35;    % after N good frames

%% ---------- Preamble detector (Schmidl & Cox-style) ----------
cfg.PreambleDetector.MetricThreshold = 0.2;
cfg.PreambleDetector.MinWindowPower  = 5e-3;

%% ---------- CFO tracking ----------
cfg.Cfo.SuperCoarseBuffLen   = 16384;   % samples for FFT-based super-coarse CFO
cfg.Cfo.TrackAlpha           = 0.1;    % IIR smoothing for CFO
cfg.Cfo.MaxJumpHz            = 200;     % limit per update
cfg.Cfo.MetricTrustThreshold = 0.22;    % only update CFO if metric >= this

%% ---------- SDR IPs ----------
cfg.SDR.TxIPAddress = '192.168.10.5';
cfg.SDR.RxIPAddress = '192.168.10.4';

%% ---------- Application / Streams (TX side) ----------
% You can add multiple streams here (round-robin DatagramSource).
% Each entry must have fields:
%   StreamId (uint8), Reader (io.Reader subclass)

cfg.Tx.StreamSpecs = struct([]);

% Example: Stream 0 sends a repeating text message via FixedMessageReader
cfg.Tx.StreamSpecs(1).StreamId = uint8(0);
cfg.Tx.StreamSpecs(1).Reader   = io.FixedMessageReader('Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed dictum augue sed lectus finibus tempor. Nulla eros risus, congue sit amet arcu vitae, porttitor molestie ipsum. Orci varius natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Nunc iaculis eget ligula non consectetur. Curabitur lacus turpis, molestie cursus pellentesque non, scelerisque eget dui. Morbi vel malesuada odio, vitae lacinia urna. Ut iaculis neque eu blandit dignissim. Mauris pretium lacus metus, in euismod dui pulvinar nec. Nulla placerat auctor diam, vel dignissim erat ultricies vitae. Vestibulum malesuada neque leo, eu mattis dui eleifend id. Morbi vel commodo justo, quis volutpat lacus. Aliquam mollis nunc ante, maximus tristique eu.', true);

%% ---------- Application / Streams (RX side) ----------
% Each entry for RX side must have:
%   StreamId (uint8), Writer (io.Writer subclass), CloseOnEnd (logical)
%
% You can register e.g. file writers, video decoders, etc., here.

cfg.Rx.StreamWriters = struct([]);

cfg.Rx.StreamWriters(1).StreamId   = uint8(0);
cfg.Rx.StreamWriters(1).Writer = io.ConsoleWriter();
cfg.Rx.StreamWriters(1).CloseOnEnd = false;

end
