function cfg = phyAppConfig()

cfg.Link.fcTx            = 25e6;
cfg.Link.fcRx            = 25e6;
cfg.Link.MasterClockRate = 100e6;
cfg.Link.Interp          = 12;
cfg.Link.Decim           = 12;
cfg.Link.Fs              = cfg.Link.MasterClockRate / cfg.Link.Decim;

cfg.Link.Sps       = 4;
cfg.Link.RrcBeta   = 0.8;
cfg.Link.RrcSpan   = 10;
cfg.Link.TxGain_dB = 0;
cfg.Link.RxGain_dB = 1;

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

cfg.Fec.ConstraintLength = 3;
cfg.Fec.Rate             = 1/2;
cfg.Fec.UseMexK3         = true;

cfg.ethernetPayloadLenght = 1500;
numOfPayloadSyms = cfg.ethernetPayloadLenght * 8 / bps;

cfg.Frame.PreambleHalfLen = 128;
cfg.Frame.PayloadSyms     = numOfPayloadSyms;

totalSym = numOfPayloadSyms + 2 * cfg.Frame.PreambleHalfLen;
fprintf('total symbol cnt and samples: %d and %d\n', totalSym, totalSym * cfg.Link.Sps);

cfg.Frame.PilotBitsLen    = 16;
cfg.Frame.MsgCapBytes     = cfg.ethernetPayloadLenght * cfg.Fec.Rate ...
    - cfg.Frame.PilotBitsLen/8;
cfg.Frame.MseqDegree      = 11;
cfg.Frame.MseqSeed        = 1001;

cfg.Link.SamplesPerFrame = (2*cfg.Frame.PreambleHalfLen + cfg.Frame.PayloadSyms) * 64 ...
    * cfg.Link.Sps;

cfg.Frame.PilotAmpOffset  = 0.1;

cfg.Agc.AveragingLength    = 1000;
cfg.Agc.MaximumGain_dB     = 30;
cfg.Agc.AdaptationStepSize = 1e-3;
cfg.Agc.TargetPower        = 1.0;

cfg.CarrierSync.DampingFactor           = 0.9;
cfg.CarrierSync.CoarseLoopBandwidthNorm = 0.02;
cfg.CarrierSync.FineLoopBandwidthNorm   = 0.008;
cfg.CarrierSync.SwitchToFineAfterFrames = 35;

cfg.PreambleDetector.MetricThreshold = 0.2;
cfg.PreambleDetector.MinWindowPower  = 5e-3;

cfg.Cfo.SuperCoarseBuffLen   = 16384;
cfg.Cfo.TrackAlpha           = 0.1;
cfg.Cfo.MaxJumpHz            = 200;
cfg.Cfo.MetricTrustThreshold = 0.22;

cfg.SDR.TxIPAddress = '192.168.10.5';
cfg.SDR.RxIPAddress = '192.168.10.4';

cfg.Tx.StreamSpecs = struct([]);

cfg.Tx.StreamSpecs(1).StreamId = uint8(0);
cfg.Tx.StreamSpecs(1).Reader   = io.FixedMessageReader( ...
    'Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed dictum augue sed lectus finibus tempor. Nulla eros risus, congue sit amet arcu vitae, porttitor molestie ipsum. Orci varius natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Nunc iaculis eget ligula non consectetur. Curabitur lacus turpis, molestie cursus pellentesque non, scelerisque eget dui. Morbi vel malesuada odio, vitae lacinia urna. Ut iaculis neque eu blandit dignissim. Mauris pretium lacus metus, in euismod dui pulvinar nec. Nulla placerat auctor diam, vel dignissim erat ultricies vitae. Vestibulum malesuada neque leo, eu mattis dui eleifend id. Morbi vel commodo justo, quis volutpat lacus. Aliquam mollis nunc ante, maximus tristique eu.', ...
    true);

cfg.Rx.StreamWriters = struct([]);

cfg.Rx.StreamWriters(1).StreamId   = uint8(0);
cfg.Rx.StreamWriters(1).Writer     = io.ConsoleWriter();
cfg.Rx.StreamWriters(1).CloseOnEnd = false;

end
