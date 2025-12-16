function cfg = conf1()

cfg.Link.fcTx            = 10e6;
cfg.Link.fcRx            = 10e6;
cfg.Link.MasterClockRate = 100e6;
cfg.Link.Interp          = 128;
cfg.Link.Decim           = 128;
cfg.Link.Fs              = cfg.Link.MasterClockRate / cfg.Link.Decim;

cfg.Link.Sps       = 8;
cfg.Link.RrcBeta   = 0.35;
cfg.Link.RrcSpan   = 10;
cfg.Link.TxGain_dB = 0;
cfg.Link.RxGain_dB = 0;

cfg.Modulation.Name = 'qpsk';

bps = 0;

switch cfg.Modulation.Name
    case 'qpsk'
        bps = 2;
    case '16-qam'
        bps = 4;
    case '32-qam'
        bps = 8;
end

cfg.Fec.ConstraintLength = 3;
cfg.Fec.Rate             = 1/2;
cfg.Fec.UseMexK3         = true;

ethernetPayloadLenght = 150;
numOfPayloadSyms = ethernetPayloadLenght * 8 / bps;

cfg.Frame.PreambleHalfLen = 128;
cfg.Frame.PayloadSyms     = numOfPayloadSyms;

cfg.Frame.PilotBitsLen    = 32;
cfg.Frame.MsgCapBytes     = ethernetPayloadLenght * cfg.Fec.Rate ...
    - cfg.Frame.PilotBitsLen/8;
cfg.Frame.MseqDegree      = 11;
cfg.Frame.MseqSeed        = 1001;

cfg.Link.SamplesPerFrame = (2*cfg.Frame.PreambleHalfLen + cfg.Frame.PayloadSyms) ...
    * cfg.Link.Sps + cfg.Link.Sps * 10;

cfg.Frame.PilotAmpOffset  = 0.8;

cfg.Agc.AveragingLength    = 1000;
cfg.Agc.MaximumGain_dB     = 30;
cfg.Agc.AdaptationStepSize = 1e-3;
cfg.Agc.TargetPower        = 1.0;

cfg.CarrierSync.DampingFactor           = 0.707;
cfg.CarrierSync.CoarseLoopBandwidthNorm = 0.01;
cfg.CarrierSync.FineLoopBandwidthNorm   = 0.001;
cfg.CarrierSync.SwitchToFineAfterFrames = 15;

cfg.PreambleDetector.MetricThreshold = 0.2;
cfg.PreambleDetector.MinWindowPower  = 1e-7;

cfg.Cfo.SuperCoarseBuffLen   = 16384;
cfg.Cfo.TrackAlpha           = 0.05;
cfg.Cfo.MaxJumpHz            = 200;
cfg.Cfo.MetricTrustThreshold = 0.24;

cfg.SDR.TxIPAddress = '192.168.10.5';
cfg.SDR.RxIPAddress = '192.168.10.4';

cfg.Tx.StreamSpecs = struct([]);

cfg.Tx.StreamSpecs(1).StreamId = uint8(0);
cfg.Tx.StreamSpecs(1).Reader   = io.FixedMessageReader('Hello from TX via USRP!', true);

cfg.Rx.StreamWriters = struct([]);

cfg.Rx.StreamWriters(1).StreamId   = uint8(0);
cfg.Rx.StreamWriters(1).Writer     = io.ConsoleWriter();
cfg.Rx.StreamWriters(1).CloseOnEnd = false;

end
