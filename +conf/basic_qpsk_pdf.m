function cfg = phyAppConfig()

cfg.Link.fcTx            = 25e6;
cfg.Link.fcRx            = 25e6;
cfg.Link.MasterClockRate = 100e6;
cfg.Link.Interp          = 10;
cfg.Link.Decim           = 10;
cfg.Link.Fs              = cfg.Link.MasterClockRate / cfg.Link.Decim;

cfg.Link.Sps       = 4;
cfg.Link.RrcBeta   = 0.8;
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

cfg.Link.SamplesPerFrame = (2*cfg.Frame.PreambleHalfLen + cfg.Frame.PayloadSyms) * 20 ...
    * cfg.Link.Sps;

cfg.Frame.PilotAmpOffset  = 0.1;

cfg.Agc.AveragingLength    = 1000;
cfg.Agc.MaximumGain_dB     = 30;
cfg.Agc.AdaptationStepSize = 1e-3;
cfg.Agc.TargetPower        = 1.0;

cfg.CarrierSync.DampingFactor           = 0.9;
cfg.CarrierSync.CoarseLoopBandwidthNorm = 0.03;
cfg.CarrierSync.FineLoopBandwidthNorm   = 0.01;
cfg.CarrierSync.SwitchToFineAfterFrames = 35;

cfg.PreambleDetector.MetricThreshold = 0.2;
cfg.PreambleDetector.MinWindowPower  = 5e-3;

cfg.Cfo.SuperCoarseBuffLen   = 16384;
cfg.Cfo.TrackAlpha           = 0.01;
cfg.Cfo.MaxJumpHz            = 200;
cfg.Cfo.MetricTrustThreshold = 0.24;

cfg.SDR.TxIPAddress = '192.168.10.5';
cfg.SDR.RxIPAddress = '192.168.10.4';

cfg.Tx.StreamSpecs = struct([]);

fileName     = 'U:\Chalmers\MCC125\codes\shirzad\hoho.pdf';
baseStream   = 0;
baseFileId   = 1;
maxDataBytes = 700;
numStreams   = 4;
loop         = false;

cfg = addParallelFileStreams(cfg, fileName, baseStream, baseFileId, maxDataBytes, numStreams, loop);

cfg.Rx.StreamWriters = struct([]);

for i = 1:numStreams
    cfg.Rx.StreamWriters(i).StreamId   = uint8(i - 1);
    cfg.Rx.StreamWriters(i).Writer     = io.FileChunkWriter( ...
        filetransfer.FileAssembler(uint8(1), ...
        io.FileWriter('U:\Chalmers\MCC125\codes\shirzad\test12.rar')), 200);
    cfg.Rx.StreamWriters(i).CloseOnEnd = false;
end

end
