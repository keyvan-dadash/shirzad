clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, 'Install "DSP System Toolbox" for UDP/USRP support.');

cfg = phyAppConfig();

fc              = cfg.Link.fcRx;
MasterClockRate = cfg.Link.MasterClockRate;
Decim           = cfg.Link.Decim;
Fs              = MasterClockRate/Decim;

SamplesPerFrame = cfg.Link.SamplesPerFrame;

rxGain_dB       = cfg.Link.RxGain_dB;

rfSrc = sources.SDRuBasebandSource( ...
      'IPAddress',        cfg.SDR.RxIPAddress, ...
      'CenterFrequency',  fc, ...
      'MasterClockRate',  MasterClockRate, ...
      'DecimationFactor', Decim, ...
      'Gain',             rxGain_dB, ...
      'SamplesPerFrame',  SamplesPerFrame, ...
      'TransportDataType', 'int8', ...
      'OutputDataType', 'single');

% ------------------- Capture file -------------------
ts = datestr(now, 'yyyymmdd_HHMMSS');
outFile = fullfile(pwd, ['rx_capture_' ts '.mat']);

mf = matfile(outFile, 'Writable', true);

mf.cfg            = cfg;
mf.Fs             = Fs;
mf.fc             = fc;
mf.SamplesPerFrame= SamplesPerFrame;

mf.x              = complex(single(zeros(0,1)));
mf.chunkStart     = uint64([]);
mf.chunkLen       = uint32([]);
mf.chunkOverrun   = logical([]);
mf.chunkWallTime  = double([]);

maxSeconds = 30;
maxChunks  = inf;
printEvery = 50;

fprintf('RX CAPTURE: writing to %s\n', outFile);
fprintf('Ctrl+C to stop.\n');

writePos   = uint64(1);
chunkIdx   = uint64(0);
tStart     = tic;

try
    while true
        if toc(tStart) >= maxSeconds
            fprintf('Stopping: reached maxSeconds.\n');
            break;
        end
        if chunkIdx >= maxChunks
            fprintf('Stopping: reached maxChunks.\n');
            break;
        end

        [xRaw, len, over] = rfSrc.readFrame();
        chunkIdx = chunkIdx + 1;

        xRaw = xRaw(:);
        N    = uint64(numel(xRaw));

        if N > 0
            mf.x(writePos:writePos+N-1, 1) = xRaw;
        end

        mf.chunkStart(1,chunkIdx)    = writePos;
        mf.chunkLen(1,chunkIdx)      = uint32(N);
        mf.chunkOverrun(1,chunkIdx)  = logical(over);
        mf.chunkWallTime(1,chunkIdx) = posixtime(datetime('now'));

        writePos = writePos + N;

        if mod(double(chunkIdx), printEvery) == 0
            totalSamp = double(writePos-1);
            fprintf('Captured chunks=%d | samples=%d | lastOverrun=%d\n', ...
                double(chunkIdx), totalSamp, over);
        end
    end

catch ME
    fprintf('Capture stopped (exception): %s\n', ME.message);
end

mf.totalChunks  = double(chunkIdx);
mf.totalSamples = double(writePos-1);

fprintf('DONE. Saved %d samples in %d chunks to:\n  %s\n', ...
    mf.totalSamples, mf.totalChunks, outFile);
