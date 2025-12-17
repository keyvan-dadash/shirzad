clear; clc;

assert(exist('dsp.UDPReceiver','class')==8, ...
    'Install "DSP System Toolbox" for UDP/USRP support.');

cfg = phyAppConfig();

fc              = cfg.Link.fcRx;
MasterClockRate = cfg.Link.MasterClockRate;
Decim           = cfg.Link.Decim;
Fs              = MasterClockRate/Decim;

SamplesPerFrame = cfg.Link.SamplesPerFrame;
rxGain_dB       = cfg.Link.RxGain_dB;

rfSrc = sources.SDRuBasebandSource( ...
    'IPAddress',          cfg.SDR.RxIPAddress, ...
    'CenterFrequency',    fc, ...
    'MasterClockRate',    MasterClockRate, ...
    'DecimationFactor',   Decim, ...
    'Gain',               rxGain_dB, ...
    'SamplesPerFrame',    SamplesPerFrame, ...
    'TransportDataType',  'int8', ...
    'OutputDataType',     'single');

% --------- Output files ----------
ts       = datestr(now, 'yyyymmdd_HHMMSS');
binFile  = fullfile(pwd, ['rx_capture_' ts '.bin']);   % raw IQ float32 interleaved
metaFile = fullfile(pwd, ['rx_capture_' ts '.mat']);   % metadata only

% Capture limits
maxSeconds = 30;
printEvery = 50;

% Rough chunk estimate (safe)
maxChunksEst = ceil(maxSeconds * Fs / SamplesPerFrame) + 200;

% Metadata buffers in RAM (small)
chunkStart    = zeros(1, maxChunksEst, 'uint64');   % sample index (1-based) in the continuous stream
chunkLen      = zeros(1, maxChunksEst, 'uint32');   % number of complex samples in this chunk
chunkOverrun  = false(1, maxChunksEst);
chunkWallTime = zeros(1, maxChunksEst, 'double');   % posixtime UTC

% Open binary file
fid = fopen(binFile, 'Wb');
assert(fid > 0, 'Could not open %s for writing.', binFile);

% Ensure we clean up even on Ctrl+C / error
cleanupObj = onCleanup(@() localCleanup(rfSrc, fid));

fprintf('RX CAPTURE:\n  BIN : %s\n  META: %s\n', binFile, metaFile);
fprintf('Fs=%.3f Hz | SamplesPerFrame=%d | maxSeconds=%d | maxChunksEst=%d\n', ...
    Fs, SamplesPerFrame, maxSeconds, maxChunksEst);
fprintf('Ctrl+C to stop.\n');

chunkIdx     = uint64(0);
writePos     = uint64(1);      % 1-based sample position
totalSamples = uint64(0);
tStart       = tic;

try
    while true
        if toc(tStart) >= maxSeconds
            fprintf('Stopping: reached maxSeconds.\n');
            break;
        end

        [xRaw, len, over] = rfSrc.readFrame();

        chunkIdx = chunkIdx + 1;
        if chunkIdx > maxChunksEst
            fprintf('Stopping: reached maxChunksEst.\n');
            break;
        end

        xRaw = xRaw(:);

        % Use len if valid, else fallback
        if isempty(len) || ~isscalar(len) || len <= 0
            N = uint32(numel(xRaw));
        else
            N = uint32(min(double(len), double(numel(xRaw))));
        end

        % Record metadata
        chunkStart(1, chunkIdx)    = writePos;
        chunkLen(1, chunkIdx)      = N;
        chunkOverrun(1, chunkIdx)  = logical(over);
        chunkWallTime(1, chunkIdx) = posixtime(datetime('now','TimeZone','UTC'));

        % Write IQ as float32 interleaved: I1,Q1,I2,Q2,...
        if N > 0
            xs = xRaw(1:N);
            iq = [real(xs).'; imag(xs).'];          % 2 x N
            fwrite(fid, iq, 'float32');
        end

        writePos     = writePos + uint64(N);
        totalSamples = totalSamples + uint64(N);

        if mod(double(chunkIdx), printEvery) == 0
            fprintf('Captured chunks=%d | totalSamples=%d | lastLen=%d | lastOverrun=%d\n', ...
                double(chunkIdx), double(totalSamples), double(N), over);
        end
    end

catch ME
    fprintf('Capture stopped (exception): %s\n', ME.message);
end

% Trim metadata arrays
k = double(chunkIdx);
chunkStart    = chunkStart(1:k);
chunkLen      = chunkLen(1:k);
chunkOverrun  = chunkOverrun(1:k);
chunkWallTime = chunkWallTime(1:k);

totalChunks  = double(chunkIdx);
totalSamples = double(totalSamples);

% Save metadata
save(metaFile, ...
    'cfg','Fs','fc','SamplesPerFrame', ...
    'binFile', ...
    'chunkStart','chunkLen','chunkOverrun','chunkWallTime', ...
    'totalChunks','totalSamples', ...
    '-v7.3');

fprintf('DONE.\n  totalSamples=%d\n  totalChunks=%d\n  BIN : %s\n  MAT : %s\n', ...
    totalSamples, totalChunks, binFile, metaFile);

% -------- local cleanup --------
function localCleanup(rfSrcObj, fid)
    try
        if fid > 0
            fclose(fid);
        end
    catch
    end
    try
        if ismethod(rfSrcObj, 'release'); release(rfSrcObj); end
    catch
    end
    try
        if isvalid(rfSrcObj); delete(rfSrcObj); end
    catch
    end
end
