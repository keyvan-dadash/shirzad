function cfg = addParallelFileStreams(cfg, fileName, baseStreamId, baseFileId, maxDataBytes, numStreams, loop)
    if nargin < 7
        loop = false;
    end

    info = dir(fileName);
    if isempty(info)
        error('addParallelFileStreams:FileNotFound', ...
              'File "%s" not found.', fileName);
    end
    fileSize = info.bytes;

    segSize = ceil(double(fileSize) / numStreams);

    specs = cfg.Tx.StreamSpecs;
    if isempty(specs)
        specs = struct('StreamId', {}, 'Reader', {});
    end

    for k = 1:numStreams
        segStart = (k-1) * segSize;
        segEnd   = min(k * segSize, double(fileSize));

        if segStart >= segEnd
            continue;
        end

        streamId = uint8(baseStreamId + (k-1));
        fileId   = uint8(baseFileId + (k-1));   % different fileId per segment

        reader = io.FileChunkReaderSegment( ...
            fileName, fileId, maxDataBytes, ...
            segStart, segEnd, loop);

        idx = numel(specs) + 1;
        specs(idx).StreamId = streamId;
        specs(idx).Reader   = reader;
    end

    cfg.Tx.StreamSpecs = specs;
end
