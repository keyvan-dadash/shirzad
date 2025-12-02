function test_payload_worker_file_roundtrip()
    import protocol.Datagram
    import fec.ConvEncoder
    import io.FileChunkReader

    % some test files
    inFile  = fullfile(pwd, 'payload_worker_in.bin');
    outFile = fullfile(pwd, 'file_1');

    % Clean up files + shutdown backend on exit
    cleanupObj = onCleanup(@() cleanup(inFile, outFile));

    data = uint8(mod(0:9999, 256));
    fid = fopen(inFile, 'w');
    assert(fid > 0, 'Failed to open input file for writing.');
    fwrite(fid, data, 'uint8');
    fclose(fid);

    sinks.payload_worker_mex('init', 2);
    sinks.payload_worker_mex('add_worker', 2, 'file');

    maxDataBytes = 256;
    fileId       = uint8(1);

    reader = FileChunkReader(inFile, fileId, maxDataBytes, false);

    enc = ConvEncoder.rateHalf_K3();

    streamId = uint8(2);

    while true
        maxPayloadBytes = 255;
        [miniPayload, count, eof] = reader.read(maxPayloadBytes);

        if eof && count == 0
            break;
        end

        dg = Datagram(streamId, miniPayload);

        dgBytes = dg.toBytes();

        bitsMat  = de2bi(dgBytes, 8, 'left-msb');
        infoBits = bitsMat.';
        infoBits = infoBits(:);
        infoBits = double(infoBits ~= 0);

        enc.reset();
        codedBits = enc.encode(infoBits, true);
        codedBits_u8 = uint8(codedBits(:) ~= 0);

        sinks.payload_worker_mex('enqueue', codedBits_u8);

        if eof
            break;
        end
    end

    t0 = tic;
    timeoutSec = 10;
    while ~exist(outFile, 'file') && toc(t0) < timeoutSec
        pause(0.1);
    end

    assert(exist(outFile, 'file') == 2, ...
        'Reassembled file did not appear at "%s" within timeout.', outFile);

    inBytes  = readAllBytes(inFile);
    outBytes = readAllBytes(outFile);

    assert(isequal(inBytes, outBytes), ...
        'Reassembled file bytes do not match original input.');

    fprintf('test_payload_worker_file_roundtrip: PASS (files identical).\n');
end

function b = readAllBytes(fname)
    fid = fopen(fname, 'r');
    assert(fid > 0, 'Failed to open file "%s".', fname);
    cleaner = onCleanup(@() fclose(fid));
    b = fread(fid, Inf, '*uint8');
end

function cleanup(inFile, outFile)
    try
        sinks.payload_worker_mex('shutdown');
    catch
    end

    % Remove test files
    if exist(inFile, 'file') == 2
        delete(inFile);
    end
    if exist(outFile, 'file') == 2
        delete(outFile);
    end
end
