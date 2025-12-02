function test_payload_worker_helloworld()
    import protocol.Datagram
    import fec.ConvEncoder

    cleanupObj = onCleanup(@() safeShutdown());

    sinks.payload_worker_mex('init', 1);

    sinks.payload_worker_mex('add_worker', 1, 'console');

    payload  = uint8('hello world 382749');
    streamId = uint8(1);

    dg = protocol.Datagram(streamId, payload);

    dgBytes = dg.toBytes();   % uint8 column

    bitsMat  = de2bi(dgBytes, 8, 'left-msb');
    infoBits = bitsMat.';
    infoBits = infoBits(:);
    infoBits = double(infoBits ~= 0);

    enc = ConvEncoder.rateHalf_K3();
    enc.reset();
    codedBits = enc.encode(infoBits, true);

    codedBits_u8 = uint8(codedBits(:) ~= 0);

    sinks.payload_worker_mex('enqueue', codedBits_u8);

    pause(0.5);

    fprintf('test_payload_worker_helloworld: done. Check C++ console/log.\n');
end

function safeShutdown()
    try
        sinks.payload_worker_mex('shutdown');
    catch
        % ignore
    end
end