function rx_backend_worker(paySymQueue, ctrlQueue)
    % Rebuild config/protocol on this worker
    cfg  = phyAppConfig();

    modu = modulators.getMmodulator(cfg.Modulation.Name);
    dem  = demodulators.getDemodulator(cfg.Modulation.Name);

    enc = fec.ConvEncoder.rateHalf_K3();

    msgCapBytes  = cfg.Frame.MsgCapBytes;
    pilotBitsLen = cfg.Frame.PilotBitsLen;

    pre = protocol.Preamble.fromMSequence( ...
        modu, ...
        cfg.Frame.PreambleHalfLen, ...
        'Degree', cfg.Frame.MseqDegree, ...
        'Seed',   cfg.Frame.MseqSeed);

    fecEncodeDummy = @(dataBits) enc.encode(logical(dataBits), true);
    fecDecodeFcn   = @(codedBits) double(fec.viterbi_k3_mex(logical(codedBits)));

    pay = protocol.Payload(modu, dem, ...
                  cfg.Frame.PayloadSyms, msgCapBytes, pilotBitsLen, ...
                  fecEncodeDummy, fecDecodeFcn);

    fr        = protocol.Frame(pre, pay);
    pilotBits = fr.Payload.PilotBits;

    % Carrier loops (PLL)
    M   = modu.M;
    carSyncCoarse = sync.CPPDecisionDirectedCarrierSync( ...
        'ModulationOrder',        M, ...
        'SamplesPerSymbol',       1, ...
        'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
        'NormalizedLoopBandwidth',cfg.CarrierSync.CoarseLoopBandwidthNorm);
    carSyncFine = sync.CPPDecisionDirectedCarrierSync( ...
        'ModulationOrder',        M, ...
        'SamplesPerSymbol',       1, ...
        'DampingFactor',          cfg.CarrierSync.DampingFactor, ...
        'NormalizedLoopBandwidth',cfg.CarrierSync.FineLoopBandwidthNorm);
    carSyncNow = carSyncCoarse;
    useFine    = false;

    % Payload sink
    paySink = sinks.CppPayloadCollectorSink('NumThreads', 2);
    for kW = 1:numel(cfg.Rx.StreamWriters)
        spec = cfg.Rx.StreamWriters(kW);
        closeOnEnd = true;
        if isfield(spec, 'CloseOnEnd')
            closeOnEnd = logical(spec.CloseOnEnd);
        end
        extra = {};
        if isfield(spec, 'WorkerType')
            extra = {'WorkerType', spec.WorkerType};
        end
        paySink.registerWriter(spec.StreamId, spec.Writer, ...
                               'CloseOnEnd', closeOnEnd, extra{:});
    end

    frames = 0;
    fprintf('Back-end worker: ready to process payload batches.\n');

    OK = true;
    while OK
        % Check for stop from client (non-blocking)
        [stopMsg, hasStop] = poll(ctrlQueue, 0);
        if hasStop
            fprintf('Back-end worker: stop = %s\n', string(stopMsg));
            break;
        end

        % Wait for next batch, but with timeout so we can re-check ctrlQueue
        [job, hasJob] = poll(paySymQueue, 0.1);
        if ~hasJob
            continue;
        end
        if isempty(job)
            continue;
        end

        rxSyms_cfo   = job.PayloadSyms;
        segStartIdx  = job.SegStartIdx;
        segLen       = job.SegLen;
        frameIdxVec  = job.FrameIndex;

        % Single PLL call over the entire big chunk
        rxSyms_eq_all = carSyncNow.process(rxSyms_cfo);

        nAcc = numel(segStartIdx);
        for k = 1:nAcc
            idx0 = segStartIdx(k);
            idx1 = idx0 + segLen(k) - 1;
            rxSyms_eq = rxSyms_eq_all(idx0:idx1);

            % Phase ambiguity resolution per frame
            [rxSyms_fixed, rotIdx, errs] = dem.resolvePhaseAmbiguity(rxSyms_eq, pilotBits); %#ok<ASGLU>

            % Decode one frame
            [codedBits, payInfo] = fr.decodeFromPayload(rxSyms_fixed); %#ok<NASGU>

            frames = frames + 1;
            frameId = frameIdxVec(k);  % for labeling / debug
            paySink.writeFrame(codedBits, struct('FrameIndex', frameId));
        end

        % Switch coarse → fine PLL after some frames
        if ~useFine && frames >= cfg.CarrierSync.SwitchToFineAfterFrames
            carSyncNow = carSyncFine;
            carSyncFine.reset(carSyncCoarse.phase, carSyncCoarse.freq);
            useFine    = true;
            fprintf('Back-end worker: switched to fine carrier loop.\n');
        end
    end

    fprintf('Back-end worker: exiting main loop (frames processed: %d).\n', frames);
end
