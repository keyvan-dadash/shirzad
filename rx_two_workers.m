function [fFront, fBack, ctrlQueue] = rx_two_workers
    % RX_TWO_WORKERS
    %   Starts a 2-process pool.
    %   Backend worker creates queues and runs rx_backend_worker.
    %   Frontend worker runs rx_frontend_worker and sends batches to backend.
    %
    %   Usage:
    %       [fFront, fBack, ctrlQueue] = rx_two_workers;
    %       % ... later, to stop:
    %       send(ctrlQueue, 'stop');   % graceful backend stop
    %       cancel(fFront);            % stop frontend worker
    %
    %   Or:
    %       delete(gcp('nocreate'));   % hard stop (kills whole pool)

    % --- Ensure 2-process pool ---
    pool = gcp('nocreate');
    if isempty(pool) || pool.NumWorkers < 2
        if ~isempty(pool)
            delete(pool);
        end
        pool = parpool("threads", 2);
    end

    % Helper queue created on CLIENT, used only to receive
    % the worker-owned queues from the backend worker.
    helperQ = parallel.pool.PollableDataQueue;

    % Start backend worker. It will:
    %   - create paySymQueue & ctrlQueue on that worker
    %   - send them back via helperQ
    fBack = parfeval(@backend_entry, 0, helperQ);

    % Wait until backend sends us both queues
    qStruct = poll(helperQ, Inf);  % blocks until data
    paySymQueue = qStruct.paySymQueue;   % created on backend worker
    ctrlQueue   = qStruct.ctrlQueue;     % created on backend worker

    % Start frontend, giving it the queue it will send to
    fFront = parfeval(@rx_frontend_worker, 0, paySymQueue);

    % Optionally drop handles into base workspace for convenience
    assignin('base', 'fFront',     fFront);
    assignin('base', 'fBack',      fBack);
    assignin('base', 'ctrlQueue',  ctrlQueue);
    assignin('base', 'paySymQueue', paySymQueue);

    fprintf('\nrx_two_workers: front-end and back-end running.\n');
    fprintf('  In base workspace you now have: fFront, fBack, ctrlQueue, paySymQueue.\n');
    fprintf('  To stop backend cleanly:   send(ctrlQueue, ''stop'');\n');
    fprintf('  To cancel frontend:        cancel(fFront);\n');
    fprintf('  To kill whole pool:        delete(gcp(''nocreate''));\n');
end

function backend_entry(helperQ)
    % This executes on the BACK-END worker.

    % Queues are created here => this worker is the receiver.
    paySymQueue = parallel.pool.PollableDataQueue;
    ctrlQueue   = parallel.pool.PollableDataQueue;

    % Send handles back to client so it can pass them around.
    send(helperQ, struct( ...
        'paySymQueue', paySymQueue, ...
        'ctrlQueue',   ctrlQueue));

    % Now run the actual backend logic
    rx_backend_worker(paySymQueue, ctrlQueue);
end
