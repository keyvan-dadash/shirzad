classdef CppPayloadCollectorSink < sinks.AbstractSink
    properties
        % If true, keep what we receive in Frames{} for debugging
        StoreHistory logical = true;
    end

    properties (SetAccess = private)
        % History (whatever is passed to writeFrame. for debugging)
        Frames       cell   = {};
        FrameLengths double = [];
        NumFrames    double = 0;
        TotalBytes   double = 0;

        % Number of worker threads to start in the C++ backend
        NumThreads   (1,1) double = 1;

        % small local map: StreamId -> workerType
        WorkerTypes
    end

    methods
        function obj = CppPayloadCollectorSink(varargin)
            obj@sinks.AbstractSink('CppPayloadCollectorSink', NaN);

            p = inputParser;
            addParameter(p, 'NumThreads', 1, @(x)isnumeric(x)&&isscalar(x)&&x>0);
            parse(p, varargin{:});

            obj.NumThreads = p.Results.NumThreads;

            % Small map to remember which stream has which worker type
            obj.WorkerTypes = containers.Map('KeyType','double', 'ValueType','char');

            % We should make sure mex exists
            if exist('utils.payload_worker_mex','file') ~= 3
                error('CppPayloadCollectorSink:MissingMex', ...
                      'utils.payload_worker_mex MEX not found on path.');
            end

            % Initialise C++ backend
            utils.payload_worker_mex('init', obj.NumThreads);
        end

        function delete(obj)
            try
                utils.payload_worker_mex('shutdown');
            catch
                % ignore
            end
        end

        function registerWriter(obj, streamId, writer, varargin)
            sid = double(streamId);

            % Parse optional args
            p = inputParser;
            addParameter(p, 'WorkerType', '', @(s)ischar(s)||isstring(s));
            % not gonna use
            addParameter(p, 'CloseOnEnd', false, @(x)islogical(x)&&isscalar(x));
            parse(p, varargin{:});

            workerType = char(p.Results.WorkerType);

            if isempty(workerType)
                % Try to infer from writer type
                if isa(writer, 'io.ConsoleWriter')
                    workerType = 'console';
                elseif isa(writer, 'io.FileWriter') || isa(writer, 'io.FileChunkWriter') ...
                        || isa(writer, 'FileAssembler') %#ok<OR2>
                    workerType = 'file';
                else
                    error('CppPayloadCollectorSink:UnknownWriterType', ...
                          ['Cannot infer worker type from writer of class "%s". ' ...
                           'Pass ''WorkerType'',''console'' or ''file'' explicitly.'], ...
                          class(writer));
                end
            end

            utils.payload_worker_mex('add_worker', sid, workerType);

            % Record locally (later)
            obj.WorkerTypes(sid) = workerType;
        end

        function unregisterWriter(obj, streamId)
            sid = double(streamId);

            % Remove from C++ backend
            try
                utils.payload_worker_mex('remove_worker', sid);
            catch ME
                warning('CppPayloadCollectorSink:RemoveWorkerError', ...
                        'Error removing worker for stream %d: %s', ...
                        sid, ME.message);
            end

            % Remove from local map if present
            if ~isempty(obj.WorkerTypes) && isKey(obj.WorkerTypes, sid)
                remove(obj.WorkerTypes, sid);
            end
        end

        function writeFrame(obj, dataBytes, info) %#ok<INUSD>
            if nargin < 3
                info = struct();
            end

            if isempty(dataBytes)
                return;
            end

            if ~isa(dataBytes, 'uint8')
                dataBytes = uint8(dataBytes ~= 0);
            else
                dataBytes = uint8(dataBytes ~= 0); % force 0/1
            end
            dataBytes = dataBytes(:);

            % Dbg info
            obj.NumFrames  = obj.NumFrames + 1;
            L              = numel(dataBytes);
            obj.TotalBytes = obj.TotalBytes + L;
            obj.FrameLengths(end+1,1) = L;

            if obj.StoreHistory
                obj.Frames{end+1,1} = dataBytes;
            end

            % Forward to C++ backend (blocking queue, should we change?)
            utils.payload_worker_mex('enqueue', dataBytes);
        end

        function bits = concatenateAll(obj)
            if ~obj.StoreHistory
                error('CppPayloadCollectorSink:NoHistory', ...
                      'History is not being stored (StoreHistory=false).');
            end
            if isempty(obj.Frames)
                bits = uint8([]);
            else
                bits = vertcat(obj.Frames{:});
            end
        end

        function reset(obj)
            obj.Frames       = {};
            obj.FrameLengths = [];
            obj.NumFrames    = 0;
            obj.TotalBytes   = 0;
        end

        function release(obj)
            try
                utils.payload_worker_mex('shutdown');
            catch
                % ignore
            end
        end
    end
end
