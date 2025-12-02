classdef EventProfiler < handle
    % EventProfiler helps with recording data events for analysing the
    % performance.

    properties (Access = private)
        % listEvents holds list of event and what time they took
        listEvents
    end

    methods
        function obj = EventProfiler()
            obj.listEvents = containers.Map('KeyType','char','ValueType','any');
        end

        function start(obj, name)
            if nargin < 2
                error('EventProfiler:start', 'Event name is required.');
            end

            key = obj.normalizeName(name);
            m   = obj.listEvents;

            if isKey(m, key)
                rec = m(key);
            else
                rec = struct( ...
                    'totalTime', 0.0, ...       % accumulated seconds
                    'count',     0,   ...       % number of completed intervals
                    'starts',    uint64([]));   % starts
            end

            t0 = tic(); 
            rec.starts(end+1, 1) = t0;

            m(key) = rec;
        end

        function stop(obj, name)
            if nargin < 2
                error('EventProfiler:stop', 'Event name is required.');
            end

            key = obj.normalizeName(name);
            m   = obj.listEvents;

            if ~isKey(m, key)
                warning('EventProfiler:stop:noSuchEvent', ...
                    'No existing event "%s" to stop.', key);
                return;
            end

            rec = m(key);

            if isempty(rec.starts)
                warning('EventProfiler:stop:emptyStack', ...
                    'Event "%s" stop() called but no corresponding start().', key);
                return;
            end

            % we should compare to the last tic
            t0 = rec.starts(end);
            rec.starts(end) = [];

            dt = toc(t0);

            rec.totalTime = rec.totalTime + dt;
            rec.count     = rec.count + 1;

            m(key) = rec;
        end

        function reset(obj)
            m = obj.listEvents;
            if ~isempty(m)
                remove(m, m.keys);
            end
        end

        function data = getData(obj)
            m = obj.listEvents;
            keys = m.keys;

            n = numel(keys);
            names      = cell(1,n);
            counts     = zeros(1,n);
            totalTimes = zeros(1,n);

            for k = 1:n
                key = keys{k};
                rec = m(key);

                names{k}      = key;
                counts(k)     = rec.count;
                totalTimes(k) = rec.totalTime;
            end

            avgTimes = zeros(1,n);
            idxNonZero = counts > 0;
            avgTimes(idxNonZero) = totalTimes(idxNonZero) ./ counts(idxNonZero);

            data = struct( ...
                'name',      {names}, ...
                'count',     counts, ...
                'totalTime', totalTimes, ...
                'avgTime',   avgTimes);
        end

        function print(obj)
            % print the recorded data

            data = obj.getData();

            names      = data.name;
            counts     = data.count;
            totalTimes = data.totalTime;
            avgTimes   = data.avgTime;

            if isempty(names)
                fprintf('EventProfiler: no events recorded.\n');
                return;
            end

            % Lets sort by heavist events
            [~, idxSort] = sort(totalTimes, 'descend');

            fprintf('EventProfiler summary:\n');
            fprintf('  %-20s  %10s  %12s  %12s\n', ...
                'Event', 'Calls', 'Avg [ms]', 'Total [s]');
            fprintf('  %s\n', repmat('-', 1, 60));

            for ii = idxSort
                name  = names{ii};
                cnt   = counts(ii);
                tTot  = totalTimes(ii);
                tAvg  = avgTimes(ii);

                fprintf('  %-20s  %10d  %12.3f  %12.3f\n', ...
                    name, cnt, tAvg*1e3, tTot);
            end
        end
    end

    methods (Access = private)
        function key = normalizeName(~, name)
            if isstring(name)
                name = char(name);
            end
            if ~ischar(name)
                error('EventProfiler:InvalidName', ...
                    'Event name must be char or string.');
            end
            key = char(name(:).');  % ensure row
        end
    end
end
