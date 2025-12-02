classdef CandidateRepeatedPreambleDetector < handle
    % CandidateRepeatedPreambleDetector is a preamble detector that implements
    % Schmidl & Cox technique. This technique forms a preamble that has two
    % repeated preamble: [a, a]. Moreover, this class is capable of
    % detecting multiple premable and pick the best sample from them.
    %
    % Base on the M(k), we can detect the preamble of a frame. Before that,
    % we need to calculate P(d) which is:
    %   P(d) = r[d + m]*r[d + m + L] (from m = 0 until m = L - 1)
    % Also we need to calculate the energy of received signal on that
    % window, which is:
    %   R(d) = |r[d + m + K]| ^ 2 (from m = 0 until m = L - 1)
    %
    % Therefore, M(d) is:
    %   M(d) = |P(d)| ^ 2 / R(d) ^ 2
    %
    % Based on our calculation M(d) in this project is aournd 0.25 (and
    % also other projects)

    properties
        SamplesPerSymbol = 10;
        PreambleHalfLen  = 32;
        MetricThreshold  = 0.2;
        MinWindowPower   = 1e-6;
    end

    properties
        wLh
        wLh2
    end

    methods
        function obj = CandidateRepeatedPreambleDetector(varargin)
            % Constructor with name-value pairs
            if mod(numel(varargin),2) ~= 0
                error('CandidateRepeatedPreambleDetector:NameValue', ...
                      'Constructor expects name-value pairs.');
            end
            for k = 1:2:numel(varargin)
                name  = varargin{k};
                value = varargin{k+1};
                switch lower(name)
                    case 'samplespersymbol'
                        obj.SamplesPerSymbol = value;
                    case 'preamblehalflen'
                        obj.PreambleHalfLen  = value;
                        obj.wLh = ones(obj.PreambleHalfLen, 1);
                        obj.wLh2 = ones(2*obj.PreambleHalfLen, 1);
                    case 'metricthreshold'
                        obj.MetricThreshold  = value;
                    case 'minwindowpower'
                        obj.MinWindowPower   = value;
                    otherwise
                        error('CandidateRepeatedPreambleDetector:UnknownParam', ...
                              'Unknown parameter "%s".', name);
                end
            end
        end

        function res = detect(obj, y)
            candList = obj.detectCandidates(y);

            if isempty(candList)
                res = struct( ...
                    'StartSample',      0, ...
                    'SampleOffset',     0, ...
                    'PreambleStartSym', 0, ...
                    'Metric',           0, ...
                    'WindowPower',      0, ...
                    'CfoRadPerSym',     0, ...
                    'Found',            false);
                return;
            end

            [~, idx] = max([candList.Metric]);
            c        = candList(idx);
            c.Found  = true;
            res      = c;
        end

        function candList = detectCandidates(obj, y)
            sps  = obj.SamplesPerSymbol;
            Lh   = obj.PreambleHalfLen;
            Lpre = 2 * Lh;

            candRaw = struct('StartSample',{}, ...
                              'SampleOffset',{}, ...
                              'PreambleStartSym',{}, ...
                              'Metric',{}, ...
                              'WindowPower',{}, ...
                              'CfoRadPerSym',{});

            candList = candRaw; % default empty

            if isempty(y)
                return;
            end

            y = y(:);
            N = numel(y);

            for off = 0:(sps-1)
                ySym = y(1+off : sps : end);
                Ns   = numel(ySym);
        
                if Ns < 2*Lh + 1
                    continue;
                end
        
                Lwin = Ns - 2*Lh;
                if Lwin <= 0
                    continue;
                end
        
                % Schmidl & Cox core
                q = ySym(1:Ns-Lh) .* conj(ySym(1+Lh:Ns));
                Pfull = filter(obj.wLh, 1, q);
                P = Pfull(Lh : Lh+Lwin-1);
        
                pow   = abs(ySym).^2;
                Rfull = filter(obj.wLh2, 1, pow);
                R = Rfull(2*Lh : 2*Lh+Lwin-1);
        
                M = abs(P).^2 ./ (R.^2 + eps);
        
                % Candidates where metric and power are above thresholds
                mask = (M > obj.MetricThreshold) & (R > obj.MinWindowPower);
                idx  = find(mask);
                if isempty(idx)
                    continue;
                end
        
                for ii = 1:numel(idx)
                    k = idx(ii);
        
                    % keep only local maxima
                    if k > 1 && M(k) <= M(k-1)
                        continue;
                    end
                    if k < Lwin && M(k) <= M(k+1)
                        continue;
                    end
        
                    startSample = 1 + off + (k-1)*sps;
        
                    Pbest = P(k);
                    phi   = angle(Pbest);
        
                    c = struct();
                    c.StartSample      = startSample;
                    c.SampleOffset     = off;
                    c.PreambleStartSym = k;
                    c.Metric           = M(k);
                    c.WindowPower      = R(k);
                    c.CfoRadPerSym     = phi / Lh;
        
                    candRaw(end+1) = c;
                end
            end

            if isempty(candRaw)
                return;
            end
        
            % Sort by time
            [~, order] = sort([candRaw.StartSample]);
            candRaw = candRaw(order);
        
            merged = struct('StartSample',{}, ...
                            'SampleOffset',{}, ...
                            'PreambleStartSym',{}, ...
                            'Metric',{}, ...
                            'WindowPower',{}, ...
                            'CfoRadPerSym',{});

            distance = zeros(size(candRaw));
            for i = 1:numel(candRaw)
                if i == 1
                    distance(1) = 0;
                    continue;
                end
                distance(i) = candRaw(i).StartSample - candRaw(i-1).StartSample;
            end

            cnt = 0;
            dist = 0;
            index = 1;
            for i = 1:numel(candRaw)
                cnt = cnt + 1;
                dist = dist + distance(i);
                if dist > Lpre/2
                    % the distance of this candidate is more than half
                    % premable comparaed to last candidate.
                    if cnt == 1
                        dist = 0;
                        cnt = 0;
                        merged(index) = candRaw(idx);
                        index = index + 1;
                        continue;
                    end

                    % we have to find best among the the cnt we have
                    samePreCandidates = candRaw(i - cnt + 1:i);
                    allMetrics = [samePreCandidates.Metric];
                    [maxVal, idx] = max(allMetrics);
                    merged(index) = candRaw(idx);
                    index = index + 1;
                end
            end

            if cnt > 0
                samePreCandidates = candRaw(end - cnt + 1:end);
                allMetrics = [samePreCandidates.Metric];
                [maxVal, idx] = max(allMetrics);
                merged(index) = candRaw(idx);
                index = index + 1;
            end
    
            candList = merged;
        end
    end
end
