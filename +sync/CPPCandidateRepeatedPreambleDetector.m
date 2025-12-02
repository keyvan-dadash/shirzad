classdef CPPCandidateRepeatedPreambleDetector < handle
    % CandidateRepeatedPreambleDetector is a preamble detector that implements
    % Schmidl & Cox technique. This technique forms a preamble that has two
    % repeated preamble: [a, a]. Moreover, this class is capable of
    % detecting multiple premable and pick the best sample from them by using c++ backend. 
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
        function obj = CPPCandidateRepeatedPreambleDetector(varargin)
            % Constructor with name-value pairs
            if mod(numel(varargin),2) ~= 0
                error('CPPCandidateRepeatedPreambleDetector:NameValue', ...
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
                        error('CPPCandidateRepeatedPreambleDetector:UnknownParam', ...
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
            candList = sync.schmidlCoxDetectMex( ...
                y, ...
                obj.SamplesPerSymbol, ...
                obj.PreambleHalfLen, ...
                obj.MetricThreshold, ...
                obj.MinWindowPower);
        end
    end
end
