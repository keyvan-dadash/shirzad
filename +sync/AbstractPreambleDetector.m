classdef (Abstract) AbstractPreambleDetector < handle
    %ABSTRACTPREAMBLEDETECTOR Base class for preamble / frame detectors.
    %
    %   result = detect(obj, yBuf)
    %
    % yBuf   : complex column vector at *sample rate* (after RRC etc)
    %
    % result : struct with at least fields
    %   - Found            : logical
    %   - SampleOffset     : integer in [0 .. SamplesPerSymbol-1]
    %   - PreambleStartSym : index (1-based) in symbol-rate stream
    %   - CfoRadPerSym     : estimated CFO (rad/symbol)
    %   - Metric           : detection metric (scalar)
    %
    % Subclasses implement the detect() method.

    properties (SetAccess = protected)
        Name              (1,:) char
        SamplesPerSymbol  (1,1) double {mustBeInteger}
        PreambleHalfLen   (1,1) double {mustBeInteger}
    end

    methods
        function obj = AbstractPreambleDetector(name, sps, preambleHalfLen)
            if nargin < 1, name = 'AbstractPreambleDetector'; end
            if nargin < 2, sps = 1; end
            if nargin < 3, preambleHalfLen = 64; end

            obj.Name            = char(name);
            obj.SamplesPerSymbol = sps;
            obj.PreambleHalfLen  = preambleHalfLen;
        end
    end

    methods (Abstract)
        result = detect(obj, yBuf)
    end

end
