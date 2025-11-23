classdef (Abstract) AbstractPreambleDetector < handle
    % AbstractPreambleDetector specifies apis that preamble detectors
    % should implemenet. Upon calling the detect function, the result is
    % depend on the type of preamble detector.

    properties (SetAccess = protected)
        Name              (1,:) char
        SamplesPerSymbol  (1,1) double {mustBeInteger}
        PreambleLen   (1,1) double {mustBeInteger}
    end

    methods
        function obj = AbstractPreambleDetector(name, sps, preambleLen)
            if nargin < 1, name = 'AbstractPreambleDetector'; end
            if nargin < 2, sps = 1; end
            if nargin < 3, preambleLen = 64; end

            obj.Name            = char(name);
            obj.SamplesPerSymbol = sps;
            obj.PreambleLen  = preambleLen;
        end
    end

    methods (Abstract)
        result = detect(obj, yBuf)
    end

end
