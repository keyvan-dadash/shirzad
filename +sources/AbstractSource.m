classdef (Abstract) AbstractSource < handle
    % AbstractSource represent a source that data (or wave) will come to
    % the system.

    properties (SetAccess = protected)
        Name
        SampleRate
        FrameLength
    end

    methods
        function obj = AbstractSource(name, sampleRate, frameLength)
            if nargin >= 1, obj.Name        = name;        else, obj.Name = ''; end
            if nargin >= 2, obj.SampleRate  = sampleRate;  else, obj.SampleRate  = NaN; end
            if nargin >= 3, obj.FrameLength = frameLength; else, obj.FrameLength = NaN; end
        end

        function delete(obj)
            try
                obj.release();
            catch
            end
        end
    end

    methods (Abstract)
        % Reading the frame from the source
        [frame, info] = readFrame(obj);

        % Reset the object (rarely used)
        reset(obj);

        % Release hardware handles, file handles, sockets, etc.
        release(obj);
    end
end
