classdef (Abstract) AbstractSink < handle
    % AbstractSink is a class that represent where the data or wave should
    % be written to.

    properties (SetAccess = protected)
        Name
        SampleRate
    end

    methods
        function obj = AbstractSink(name, sampleRate)
            if nargin >= 1, obj.Name       = name;       else, obj.Name = ''; end
            if nargin >= 2, obj.SampleRate = sampleRate; else, obj.SampleRate = NaN; end
        end

        function delete(obj)
            try
                obj.release();
            catch
            end
        end
    end

    methods (Abstract)
        % Write the frame into the sink
        writeFrame(obj, frame, info);

        % Release hardware, files, sockets, etc.
        release(obj);
    end
end
