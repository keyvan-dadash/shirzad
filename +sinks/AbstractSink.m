classdef (Abstract) AbstractSink < handle
    %ABSTRACTSINK Common interface for all sinks in the TX/RX chain.
    %
    % A Sink consumes frames of data via writeFrame().
    % Examples:
    %   - USRP transmitter (TX side)
    %   - Payload bit logger / file writer (RX side)
    %   - UDP loopback endpoint, etc.

    properties (SetAccess = protected)
        Name        % Human-readable name
        SampleRate  % [Hz] for sample-taking sinks (NaN if not applicable)
    end

    methods
        function obj = AbstractSink(name, sampleRate)
            if nargin >= 1, obj.Name       = name;       else, obj.Name = ''; end
            if nargin >= 2, obj.SampleRate = sampleRate; else, obj.SampleRate = NaN; end
        end

        function delete(obj)
            % Attempt to release resources automatically when cleared.
            try
                obj.release();
            catch
            end
        end
    end

    methods (Abstract)
        % Consume one frame of data.
        %  frame : column vector / matrix
        %  info  : optional struct (e.g. frame index, timestamp)
        writeFrame(obj, frame, info);

        % Release hardware, files, sockets, etc.
        release(obj);
    end
end
