classdef (Abstract) AbstractSource < handle
    %ABSTRACTSOURCE Common interface for all sources in the TX/RX chain.
    %
    % A Source produces frames of data every time readFrame() is called.
    % Examples:
    %   - Random payload bits
    %   - Baseband samples from an SDR
    %   - Video frames (future)
    %
    % The idea is:
    %   [frame, info] = src.readFrame();
    %
    % where 'frame' is a column vector or matrix holding this frame's data,
    % and 'info' is a struct with metadata (e.g. IsValid, Overrun, etc.).

    properties (SetAccess = protected)
        Name        % Human-readable name
        SampleRate  % [Hz] for sample-producing sources (NaN if not applicable)
        FrameLength % Number of samples/bits per frame, if fixed (NaN if variable)
    end

    methods
        function obj = AbstractSource(name, sampleRate, frameLength)
            if nargin >= 1, obj.Name        = name;        else, obj.Name = ''; end
            if nargin >= 2, obj.SampleRate  = sampleRate;  else, obj.SampleRate  = NaN; end
            if nargin >= 3, obj.FrameLength = frameLength; else, obj.FrameLength = NaN; end
        end

        function delete(obj)
            % Attempt to release resources automatically when cleared.
            try
                obj.release();
            catch
                % Swallow errors during cleanup
            end
        end
    end

    methods (Abstract)
        % Produce one frame of data.
        %  frame : column vector (bits, samples, etc.)
        %  info  : struct, must have at least field 'IsValid' (true/false)
        [frame, info] = readFrame(obj);

        % Reset internal state (counters, RNG, etc.)
        reset(obj);

        % Release hardware handles, file handles, sockets, etc.
        release(obj);
    end
end
