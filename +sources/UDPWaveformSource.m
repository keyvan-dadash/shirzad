classdef UDPWaveformSource < sources.AbstractSource
    %UDPWAVEFORMSOURCE Source that receives complex baseband waveforms over UDP.
    %
    % This is a drop-in replacement for SDRuBasebandSource when you want
    % to loop back TX->RX over localhost.
    %
    % Usage in RX:
    %   Fs = 1e6;
    %   SamplesPerFrame = 32768; % must be <= MaximumMessageLength
    %
    %   udpRx = sources.UDPWaveformSource( ...
    %       'LocalIPPort',          31000, ...
    %       'SampleRate',           Fs, ...
    %       'MaximumMessageLength', SamplesPerFrame, ...
    %       'Blocking',             true, ...
    %       'IsMessageComplex',     true, ...
    %       'MessageDataType',      'double');
    %
    %   [x, info] = udpRx.readFrame();
    %
    % info fields:
    %   - IsValid
    %   - NumSamples
    %   - FrameIndex
    %
    % NOTE:
    %   - Uses dsp.UDPReceiver (DSP System Toolbox).
    %   - We assume complex double packets (to match the sink above).
    %   - If Blocking=true, readFrame() loops internally until data arrives
    %     (or until TimeoutSeconds, if set).

    properties (SetAccess = private)
        LocalIPPort         (1,1) double {mustBeInteger}
        MaximumMessageLength(1,1) double {mustBeInteger}
        IsMessageComplex    (1,1) logical
        MessageDataType     (1,:) char
        Blocking            (1,1) logical = true
        TimeoutSeconds      (1,1) double = 1.0    % for blocking mode

        UdpReceiver                     % dsp.UDPReceiver
        FrameIndex         (1,1) double = 0
    end

    methods
        function obj = UDPWaveformSource(varargin)
            % UDPWaveformSource('LocalIPPort',31000, ...
            %                   'SampleRate',Fs, ...
            %                   'MaximumMessageLength',32768, ...
            %                   'IsMessageComplex',true, ...
            %                   'MessageDataType','double', ...
            %                   'Blocking',true)
            p = inputParser;
            p.addParameter('LocalIPPort',          31000,       @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('SampleRate',           NaN,         @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('MaximumMessageLength', 32768,       @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('IsMessageComplex',     true,        @islogical);
            p.addParameter('MessageDataType',      'double',    @(s)ischar(s)||isstring(s));
            p.addParameter('Blocking',             true,        @islogical);
            p.addParameter('TimeoutSeconds',       1.0,         @(x)isnumeric(x)&&isscalar(x));
            p.parse(varargin{:});
            cfg = p.Results;

            obj@sources.AbstractSource('UDPWaveformSource', ...
                                       cfg.SampleRate, cfg.MaximumMessageLength);

            obj.LocalIPPort          = cfg.LocalIPPort;
            obj.MaximumMessageLength = cfg.MaximumMessageLength;
            obj.IsMessageComplex     = cfg.IsMessageComplex;
            obj.MessageDataType      = char(cfg.MessageDataType);
            obj.Blocking             = cfg.Blocking;
            obj.TimeoutSeconds       = cfg.TimeoutSeconds;

            obj.createReceiver();
        end

        function [frame, info] = readFrame(obj)
            %READFRAME Receive one UDP packet as a frame.
            %
            % frame : complex column vector (or empty if none)
            % info  : struct with IsValid, NumSamples, FrameIndex

            if isempty(obj.UdpReceiver)
                error('UDPWaveformSource:NotInitialized', ...
                      'UDP receiver object has not been created.');
            end

            data = [];
            if obj.Blocking
                t0 = tic;
                while isempty(data)
                    data = obj.UdpReceiver();
                    if ~isempty(data)
                        break;
                    end
                    if toc(t0) > obj.TimeoutSeconds
                        break;
                    end
                    pause(0.001); % throttle the loop a bit
                end
            else
                data = obj.UdpReceiver();
            end

            if isempty(data)
                frame = complex([]); %#ok<*COMPLEX>
                info = struct('IsValid',false, ...
                              'NumSamples',0, ...
                              'FrameIndex',obj.FrameIndex);
                return;
            end

            % Received vector is usually row; convert to column
            frame = data(:);
            obj.FrameIndex = obj.FrameIndex + 1;

            info = struct('IsValid',true, ...
                          'NumSamples',numel(frame), ...
                          'FrameIndex',obj.FrameIndex);
        end

        function reset(obj)
            %RESET Recreate receiver and reset counters.
            obj.release();
            obj.createReceiver();
            obj.FrameIndex = 0;
        end

        function release(obj)
            %RELEASE Release UDP receiver.
            if ~isempty(obj.UdpReceiver)
                try
                    release(obj.UdpReceiver);
                catch
                end
                obj.UdpReceiver = [];
            end
        end
    end

    methods (Access = private)
        function createReceiver(obj)
            % Internal helper to configure dsp.UDPReceiver.
            %
            % For complex doubles, we must set:
            %   IsMessageComplex = true
            %   MessageDataType  = 'double'
            % and MaximumMessageLength >= vector length. :contentReference[oaicite:1]{index=1}
            obj.UdpReceiver = dsp.UDPReceiver( ...
                'LocalIPPort',          obj.LocalIPPort, ...
                'IsMessageComplex',     obj.IsMessageComplex, ...
                'MessageDataType',      obj.MessageDataType, ...
                'MaximumMessageLength', obj.MaximumMessageLength);

            % Recommended to call setup() before first use. :contentReference[oaicite:2]{index=2}
            setup(obj.UdpReceiver);
        end
    end
end
