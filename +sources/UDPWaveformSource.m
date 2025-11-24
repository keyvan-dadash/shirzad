classdef UDPWaveformSource < sources.AbstractSource
    % UDPWaveformSource provide data from udp socket (for loop back).

    properties (SetAccess = private)
        LocalIPPort         (1,1) double {mustBeInteger}
        MaximumMessageLength(1,1) double {mustBeInteger}
        IsMessageComplex    (1,1) logical
        MessageDataType     (1,:) char
        Blocking            (1,1) logical = true
        TimeoutSeconds      (1,1) double = 1.0

        UdpReceiver
        FrameIndex         (1,1) double = 0
    end

    methods
        function obj = UDPWaveformSource(varargin)
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
                frame = complex([]);
                info = struct('IsValid',false, ...
                              'NumSamples',0, ...
                              'FrameIndex',obj.FrameIndex);
                return;
            end

            frame = data(:);
            obj.FrameIndex = obj.FrameIndex + 1;

            info = struct('IsValid',true, ...
                          'NumSamples',numel(frame), ...
                          'FrameIndex',obj.FrameIndex);
        end

        function reset(obj)
            obj.release();
            obj.createReceiver();
            obj.FrameIndex = 0;
        end

        function release(obj)
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
            obj.UdpReceiver = dsp.UDPReceiver( ...
                'LocalIPPort',          obj.LocalIPPort, ...
                'IsMessageComplex',     obj.IsMessageComplex, ...
                'MessageDataType',      obj.MessageDataType, ...
                'MaximumMessageLength', obj.MaximumMessageLength);

            % It is better to setup receiver.
            setup(obj.UdpReceiver);
        end
    end
end
