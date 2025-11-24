classdef UDPWaveformSink < sinks.AbstractSink
    % UDPWaveformSink represents the sink that work on udp loop back

    properties (SetAccess = private)
        RemoteIPAddress   (1,:) char
        RemoteIPPort      (1,1) double
        UdpSender            
        FrameIndex       (1,1) double = 0
    end

    methods
        function obj = UDPWaveformSink(varargin)
            p = inputParser;
            p.addParameter('RemoteIPAddress', '127.0.0.1', @(s)ischar(s) || isstring(s));
            p.addParameter('RemoteIPPort',   31000,       @(x)isnumeric(x)&&isscalar(x));
            p.addParameter('SampleRate',     NaN,         @(x)isnumeric(x)&&isscalar(x));
            p.parse(varargin{:});
            cfg = p.Results;

            obj@sinks.AbstractSink('UDPWaveformSink', cfg.SampleRate);

            obj.RemoteIPAddress = char(cfg.RemoteIPAddress);
            obj.RemoteIPPort    = cfg.RemoteIPPort;

            obj.createSender();
        end

        function writeFrame(obj, frame, info)
            if nargin < 3
                info = struct();
            end

            if isempty(obj.UdpSender)
                error('UDPWaveformSink:NotInitialized', ...
                      'UDP sender object has not been created.');
            end

            if isempty(frame)
                return;
            end

            dataRow = frame(:).';

            obj.UdpSender(dataRow);
            obj.FrameIndex = obj.FrameIndex + 1;

            if isfield(info,'Verbose') && info.Verbose
                fprintf('UDP TX frame %d, %d samples to %s:%d\n', ...
                    obj.FrameIndex, numel(frame), ...
                    obj.RemoteIPAddress, obj.RemoteIPPort);
            end
        end

        function reset(obj)
            obj.release();
            obj.createSender();
            obj.FrameIndex = 0;
        end

        function release(obj)
            if ~isempty(obj.UdpSender)
                try
                    release(obj.UdpSender);
                catch
                end
                obj.UdpSender = [];
            end
        end
    end

    methods (Access = private)
        function createSender(obj)
            % We use dsp for udp transmission
            obj.UdpSender = dsp.UDPSender( ...
                'RemoteIPAddress', obj.RemoteIPAddress, ...
                'RemoteIPPort',    obj.RemoteIPPort);
        end
    end
end
