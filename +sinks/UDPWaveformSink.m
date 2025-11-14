classdef UDPWaveformSink < sinks.AbstractSink
    %UDPWAVEFORMSINK Sink that sends complex baseband waveforms over UDP.
    %
    % Intended as a drop-in replacement for SDRuWaveformSink when you
    % don't have the USRP handy.
    %
    % Typical usage in TX:
    %   Fs = 1e6;
    %   udpTx = sinks.UDPWaveformSink( ...
    %       'RemoteIPAddress', '127.0.0.1', ...
    %       'RemoteIPPort',   31000, ...
    %       'SampleRate',     Fs);
    %
    %   % Inside TX loop:
    %   udpTx.writeFrame(txWave, struct('FrameIndex',k));
    %
    % NOTES:
    %   - This uses dsp.UDPSender (DSP System Toolbox).
    %   - Each call sends ONE UDP packet.
    %   - Make sure numel(txWave) <= MaximumMessageLength on the receiver.
    %   - We send as complex double; receiver must set IsMessageComplex=true,
    %     MessageDataType='double'.

    properties (SetAccess = private)
        RemoteIPAddress   (1,:) char
        RemoteIPPort      (1,1) double
        UdpSender                     % dsp.UDPSender
        FrameIndex       (1,1) double = 0
    end

    methods
        function obj = UDPWaveformSink(varargin)
            % UDPWaveformSink('RemoteIPAddress','127.0.0.1', ...
            %                'RemoteIPPort',31000, ...
            %                'SampleRate',Fs)
            %
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
            %WRITEFRAME Send one waveform frame as a UDP packet.
            %
            % frame : complex column/row vector (will be reshaped to row)
            % info  : optional struct, e.g. info.FrameIndex, info.Verbose

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

            % dsp.UDPSender expects a vector; we send row vector
            dataRow = frame(:).';  % complex double row

            obj.UdpSender(dataRow);
            obj.FrameIndex = obj.FrameIndex + 1;

            if isfield(info,'Verbose') && info.Verbose
                fprintf('UDP TX frame %d, %d samples to %s:%d\n', ...
                    obj.FrameIndex, numel(frame), ...
                    obj.RemoteIPAddress, obj.RemoteIPPort);
            end
        end

        function reset(obj)
            %RESET Recreate sender and reset counters.
            obj.release();
            obj.createSender();
            obj.FrameIndex = 0;
        end

        function release(obj)
            %RELEASE Release UDP sender.
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
            % Internal helper to configure dsp.UDPSender.
            %
            % RemoteIPPort is mandatory; RemoteIPAddress can stay default
            % (127.0.0.1) for loopback, or be set to another host.
            obj.UdpSender = dsp.UDPSender( ...
                'RemoteIPAddress', obj.RemoteIPAddress, ...
                'RemoteIPPort',    obj.RemoteIPPort);
        end
    end
end
