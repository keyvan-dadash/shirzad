classdef SDRuWaveformSink < sinks.AbstractSink
    % SDRuWaveformSink transfers the wave to the SDR platform.

    properties (SetAccess = private)
        IPAddress
        CenterFrequency
        MasterClockRate
        InterpolationFactor
        Gain
        Platform
        TransportDataType
        UseExternalRef

        TxObj
        FrameIndex = 0
    end

    methods
        function obj = SDRuWaveformSink(varargin)
            % SDRuWaveformSink('IPAddress','192.168.10.5', 'CenterFrequency',fc, ...)

            p = inputParser;
            p.addParameter('IPAddress',          '192.168.10.5', @(s)ischar(s)||isstring(s));
            p.addParameter('CenterFrequency',    10e6,           @isnumeric);
            p.addParameter('MasterClockRate',    100e6,          @isnumeric);
            p.addParameter('InterpolationFactor',100,            @isnumeric);
            p.addParameter('Gain',               3,              @isnumeric);
            p.addParameter('Platform',           'N200/N210/USRP2', @(s)ischar(s)||isstring(s));
            p.addParameter('TransportDataType',  'int16',        @(s)ischar(s)||isstring(s));
            p.addParameter('UseExternalRef',     false,          @islogical);
            p.parse(varargin{:});

            cfg = p.Results;

            Fs = cfg.MasterClockRate / cfg.InterpolationFactor;
            obj@sinks.AbstractSink('SDRuWaveformSink', Fs);

            obj.IPAddress          = char(cfg.IPAddress);
            obj.CenterFrequency    = cfg.CenterFrequency;
            obj.MasterClockRate    = cfg.MasterClockRate;
            obj.InterpolationFactor= cfg.InterpolationFactor;
            obj.Gain               = cfg.Gain;
            obj.Platform           = char(cfg.Platform);
            obj.TransportDataType  = char(cfg.TransportDataType);
            obj.UseExternalRef     = cfg.UseExternalRef;

            obj.createTxObj();
        end

        function writeFrame(obj, frame, info)
            if nargin < 3
                info = struct();
            end

            if isempty(obj.TxObj)
                error('SDRuWaveformSink:NotInitialized', ...
                      'Transmitter object has not been created.');
            end

            obj.TxObj(frame);
            obj.FrameIndex = obj.FrameIndex + 1;

            % For debug
            if isfield(info,'Verbose') && info.Verbose
                fprintf('TX frame %d, %d samples\n', ...
                        obj.FrameIndex, numel(frame));
            end
        end

        function reset(obj)
            obj.release();
            obj.createTxObj();
            obj.FrameIndex = 0;
        end

        function release(obj)
            if ~isempty(obj.TxObj)
                try
                    release(obj.TxObj);
                catch
                end
                obj.TxObj = [];
            end
        end
    end

    methods (Access = private)
        function createTxObj(obj)
            % Configuration for using comm.SDRuTransmitter

            tx = comm.SDRuTransmitter( ...
                'Platform',         obj.Platform, ...
                'IPAddress',        obj.IPAddress, ...
                'CenterFrequency',  obj.CenterFrequency, ...
                'MasterClockRate',  obj.MasterClockRate, ...
                'InterpolationFactor', obj.InterpolationFactor, ...
                'Gain',             obj.Gain, ...
                'TransportDataType',obj.TransportDataType);

            if obj.UseExternalRef
                try
                    tx.ClockSource = 'External';
                    tx.PPSSource   = 'External';
                catch
                end
            end

            obj.TxObj = tx;
        end
    end
end
