classdef SDRuBasebandSource < sources.AbstractSource
    %SDRUBASEBANDSOURCE Source of complex baseband samples from USRP N200/N210.
    %
    % This corresponds to the "rx" object in your RX script and hides
    % warm-up, center frequency, gain, etc.
    %
    % Usage:
    %   src = sources.SDRuBasebandSource( ...
    %       'IPAddress',        '192.168.10.4', ...
    %       'CenterFrequency',  fc, ...
    %       'MasterClockRate',  MasterClockRate, ...
    %       'DecimationFactor', Decim, ...
    %       'Gain',             rxGain_dB, ...
    %       'SamplesPerFrame',  SamplesPerFrame);
    %
    %   [x, info] = src.readFrame();
    %
    % info fields:
    %   - IsValid     : true if len>0
    %   - NumSamples  : actual number of samples
    %   - Overrun     : overrun flag (if available)
    %   - CenterFreq  : center frequency in Hz
    %   - FrameIndex  : frame counter

    properties (SetAccess = private)
        IPAddress
        CenterFrequency
        MasterClockRate
        DecimationFactor
        Gain
        SamplesPerFrame
        Platform
        TransportDataType
        OutputDataType
        EnableBurstMode
        Antenna

        RxObj
        FrameIndex = 0
        NumWarmupFrames
    end

    methods
        function obj = SDRuBasebandSource(varargin)
            % SDRuBasebandSource('IPAddress','192.168.10.4', 'CenterFrequency',fc, ...)
            %
            % Default platform matches your script: 'N200/N210/USRP2'

            p = inputParser;
            p.addParameter('IPAddress',        '192.168.10.4', @(s)ischar(s)||isstring(s));
            p.addParameter('CenterFrequency',  10e6,           @isnumeric);
            p.addParameter('MasterClockRate',  100e6,          @isnumeric);
            p.addParameter('DecimationFactor', 100,            @isnumeric);
            p.addParameter('Gain',             3,              @isnumeric);
            p.addParameter('SamplesPerFrame',  32768,          @isnumeric);
            p.addParameter('Platform',         'N200/N210/USRP2', @(s)ischar(s)||isstring(s));
            p.addParameter('TransportDataType','int16',        @(s)ischar(s)||isstring(s));
            p.addParameter('OutputDataType',   'double',       @(s)ischar(s)||isstring(s));
            p.addParameter('EnableBurstMode',  false,          @islogical);
            p.addParameter('Antenna',          'RX2',          @(s)ischar(s)||isstring(s));
            p.addParameter('NumWarmupFrames',  10,             @isnumeric);
            p.parse(varargin{:});

            cfg = p.Results;

            Fs = cfg.MasterClockRate / cfg.DecimationFactor;
            obj@sources.AbstractSource('SDRuBasebandSource', Fs, cfg.SamplesPerFrame);

            obj.IPAddress        = char(cfg.IPAddress);
            obj.CenterFrequency  = cfg.CenterFrequency;
            obj.MasterClockRate  = cfg.MasterClockRate;
            obj.DecimationFactor = cfg.DecimationFactor;
            obj.Gain             = cfg.Gain;
            obj.SamplesPerFrame  = cfg.SamplesPerFrame;
            obj.Platform         = char(cfg.Platform);
            obj.TransportDataType= char(cfg.TransportDataType);
            obj.OutputDataType   = char(cfg.OutputDataType);
            obj.EnableBurstMode  = cfg.EnableBurstMode;
            obj.Antenna          = char(cfg.Antenna);
            obj.NumWarmupFrames  = cfg.NumWarmupFrames;

            obj.createRxObj();
        end

        function [frame, info] = readFrame(obj)
            %READFRAME Get one frame of baseband samples from USRP.
            %
            % Returns complex column vector 'frame' and struct 'info'.

            if isempty(obj.RxObj)
                error('SDRuBasebandSource:NotInitialized', ...
                      'Receiver object has not been created.');
            end

            % comm.SDRuReceiver can return [data,len,overrun]
            try
                [x, len, over] = obj.RxObj();
            catch
                % Older versions may not support 'overrun' output
                [x, len] = obj.RxObj();
                over = false;
            end

            if len > 0
                frame = x(1:len, :);
                obj.FrameIndex = obj.FrameIndex + 1;
                isValid = true;
            else
                frame   = complex([]); %#ok<*COMPLEX>
                isValid = false;
            end

            info = struct( ...
                'IsValid',    isValid, ...
                'NumSamples', len, ...
                'Overrun',    logical(over), ...
                'CenterFreq', obj.CenterFrequency, ...
                'FrameIndex', obj.FrameIndex);
        end

        function reset(obj)
            %RESET Re-create the RX object and re-run warm-up.
            obj.release();
            obj.createRxObj();
            obj.FrameIndex = 0;
        end

        function release(obj)
            %RELEASE Release USRP handle.
            if ~isempty(obj.RxObj)
                try
                    release(obj.RxObj);
                catch
                end
                obj.RxObj = [];
            end
        end
    end

    methods (Access = private)
        function createRxObj(obj)
            % Internal helper to configure comm.SDRuReceiver

            rx = comm.SDRuReceiver( ...
                'Platform',         obj.Platform, ...
                'IPAddress',        obj.IPAddress, ...
                'CenterFrequency',  obj.CenterFrequency, ...
                'MasterClockRate',  obj.MasterClockRate, ...
                'DecimationFactor', obj.DecimationFactor, ...
                'Gain',             obj.Gain, ...
                'SamplesPerFrame',  obj.SamplesPerFrame, ...
                'OutputDataType',   obj.OutputDataType, ...
                'TransportDataType',obj.TransportDataType, ...
                'EnableBurstMode',  obj.EnableBurstMode);

            % Antenna may not exist on some platforms
            try
                rx.Antenna = obj.Antenna;
            catch
            end

            % Warm-up / flush (like your "for i=1:10, step(rx); end")
            for k = 1:obj.NumWarmupFrames
                step(rx);
            end

            obj.RxObj = rx;
        end
    end
end
