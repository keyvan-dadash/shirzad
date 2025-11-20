classdef FixedMessageReader < io.Reader
    % FixedMessageReader: returns the same message bytes on every read
    % (if repeat=true) or only once (if repeat=false).

    properties (Access = private)
        msgBytes   % uint8 row vector
        repeat     % logical
        exhausted  % logical, for repeat=false
    end

    methods
        function obj = FixedMessageReader(msg, repeat)
            if nargin < 2
                repeat = true;
            end

            if isstring(msg)
                msg = char(msg);
            end

            if ischar(msg)
                obj.msgBytes = uint8(msg(:).');  % store as row
            elseif isa(msg, 'uint8')
                obj.msgBytes = msg(:).';
            else
                error('FixedMessageReader: msg must be char, string, or uint8.');
            end

            obj.repeat    = logical(repeat);
            obj.exhausted = false;
        end

        function [data, count, eof] = read(obj, maxBytes)
            if nargin < 2 || maxBytes <= 0
                data  = uint8([]);
                count = 0;
                eof   = obj.exhausted;
                return;
            end

            if obj.exhausted && ~obj.repeat
                data  = uint8([]);
                count = 0;
                eof   = true;
                return;
            end

            nAvail = numel(obj.msgBytes);
            n      = min(maxBytes, nAvail);

            data  = obj.msgBytes(1:n).';   % column vector
            count = n;

            if obj.repeat
                eof = false;
            else
                obj.exhausted = true;
                eof           = true;
            end
        end
    end
end
