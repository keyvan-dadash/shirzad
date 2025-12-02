classdef FixedMessageReader < io.Reader
    % FixedMessageReader: returns the same message bytes on every read
    % either once or infinite

    properties (Access = private)
        msgBytes   % uint8 row vector
        repeat     % should be repated?
        exhausted  % for repeat=false
        internal
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
                obj.msgBytes = uint8(msg(:).');
            elseif isa(msg, 'uint8')
                obj.msgBytes = msg(:).';
            else
                error('FixedMessageReader: msg must be char, string, or uint8.');
            end

            obj.repeat    = logical(repeat);
            obj.exhausted = false;
            obj.internal  = uint64(0);
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

            obj.internal = obj.internal + 1;

            % base message as char
            baseStr = char(obj.msgBytes);   % row char

            % append space + decimal counter
            suffixStr = [' ' num2str(obj.internal)];

            % full message as uint8
            msgWithCounter = uint8([baseStr suffixStr]);

            nAvail = numel(msgWithCounter);
            n      = min(maxBytes, nAvail);

            data  = msgWithCounter(1:n).';
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
