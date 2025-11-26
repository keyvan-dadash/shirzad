classdef (Abstract) Writer < handle
    % Writer interface (Go-style):
    %   write(obj, data)
    %
    % data : typically uint8 column vector (raw bytes).

    methods (Abstract)
        write(obj, data)
        close(obj)
    end
end
