classdef (Abstract) Writer < handle
    % Writer interface (Go-style):
    %   write(obj, data)
    %
    % data : typically uint8 column vector (raw bytes), but subclasses
    %        can also accept char/string and normalize.

    methods (Abstract)
        write(obj, data)
    end
end
