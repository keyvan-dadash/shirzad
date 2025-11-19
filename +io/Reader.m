classdef (Abstract) Reader < handle
    % Reader interface (Go-style):
    %   [data, count, eof] = read(obj, maxBytes)
    %
    % data  : uint8 column vector
    % count : number of bytes returned (numel(data))
    % eof   : logical, true if no more data (for repeating sources you can keep it false)

    methods (Abstract)
        [data, count, eof] = read(obj, maxBytes)
    end
end
