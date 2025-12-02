classdef FileWriter < io.Writer
    % FileWriter
    %   Simple Writer that appends bytes to a file.
    %   Used by FileAssembler once chunks are in order.
    
    properties (Access = private)
        Fid
        FileName
    end
    
    methods
        function obj = FileWriter(fileName)
            if nargin < 1
                error('FileWriter requires a fileName.');
            end
            obj.FileName = fileName;
            % Open for write
            [fid, msg] = fopen(fileName, 'w');
            if fid < 0
                error('FileWriter:CannotOpen', ...
                      'Failed to open "%s" for writing: %s', fileName, msg);
            end
            obj.Fid = fid;
        end
        
        function close(obj)
            if ~isempty(obj.Fid) && obj.Fid > 0
                fclose(obj.Fid);
                obj.Fid = [];
            end
        end
        
        function write(obj, data)
            if isempty(data)
                return;
            end
            if ~isa(data, 'uint8')
                data = uint8(data);
            end
            data = data(:);
            cnt = fwrite(obj.Fid, data, 'uint8');
            if cnt ~= numel(data)
                error('FileWriter:ShortWrite', ...
                      'Wrote %d of %d bytes to "%s".', ...
                      cnt, numel(data), obj.FileName);
            end
        end
    end
end
