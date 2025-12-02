classdef FileAssembler < handle
    % FileAssembler
    %   Reassembles a single file from out-of-order FileChunks.
    
    properties
        FileId     uint8
        Writer               % a Writer subclass
    end
    
    properties (Access = private)
        NextOffset uint32 = uint32(0);     % next offset to write
        TotalSize  uint32 = uint32(0);
        HasTotal   logical = false;
        Chunks                    % A map that holds chunks and their offsets
    end
    
    methods
        function obj = FileAssembler(fileId, writer)
            if nargin < 2
                error('FileAssembler requires fileId and writer.');
            end
            obj.FileId = uint8(fileId);
            obj.Writer = writer;
            obj.Chunks = containers.Map('KeyType','uint32','ValueType','any');
        end
        
        function acceptChunk(obj, offset, totalSize, isLast, data)
            if ~isa(data, 'uint8')
                data = uint8(data);
            end
            data = data(:);
            
            offset = uint32(offset);
            len    = uint32(numel(data));
            
            if isLast
                obj.TotalSize = uint32(totalSize);
                obj.HasTotal  = true;
            end
            
            % Already completely written?
            if offset + len <= obj.NextOffset
                % Duplicate / obsolete
                return;
            end
            
            % If it partially overlaps the already-written area, trim
            % leading (good for redundancy)
            if offset < obj.NextOffset
                trim = obj.NextOffset - offset;   % bytes to drop
                data(1:double(trim)) = [];
                offset = obj.NextOffset;
                len    = uint32(numel(data));
            end
            
            if len == 0
                return;
            end
            
            % Store chunk if not already present and it is not the next
            % chunk to write
            if ~isKey(obj.Chunks, offset)
                obj.Chunks(offset) = data;
            else
                % Already have chunk starting at this offset: ignore
            end
            
            % Flush data
            obj.flush();
        end
        
        function flush(obj)
            % Write any chunks that start at NextOffset
            while isKey(obj.Chunks, obj.NextOffset)
                startOff = obj.NextOffset;
                data = obj.Chunks(startOff);
                remove(obj.Chunks, startOff);

                fprintf('FLUSH: writing offset=%u, len=%u\n', ...
                    uint32(startOff), uint32(numel(data)));
                
                obj.Writer.write(data);
                % Update the next offset
                obj.NextOffset = obj.NextOffset + uint32(numel(data));
            end
        end
        
        function done = isComplete(obj)
            done = obj.HasTotal && (obj.NextOffset >= obj.TotalSize);
        end
        
        function [nextOff, total, hasTotal, buffered] = status(obj)
            nextOff  = obj.NextOffset;
            total    = obj.TotalSize;
            hasTotal = obj.HasTotal;
            
            % Sum lengths of all stored chunks
            buffered = uint32(0);
            ks = obj.Chunks.keys;
            for i = 1:numel(ks)
                offKey = ks{i};
                data   = obj.Chunks(offKey);
                buffered = buffered + uint32(numel(data));
            end
        end
    end
end
