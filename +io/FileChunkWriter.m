classdef FileChunkWriter < io.Writer
    % FileChunkWriter uses file assembler to write the file to the disk
    properties
        assembler
        cnt
        printCnt
    end
    
    methods
        function obj = FileChunkWriter(fileAssembler, printCnt)
            obj.assembler = fileAssembler;
            obj.cnt = 0;
            obj.printCnt = printCnt;
        end
        
        function n = write(obj, data)
            obj.cnt = obj.cnt + 1;
            [meta, chunkData] = filetransfer.FileChunk.decode(data);

            if meta.FileId == obj.assembler.FileId
                obj.assembler.acceptChunk(meta.Offset, meta.TotalSize, meta.IsLast, chunkData);

                [offTot, tot, hasTotal, bufBytes] = obj.assembler.status();

                if mod(obj.cnt, obj.printCnt) == 0
                    fprintf('Chunk off=%u len=%d isLast=%d | written=%u, buffered=%u, minOff=%u\n', ...
                        meta.Offset, numel(chunkData), meta.IsLast, offTot, bufBytes, minOff);

                    if hasTotal
                        fprintf('File progress: %d / %d bytes (%.1f%%), buffered=%d bytes\n', ...
                            offTot, tot, 100*double(offTot)/double(tot), bufBytes);
                    else
                        fprintf('File progress: %d bytes written so far, %d bytes buffered (total unknown)\n', ...
                            offTot, bufBytes);
                    end
                end

                if obj.assembler.isComplete()
                    [offTot, tot, hasTotal] = obj.assembler.status();
                    fprintf('File transfer COMPLETE: %d/%d bytes (HasTotal=%d)\n', ...
                        offTot, tot, hasTotal);
                    obj.assembler.Writer.close();
                    return;
                end
            end
        end
        
        function isClosed = close(obj)
            obj.assembler.Writer.close();
        end
    end
end

