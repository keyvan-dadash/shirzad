classdef TestFileChunk < matlab.unittest.TestCase
    % Unit tests for filetransfer.FileChunk

    methods (Test)
        function testRoundTripBasic(testCase)
            fileId    = uint8(5);
            offset    = uint32(12345);
            totalSize = uint32(98765);
            isLast    = true;
            dataIn    = uint8(randi([0 1], 50, 1));

            payload = filetransfer.FileChunk.encode(fileId, offset, totalSize, isLast, dataIn);

            [meta, dataOut] = filetransfer.FileChunk.decode(payload);

            % Meta fields
            testCase.verifyEqual(meta.FileId,    fileId,    'FileId mismatch');
            testCase.verifyEqual(meta.Offset,    offset,    'Offset mismatch');
            testCase.verifyEqual(meta.TotalSize, totalSize, 'TotalSize mismatch');
            testCase.verifyTrue(meta.IsLast,     'IsLast should be true');

            % Data payload
            testCase.verifyEqual(dataOut(:), dataIn(:), 'Data payload mismatch');
        end

        function testHeaderLengthConstant(testCase)
            % header size should match the reader's constant (10)
            testCase.verifyEqual( ...
                double(filetransfer.FileChunk.HEADER_BYTES), ...
                double(io.FileChunkReader.HEADER_BYTES), ...
                'HEADER_BYTES mismatch between FileChunk and FileChunkReader.');
        end
    end
end
