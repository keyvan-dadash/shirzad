classdef TestConvEncoderViterbiDecoder < matlab.unittest.TestCase
    % Unit tests for fec.ConvEncoder and fec.ViterbiDecoder
    
    methods (Test)
        function testConvEncoderKnownSequenceNoTerminate(testCase)
            % Check hand-derived output for a small sequence.
            % Code: G = [111; 101], initial state [0 0]
            % u = [1 0 1], terminate = false
            %
            % Time 1: u=1, reg=[1 0 0] -> v=[1 1]
            % Time 2: u=0, reg=[0 1 0] -> v=[1 0]
            % Time 3: u=1, reg=[1 0 1] -> v=[0 0]
            % So v = [1 1 1 0 0 0]
            
            G = [1 1 1;
                 1 0 1];
            enc = fec.ConvEncoder(G);
            u   = [1; 0; 1];
            
            v = enc.encode(u, false);
            
            expectedV = [1; 1; 1; 0; 0; 0];
            testCase.verifyEqual(v(:), expectedV(:), ...
                'Encoded bits do not match hand calculation.');
        end
        
        function testConvEncoderTerminateAddsTailBits(testCase)
            % Terminate should append Memory zeros.
            enc = fec.ConvEncoder.rateHalf_K3();
            u   = randi([0 1], 10, 1);
            
            vTerm   = enc.encode(u, true);
            enc.reset();
            vNoTerm = enc.encode(u, false);
            
            mem = enc.Memory;
            testCase.verifyEqual(numel(vTerm), (numel(u)+mem)*enc.nOut, ...
                'Terminated codeword length mismatch.');
            testCase.verifyEqual(numel(vNoTerm), numel(u)*enc.nOut, ...
                'Non-terminated codeword length mismatch.');
        end
        
        function testViterbiRoundTripShortRandom(testCase)
            % Encode + decode a short sequence.
            enc = fec.ConvEncoder.rateHalf_K3();
            dec = fec.ViterbiDecoder.rateHalf_K3();
            
            rng(123);
            infoLen = 20;
            u       = randi([0 1], infoLen, 1);
            
            v    = enc.encode(u, true);      % terminated
            uHat = dec.decode(v);
            
            testCase.verifyEqual(uHat(:), double(u(:)), ...
                'Short random round-trip failed.');
        end
        
        function testViterbiInputLengthMustBeMultipleOfnOut(testCase)
            dec = fec.ViterbiDecoder.rateHalf_K3();
            badV = [1 0 1];  % length 3, nOut=2 -> invalid
            
            caught = false;
            try
                dec.decode(badV);
            catch ME
                caught = true;
                % Message should at least mention "input length"
                testCase.verifyNotEmpty(strfind(ME.message, 'input length'), ...
                    'Error message does not mention input length mismatch.');
            end
            testCase.verifyTrue(caught, ...
                'Decoder did not throw any error for bad input length.');
        end
        
        function testViterbiRoundTripLongRandomPerformance(testCase)
            % Long sequence to inspect performance and correctness.
            enc = fec.ConvEncoder.rateHalf_K3();
            dec = fec.ViterbiDecoder.rateHalf_K3();
            
            rng(42);
            infoLen = 20000;
            
            u = randi([0 1], infoLen, 1);
            
            tEnc = tic;
            v    = enc.encode(u, true);
            tEnc = toc(tEnc);
            
            tDec = tic;
            uHat = dec.decode(v);
            tDec = toc(tDec);
            
            testCase.verifyEqual(uHat(:), double(u(:)), ...
                'Long random round-trip failed.');
            
            totalT = tEnc + tDec;
            fprintf('\nConvEncoder/Viterbi timing (infoLen=%d bits):\n', infoLen);
            fprintf('  encode: %.6f s\n', tEnc);
            fprintf('  decode: %.6f s\n', tDec);
            fprintf('  total : %.6f s\n', totalT);
        end
        
        function testEncoderResetBehavior(testCase)
            % Ensure reset() really returns to all-zero state.
            enc = fec.ConvEncoder.rateHalf_K3();
            u1  = [1; 0; 1];
            v1  = enc.encode(u1, false);
            
            enc.reset();
            v2 = enc.encode(u1, false);
            
            testCase.verifyEqual(v1, v2, ...
                'Encoder reset() did not restore initial state behavior.');
        end
        
        function testCompareWithCommAndScalarMexViterbiDecoders(testCase)
            % Compare three convolutional decoders on the same codeword:
            %   1) fec.ViterbiDecoder (Custom written code)
            %   2) comm.ViterbiDecoder (Comm Toolbox)
            %   3) fec.viterbi_k3_mex  (Custom written scalar MEX for K=3)
            %
            % Each is timed over 100 runs.
        
            enc       = fec.ConvEncoder.rateHalf_K3();
            decCustom = fec.ViterbiDecoder.rateHalf_K3();
        
            trellis = poly2trellis(3, [7 5]);
            decComm = comm.ViterbiDecoder( ...
                'TrellisStructure',  trellis, ...
                'InputFormat',       'Hard', ...
                'TerminationMethod', 'Terminated', ...
                'TracebackDepth',    34);
        
            rng(42);
            infoLen = 2000;
            u       = randi([0 1], infoLen, 1);
        
            v = enc.encode(u, true);
            nIter = 100;
        
            % Custom written code in MATLAB
            tSumCustom = 0;
            for k = 1:nIter
                tStart = tic;
                uHatCustom = decCustom.decode(v);
                tSumCustom = tSumCustom + toc(tStart);
            end
            tCustom = tSumCustom / nIter;
        
            % Communications Toolbox Decoder
            tSumComm = 0;
            for k = 1:nIter
                tStart = tic;
                uHatComm = decComm(v);
                tSumComm = tSumComm + toc(tStart);
            end
            tComm = tSumComm / nIter;
        
            % MEX decoder
            tSumMexScalar = 0;
            for k = 1:nIter
                tStart = tic;
                uHatMexScalar = fec.viterbi_k3_mex(v);
                tSumMexScalar = tSumMexScalar + toc(tStart);
            end
            tMexScalar = tSumMexScalar / nIter;
        
            uHatCustom    = double(uHatCustom(:));
            uHatComm      = double(uHatComm(:));
            uHatMexScalar = double(uHatMexScalar(:));
            uTrue         = double(u(:));
        
            nMin = min([numel(uTrue), numel(uHatCustom), numel(uHatComm), numel(uHatMexScalar)]);
        
            testCase.verifyGreaterThan(nMin, 0, ...
                'No overlapping bits between decoders / reference.');
        
            % All the decoders should agree on same decoded result
            testCase.verifyEqual(uHatCustom(1:nMin),    uTrue(1:nMin), ...
                'Custom MATLAB decoder does not match original bits over first nMin bits.');
            testCase.verifyEqual(uHatComm(1:nMin),      uTrue(1:nMin), ...
                'comm.ViterbiDecoder does not match original bits over first nMin bits.');
            testCase.verifyEqual(uHatMexScalar(1:nMin), uTrue(1:nMin), ...
                'Scalar MEX decoder does not match original bits over first nMin bits.');
        
            % Summary of timing
            fprintf('\nConvolutional Viterbi comparison (infoLen=%d bits, %d iterations):\n', ...
                    infoLen, nIter);
            fprintf('  Custom fec.ViterbiDecoder  : %.6f s\n', tCustom);
            fprintf('  comm.ViterbiDecoder        : %.6f s\n', tComm);
            fprintf('  fec.viterbi_k3_mex (K=3)   : %.6f s\n\n', tMexScalar);
        end
    end
end
