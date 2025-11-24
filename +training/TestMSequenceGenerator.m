classdef TestMSequenceGenerator < matlab.unittest.TestCase
    % Tests for training.MSequenceGenerator

    methods (Test)
        function testDefaultConstruction(testCase)
            gen = training.MSequenceGenerator('Degree', 9);

            testCase.verifyEqual(gen.Degree, 9);
            testCase.verifyEqual(gen.Taps, [9 5], ...
                'Default taps for Degree=9 should be [9 5].');

            testCase.verifyEqual(gen.State, true(1,9), ...
                'Default init state should be all ones.');
        end

        function testKnownSequenceDegree3(testCase)
            % Check that Degree=3 default taps produce a known m-sequence.
            %
            % Polynomial: x^3 + x^2 + 1 -> taps [3 2]
            % With all-ones initial state and this implementation
            % (output = last stage), the first 7 bits are:
            %   [1 1 1 0 0 1 0].

            gen = training.MSequenceGenerator('Degree', 3);
            N   = 2^gen.Degree - 1;      % 7
            bits = gen.generateBits(N);

            expBits = [1; 1; 1; 0; 0; 1; 0];

            testCase.verifyEqual(bits, expBits, ...
                'Degree=3 sequence does not match expected m-sequence.');

            % Check that sequence is not all zeros or all ones
            testCase.verifyTrue(any(bits == 0) && any(bits == 1), ...
                'Sequence should contain both 0 and 1.');
        end

        function testMaxLengthPeriodAndRepetition(testCase)
            % For Degree=4, verify periodicity over one full m-sequence.
            gen = training.MSequenceGenerator('Degree', 4);
            N   = 2^gen.Degree - 1;   % 15

            seq1 = gen.generateBits(N);
            seq2 = gen.generateBits(N);  % Same state after one period

            % Both sequence should be same as we rotate a full period
            testCase.verifyEqual(seq1, seq2, ...
                'Two consecutive periods of m-sequence should match.');
        end

        function testNoDefaultTapsDegreeError(testCase)
            testCase.verifyError(@() training.MSequenceGenerator('Degree', 13), ...
                'MSequenceGenerator:NoDefaultTaps');
        end
    end
end
