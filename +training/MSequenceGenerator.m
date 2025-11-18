% File: +training/MSequenceGenerator.m
classdef MSequenceGenerator < handle
    % MSequenceGenerator
    %
    % Simple maximal-length LFSR-based m-sequence generator.
    %
    % Usage (common case):
    %   gen = training.MSequenceGenerator('Degree', 9);    % m=9 -> period 511
    %   bits = gen.generateBits( N );                     % 0/1 bits
    %   chips = gen.generateBipolar( N );                 % +1/-1
    %
    % You can then map bits to QPSK using your existing modQPSK, and use
    % the resulting symbols as the "half" preamble in a Schmidl [a, a].
    %
    % This is *stateful* across calls. For a fixed preamble, typically:
    %   1) Construct generator once
    %   2) Call generateBits(...) once at script startup
    %   3) Store that sequence as your preamble bits
    %
    % Properties (name–value in ctor):
    %   Degree      : LFSR length m (3..16 practical)
    %   Taps        : tap positions (1..m). If empty, a default primitive
    %                 polynomial is chosen for common m.
    %   InitState   : 1×m or m×1 nonzero vector of 0/1. Default: all ones.
    %
    % Methods:
    %   generateBits(N)    -> N×1 vector of {0,1}
    %   generateBipolar(N) -> N×1 vector of {+1, -1}
    %   generateQpskSyms(Nsym) -> Nsym×1 QPSK symbols (Gray mapping)
    %   reset(seed)        -> reinitialize state

    properties
        Degree     (1,1) double {mustBeInteger,mustBePositive} = 9;
        Taps       (1,:) double = [];      % indices in [1..Degree]
        State      (1,:) logical = [];     % current LFSR state
    end

    methods
        function obj = MSequenceGenerator(varargin)
            % Name–value parsing
            if mod(numel(varargin),2) ~= 0
                error('MSequenceGenerator:NameValue', ...
                      'Constructor expects name-value pairs.');
            end
            initStateUser = [];

            for k = 1:2:numel(varargin)
                name  = lower(varargin{k});
                value = varargin{k+1};
                switch name
                    case 'degree'
                        obj.Degree = value;
                    case 'taps'
                        obj.Taps = value(:).';
                    case 'initstate'
                        initStateUser = logical(value(:).');
                    otherwise
                        error('MSequenceGenerator:UnknownParam', ...
                              'Unknown parameter "%s".', name);
                end
            end

            % Set default taps for some common degrees if none given
            if isempty(obj.Taps)
                obj.Taps = obj.defaultTaps(obj.Degree);
            end

            % Default init state: all ones
            if isempty(initStateUser)
                obj.State = true(1, obj.Degree);
            else
                if numel(initStateUser) ~= obj.Degree
                    error('MSequenceGenerator:InitStateSize', ...
                          'InitState length must equal Degree.');
                end
                if ~any(initStateUser)
                    error('MSequenceGenerator:InitStateZero', ...
                          'InitState must be non-zero for m-sequence.');
                end
                obj.State = initStateUser;
            end
        end

        function reset(obj, newState)
            % RESET  Reinitialize LFSR state
            if nargin < 2 || isempty(newState)
                obj.State = true(1, obj.Degree);  % all ones
            else
                newState = logical(newState(:).');
                if numel(newState) ~= obj.Degree
                    error('MSequenceGenerator:ResetSize', ...
                          'New state length must equal Degree.');
                end
                if ~any(newState)
                    error('MSequenceGenerator:ResetZero', ...
                          'New state must be non-zero for m-sequence.');
                end
                obj.State = newState;
            end
        end

        function bits = generateBits(obj, N)
            % GENERATEBITS  Produce N bits (0/1) from the LFSR
            bits = false(N,1);
            for n = 1:N
                % Output bit: last stage
                outBit = obj.State(end);
                bits(n) = outBit;

                % Feedback: XOR of tap positions
                fb = mod(sum(obj.State(obj.Taps)), 2);

                % Shift right, insert feedback at left
                obj.State(2:end) = obj.State(1:end-1);
                obj.State(1)     = logical(fb);
            end
            bits = double(bits); % return as 0/1 (double) for convenience
        end

        function chips = generateBipolar(obj, N)
            % GENERATEBIPOLAR  N bipolar (+1/-1) chips
            b = obj.generateBits(N);
            chips = 1 - 2*b;   % 0 -> +1, 1 -> -1
        end

        function syms = generateQpskSyms(obj, Nsym)
            % GENERATEQPSKSYMS  Produce Nsym Gray-mapped QPSK symbols
            % using the internal m-sequence bits.
            %
            % Mapping (Gray):
            %   b0 b1 : symbol
            %   0  0  :  +1 + j
            %   0  1  :  -1 + j
            %   1  1  :  -1 - j
            %   1  0  :  +1 - j

            Nbits = 2 * Nsym;
            b     = obj.generateBits(Nbits);
            b     = reshape(b, 2, []).';  % Nsym x 2

            syms = zeros(Nsym,1);
            for k = 1:Nsym
                b0 = b(k,1);
                b1 = b(k,2);
                if     b0==0 && b1==0
                    s = 1 + 1j;
                elseif b0==0 && b1==1
                    s = -1 + 1j;
                elseif b0==1 && b1==1
                    s = -1 - 1j;
                else % 10
                    s = 1 - 1j;
                end
                syms(k) = s;
            end
            syms = syms / sqrt(2);  % unit average power
        end
    end

    methods (Access = private)
        function taps = defaultTaps(obj, m)
            % DEFAULTTAPS  Provide some primitive polynomials (Fibonacci form)
            % taps are given as positions (1..m) to XOR for feedback.
            switch m
                case 3    % x^3 + x + 1
                    taps = [3 1];
                case 4    % x^4 + x + 1
                    taps = [4 1];
                case 5    % x^5 + x^2 + 1
                    taps = [5 2];
                case 6    % x^6 + x + 1
                    taps = [6 1];
                case 7    % x^7 + x^3 + 1
                    taps = [7 3];
                case 8    % x^8 + x^6 + x^5 + x^4 + 1
                    taps = [8 6 5 4];
                case 9    % x^9 + x^5 + 1
                    taps = [9 5];
                case 10   % x^10 + x^7 + 1
                    taps = [10 7];
                case 11   % x^11 + x^9 + 1
                    taps = [11 9];
                case 12   % x^12 + x^11 + x^10 + x^4 + 1
                    taps = [12 11 10 4];
                otherwise
                    error('MSequenceGenerator:NoDefaultTaps', ...
                        'No default taps for Degree=%d. Please provide ''Taps'' explicitly.', m);
            end
        end
    end
end
