classdef MSequenceGenerator < handle
    % MSequenceGenerator generate a sequence that consist of m bits

    properties
        Degree     (1,1) double {mustBeInteger,mustBePositive} = 9;
        Taps       (1,:) double = []; % taps for generating the sequence
        State      (1,:) logical = []; % the state that the linear shift register has
    end

    methods
        function obj = MSequenceGenerator(varargin)
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
                          'Init state length must equal Degree.');
                end
                if ~any(initStateUser)
                    error('MSequenceGenerator:InitStateZero', ...
                          'Init state must be non-zero for m-sequence.');
                end
                obj.State = initStateUser;
            end
        end

        function bits = generateBits(obj, N)
            % Produce N bits (0/1) from the LFSR
            bits = false(N,1);
            for n = 1:N
                % Output bit: last stage
                outBit = obj.State(end);
                bits(n) = outBit;

                % XOR of tap positions
                fb = mod(sum(obj.State(obj.Taps)), 2);

                % Shift right, insert feedback at left
                obj.State(2:end) = obj.State(1:end-1);
                obj.State(1)     = logical(fb);
            end
            bits = double(bits);
        end
    end

    methods (Access = private)
        function taps = defaultTaps(obj, m)
            % Default taps from:
            % https://en.wikipedia.org/wiki/Linear-feedback_shift_register
            switch m
                case 3    % x^3 + x^2 + 1
                    taps = [3 2];
                case 4    % x^4 + x^3 + 1
                    taps = [4 3];
                case 5    % x^5 + x^3 + 1
                    taps = [5 3];
                case 6    % x^6 + x^5 + 1
                    taps = [6 5];
                case 7    % x^7 + x^6 + 1
                    taps = [7 6];
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
