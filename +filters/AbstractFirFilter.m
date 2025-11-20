classdef (Abstract) AbstractFirFilter < handle
    %ABSTRACTFIRFILTER Base class for streaming FIR filters.
    %
    %   - Coefficients : row vector of FIR taps
    %   - State        : filter state (for streaming)
    %   - Name         : descriptive name

    properties (SetAccess = protected)
        Coefficients   % 1xN double
        State          % (N-1)x1 double
        Name           % char
    end

    methods
        function obj = AbstractFirFilter(b, name)
            if nargin < 1 || isempty(b)
                error('AbstractFirFilter:MissingCoeffs', ...
                      'Filter coefficients must be provided.');
            end
            b = b(:).';                    % ensure row vector
            obj.Coefficients = b;
            obj.State        = zeros(numel(b)-1,1);

            if nargin >= 2
                obj.Name = char(name);
            else
                obj.Name = 'FIR Filter';
            end
        end

        function reset(obj)
            %RESET Clear the filter state.
            obj.State(:) = 0;
        end

        function y = process(obj, x)
            %PROCESS Stream input x through FIR filter.
            %
            % x : column or row vector
            % y : column vector (same length)

            x = x(:);   % column
            [y, obj.State] = filter(obj.Coefficients, 1, x, obj.State);
        end
    end
end