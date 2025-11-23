classdef (Abstract) AbstractFirFilter < handle
    % AbstractFirFilter represents the general API of FIR filters in
    % shirzad. The abstract class has three important attributes:
    %   - Coefficients: FIR taps
    %   - State: holding filter state for future use
    %   - Name: name of the filter such as root raised cosine
    properties (SetAccess = protected)
        Coefficients   % 1xN double
        State          % (N-1)x1 double
        Name           % char
    end

    methods
        function obj = AbstractFirFilter(b, name)
            b = b(:).';                    % ensure row vector
            obj.Coefficients = b;
            obj.State        = zeros(numel(b)-1,1);
            obj.Name = name;
        end

        function reset(obj)
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