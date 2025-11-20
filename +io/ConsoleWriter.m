classdef ConsoleWriter < io.Writer
    % ConsoleWriter: prints received bytes as text to stdout.

    methods
        function write(~, data)
            if isempty(data)
                return;
            end

            % Normalize to uint8
            if isstring(data) || ischar(data)
                txt = char(data);
            elseif isa(data, 'uint8')
                txt = char(data(:).');  % row char vector
            else
                error('ConsoleWriter: data must be uint8, char, or string.');
            end

            fprintf('ConsoleWriter: "%s"\n', txt);
        end
    end
end
