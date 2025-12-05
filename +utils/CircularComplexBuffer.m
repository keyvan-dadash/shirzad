classdef CircularComplexBuffer < handle
    %CIRCULARCOMPLEXBUFFER Fixed-size circular buffer for complex samples.
    % The intetion of this class is make the management of samples in the
    % rx code more easy.

    properties (SetAccess = private)
        Capacity
        Data
        Head
        Length
    end

    methods
        function obj = CircularComplexBuffer(capacity)
            obj.Capacity = capacity;
            obj.Data     = single(complex(zeros(capacity,1)));
            obj.Head     = 1;
            obj.Length   = 0;
        end

        function clear(obj)
            obj.Head   = 1;
            obj.Length = 0;
        end

        function append(obj, x)
            x = x(:);
            n = numel(x);
            if n == 0
                return;
            end

            % If more than capacity, keep only last Capacity samples
            if n >= obj.Capacity
                obj.Data = complex(x(end-obj.Capacity+1:end));
                obj.Head = 1;
                obj.Length = obj.Capacity;
                return;
            end

            % Drop oldest if we would overflow
            freeSpace = obj.Capacity - obj.Length;
            overflow  = n - freeSpace;
            if overflow > 0
                obj.dropFirst(overflow);
            end

            % Now we have space, so lets write at the tail
            tailPos = obj.wrapIndex(obj.Head + obj.Length); % first free slot
            firstChunk = min(n, obj.Capacity - tailPos + 1);

            obj.Data(tailPos : tailPos + firstChunk - 1) = x(1:firstChunk);

            rem = n - firstChunk;
            if rem > 0
                obj.Data(1:rem) = x(firstChunk+1:end);
            end

            obj.Length = obj.Length + n;
        end

        function dropFirst(obj, k)
            % Drop first k samples
            if k <= 0 || obj.Length == 0
                return;
            end
            if k >= obj.Length
                obj.clear();
                return;
            end
            obj.Head   = obj.wrapIndex(obj.Head + k);
            obj.Length = obj.Length - k;
        end

        function v = toVector(obj)
            % Returning currection buffer as one continous buffer.
            L = obj.Length;
            if L == 0
                v = complex([]); 
                return;
            end

            h = obj.Head;
            c = obj.Capacity;

            if h + L - 1 <= c
                % Single contiguous block
                v = obj.Data(h : h + L - 1);
            else
                % The buffer is wrapped
                firstLen  = c - h + 1;
                secondLen = L - firstLen;
                v = [obj.Data(h:end); obj.Data(1:secondLen)];
            end
        end
    end

    methods (Access = private)
        function idx = wrapIndex(obj, idx)
            idx = mod(idx-1, obj.Capacity) + 1;
        end
    end
end
