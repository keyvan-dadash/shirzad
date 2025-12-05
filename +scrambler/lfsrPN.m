function pn = lfsrPN(N, state)
    state = logical(state(:).'); % row
    L = numel(state);
    if L ~= 15
        error('lfsrPN:StateLength', 'Expected state length = 15.');
    end
    if ~any(state)
        error('lfsrPN:AllZeroState', 'LFSR state cannot be all zeros.');
    end

    pn = false(N,1);

    for n = 1:N
        % Output bit = last stage
        pn(n) = state(end);

        % Feedback = xor of taps at stages 15 and 14
        feedback = xor(state(end), state(end-1));

        % Shift right, insert feedback
        state(2:end) = state(1:end-1);
        state(1)     = feedback;
    end
end
