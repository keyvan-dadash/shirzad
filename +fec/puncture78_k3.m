function vPunct = puncture78_k3(vMother)
    vMother = vMother(:).';
    if mod(numel(vMother), 2) ~= 0
        error('puncture78_k3:InputLength', ...
              'Mother code length must be even (2 bits per step).');
    end

    T = numel(vMother) / 2;  % trellis steps

    % reshape into [2 x T]: row1 = g0, row2 = g1
    V = reshape(vMother, 2, T);

    % 7/8 puncturing pattern over 7 trellis steps:
    % rows: [g0; g1], columns: step mod 7
    P = [1 1 0 1 0 1 1;   % g0 kept?
         1 0 1 0 1 0 0];  % g1 kept?

    % Repeat pattern horizontally to cover all T steps,
    % then truncate to exactly T columns.
    reps   = ceil(T / 7);
    P_rep  = repmat(P, 1, reps);   % [2 x (7*reps)]
    P_rep  = P_rep(:, 1:T);        % [2 x T]

    % Apply mask in column-major order
    keepMask = P_rep(:) ~= 0;
    allBits  = V(:);

    vPunct = allBits(keepMask).';  % row vector of kept bits
end
