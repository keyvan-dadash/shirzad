function vPunct = puncture78_k3(vMother)
    vMother = vMother(:).';
    if mod(numel(vMother), 2) ~= 0
        error('puncture78_k3:InputLength', ...
              'Mother code length must be even (2 bits per step).');
    end

    T = numel(vMother) / 2;  % trellis steps

    % reshape into [2 x T]: row1 = g0, row2 = g1
    V = reshape(vMother, 2, T);

    P = [1 1 0 1 0 1 1;
         1 0 1 0 1 0 0];

    reps   = ceil(T / 7);
    P_rep  = repmat(P, 1, reps); 
    P_rep  = P_rep(:, 1:T);

    % Apply mask in column-major order
    keepMask = P_rep(:) ~= 0;
    allBits  = V(:);

    vPunct = allBits(keepMask).';  % row vector of kept bits
end
