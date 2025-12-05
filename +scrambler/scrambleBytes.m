function bytesOut = scrambleBytes(bytesIn, seedBits)
    if ~isa(bytesIn, 'uint8')
        error('scrambleBytes:InputType', 'bytesIn must be uint8.');
    end

    b = bytesIn(:);  % column

    seedBits = logical(seedBits(:).');
    if numel(seedBits) ~= 15
        error('scrambleBytes:SeedLen', 'seedBits must have length 15.');
    end

    Nbytes = numel(b);
    if Nbytes == 0
        bytesOut = b;
        return;
    end

    % Generate 8 bits per byte
    Nbits = 8 * Nbytes;
    pnBits = scrambler.lfsrPN(Nbits, seedBits);  % logical [Nbits x 1]

    % Reshape into [Nbytes x 8], each row = 8 bits (MSB first)
    pnBits = reshape(pnBits, 8, []).';   % [Nbytes x 8]

    % Convert bits -> uint8 mask (bit7 ... bit0)
    weights = double([128 64 32 16 8 4 2 1]).';
    mask = uint8( double(pnBits) * weights );

    % XOR in byte domain
    out = bitxor(b, mask);

    % Keep original shape
    if isrow(bytesIn)
        bytesOut = reshape(out, 1, []);
    else
        bytesOut = out;
    end
end
