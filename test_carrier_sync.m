clear; clc;

M   = 4;
bps = log2(M);
Ns  = 2000;               % number of symbols to test

modQPSK = modulators.QpskModulator();
demQPSK = demodulators.QpskDemodulator();

% ----- 1) Generate random bits and QPSK symbols -----
bits = randi([0 1], Ns*bps, 1);
s    = modQPSK.modulate(bits);    % ideal symbols, unit power

% ----- 2) Inject known CFO + phase offset -----
phi0   = pi/4;           % constant phase offset
wSym   = 0.05;           % rad/symbol CFO (quite noticeable)
n      = (0:Ns-1).';
s_cfo  = s .* exp(1j*(phi0 + wSym*n));

% (optional) Add some noise
EsN0dB = 15;
EsN0   = 10^(EsN0dB/10);
noise  = (randn(Ns,1) + 1j*randn(Ns,1)) / sqrt(2*EsN0);
r      = s_cfo + noise;

% ----- 3) Run our carrier synchronizer -----
carSync = sync.DecisionDirectedCarrierSync( ...
    'ModulationOrder',        M, ...
    'SamplesPerSymbol',       1, ...
    'DampingFactor',          0.707, ...
    'NormalizedLoopBandwidth',0.01);  % fairly tight

y = carSync.process(r);   % corrected symbols

% ----- 4) Measure BER before and after -----
bits_rx_no_sync = demQPSK.demodulateHard(r);
bits_rx_sync    = demQPSK.demodulateHard(y);

ber_no_sync = mean(bits_rx_no_sync ~= bits);
ber_sync    = mean(bits_rx_sync    ~= bits);

fprintf('BER without carrier sync: %.3e\n', ber_no_sync);
fprintf('BER with    carrier sync: %.3e\n', ber_sync);
