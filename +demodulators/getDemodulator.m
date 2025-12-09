% template function to get different qam
function [demodulator] = getDemodulator(demodulatorName)

demodulatorNameLower = lower(demodulatorName);

switch demodulatorNameLower
    case 'qpsk'
        demodulator = demodulators.QpskDemodulator();
    case '16-qam'
        demodulator = demodulators.CPPQam16Demodulator();
    case '64-qam'
        demodulator = demodulators.CPPQam64Demodulator();
    case '256-qam'
        demodulator = demodulators.CPPQam256Demodulator();
    otherwise
        fprintf('The chosen demodulator (%s) is unkown.\n', demodulatorName);
        assert(false);
end
end

