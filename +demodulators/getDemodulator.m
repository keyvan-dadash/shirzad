function [demodulator] = getDemodulator(demodulatorName)

demodulatorNameLower = lower(demodulatorName);

switch demodulatorNameLower
    case 'qpsk'
        demodulator = demodulators.QpskDemodulator();
    case '16-qam'
        demodulator = demodulators.Qam16Demodulator();
    case '32-qam'
        demodulator = demodulators.Qam32Demodulator();
    otherwise
        fprintf('The chosen demodulator (%s) is unkown.\n', demodulatorName);
        assert(false);
end
end

