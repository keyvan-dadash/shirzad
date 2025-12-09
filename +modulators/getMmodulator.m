function [modulator] = getMmodulator(modulatorName)

modulatorNameLower = lower(modulatorName);

switch modulatorNameLower
    case 'qpsk'
        modulator = modulators.QpskModulator();
    case '16-qam'
        modulator = modulators.Qam16Modulator();
    case '64-qam'
        modulator = modulators.Qam64Modulator();
    case '256-qam'
        modulator = modulators.Qam256Modulator();
    otherwise
        fprintf('The chosen modulator (%s) is unkown.\n', modulatorName);
        assert(false);
end
end

