function [modulator] = getMmodulator(modulatorName)

modulatorNameLower = lower(modulatorName);

switch modulatorNameLower
    case 'qpsk'
        modulator = modulators.QpskModulator();
    case '16-qam'
        modulator = modulators.Qam16Modulator();
    otherwise
        fprintf('The chosen modulator (%s) is unkown.\n', modulatorName);
        assert(false);
end
end

