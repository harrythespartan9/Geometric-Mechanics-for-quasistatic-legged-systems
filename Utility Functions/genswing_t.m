function out = genswing_t(t, p)
%GENSWING_T generate a pure sinusoid
%   Given the current time-step 't' and construction parameters 'p', generate a sinusoid. 'p' include amplitude, dc offset, frequency, phase offset, direction.
    out = p(1)*p(2)*cos(2*pi*p(3)*(t - p(4))) + p(5);
end

