function out = genswingrate_t(t, p)
%GENSWINGRATE_T generate a pure sinusoid
%   Given the current time-step 't' and construction parameters 'p', generate a sinusoid. 'p' include amplitude, dc offset, frequency, phase offset, direction.
    out = -2*pi*p(3)*p(1)*p(2)*sin(2*pi*p(3)*(t - p(4)));
end