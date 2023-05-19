function in_dot = timeseriesderivatives(in, t, chc)
%TIMESERIESDERIVATIVES This function computes the time-derivatives for an input time series 'in' and time vector 't'.
%   Given the inputs, this function chooses between forward difference (chc == 1), fourier derivative (assuming the signal is periodic; chc == 2), and a 
%   digital FIR (finite-impulse-response) differentiating filter (chc == 3). Also, if the forward difference

    switch chc

        case 1 % backward difference
            
            in_dot = bwdDiffDeriv(in, t);

        case 2 % spectral derivative
            
            in_dot = specderiv(in, t);

        case 3 % digital FIR dervative filter

            in_dot = firdiff(in, t);

    end

end

