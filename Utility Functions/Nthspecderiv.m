function in_spec_dot = Nthspecderiv(in, t, N)
%SPECDERIV Nth fourier/spectral derivative of an input time series
%   Given an input time series and time vector, the Nth spectral/fourier derivative is computed and returned.
    if iscell(in)
        in_spec_dot = cell(size(in));
        for i = 1:numel(in)
            if numel(in{i}) ~= numel(t)
                error('ERROR! The input time series has a different length from the input time vector.');
            end
            in_spec_dot{i} = compnthspecderiv(in{i}, t, N);
        end
    else
        if numel(in) ~= numel(t)
            error('ERROR! The input time series has a different length from the input time vector.');
        end
        in_spec_dot = compnthspecderiv(in, t, N);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HELPER FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function in_spec_dot = compnthspecderiv(in, t, n)
%COMPNTHSPECDERIV This function computes the nth spectral derivative of the given time series data
%   Given a time vector and input time series, this function computes the nth spectral derivative. 
%   Also, this function inherently assumes fixed sampling rate. 
%   TO BE CHECKED LATER: for odd powers, should W(N/2) = 0?
    
    % % condition inputs
    % in = in(:);
    N = numel(t);
    t_elapsed = t(end) - t(1);

    % compute
    inhat = fftshift(fft(in));
    df = 1/t_elapsed;
    if ~mod(n,2) % computed as 2*pi*f
        W = 2*pi*df*(-N/2:N/2-1); % if N even; transpose at the end to make it a col matrix
    else
        W = 2*pi*df*(-(N-1)/2:(N-1)/2); % if N odd
    end
    in_spec_dot = real( ifft( ifftshift( inhat.*(1i*W).^n ) ) );

end