function [a0, a, b, signalsFS] = generateFourierSeriesAppx(tau, signals, ...
                                                                appxOrder)
%GENERATEFOURIERSERIESAPPX given a function, generate its fourier series
%approximation given the phase vector 'tau' (in 0 to 2\pi), the 'signal' as
%a function of the phase, and the approximation order.

    % iterate over the different signals provided
    numSignal = size(signals, 2);
    a0 = trapz(tau-pi, signals)/pi;
    a = nan(appxOrder, numSignal);
    b = nan(appxOrder, numSignal);
    A = nan([size(signals), numSignal]);
    B = nan([size(signals), numSignal]);
    for i = 1:numSignal
        for j = 1:appxOrder
            % ... compute coefficients
            a(j, i) = trapz(tau, signals(:,i).*cos(j*tau))/pi;
            b(j, i) = trapz(tau, signals(:,i).*sin(j*tau))/pi;
            % ... matrix to obtain approximated signal as a function of
            % ... phase
            A(:, j, i) = cos(j*tau);
            B(:, j, i) = sin(j*tau);
        end
    end

    % output a signal vector for plotting purposes
    signalsFS = nan(size(signals));
    for i = 1:numSignal
        signalsFS(:, i) = a0(i)/2*ones(size(signalsFS(:, i))); % init DC offset
        signalsFS(:, i) = signalsFS(:, i) + ... % add the fourier series
            reshape(A(:, :, i), size(A, 1), size(A, 2))*a(:, i) + ...% cos
            reshape(B(:, :, i), size(B, 1), size(B, 2))*b(:, i); % sin
    end

end

