function [a0, a, b, signalFS] = generateFourierSeriesAppx(tau, signalCols, appxOrder)
%GENERATEFOURIERSERIESAPPX given a function, generate its fourier series
%approximation given the phase vector 'tau' (in 0 to 2\pi), the 'signal' as
%a function of the phase, and the approximation order.

    % iterate over the different signals provided
    numSignal = size(signalCols, 2);
    a0 = trapz(tau-pi, signalCols)/pi;
    a = nan(appxOrder, numSignal);
    b = nan(appxOrder, numSignal);
    for i = 1:size(signalCols, 2)
    end

    % output a signal vector for plotting purposes
    
end

