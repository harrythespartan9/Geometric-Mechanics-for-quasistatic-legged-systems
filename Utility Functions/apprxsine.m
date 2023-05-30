function r_out = apprxsine(in)
%APPRXSINE approximates the input time-series as a pure sinusoid
%   The input cell array contains the periodic time-series to be approximated, the sampling frequency, and the cycle time period.
    
    % Unpack
    r = in{1};                                                        % time-series
    Fs = in{2};                                                       % sampling frequency
    T = in{3};                                                        % time period                          

    % Iterate and compute
    if iscell(r)
        r_out = cell(size(r));
        for i = 1:numel(r)
            [pos_p, t_p] = findpeaks(r{i}, Fs, 'MinPeakDistance', 0.96*T);  % pos peaks-- 4% thresh-held peak location should match stride time period
            [pos_m, t_m] = findpeaks(-r{i}, Fs, 'MinPeakDistance', 0.96*T); pos_m = -pos_m; % neg peaks
            yamp = (mean(pos_p) - mean(pos_m))/2;
            y_dc = (mean(pos_p) + mean(pos_m))/2;
            if t_p(1) < t_m(1)
                tau = t_p(1); mul = 1;
            else
                tau = t_m(1); mul = -1; % cosine function definition-- mul*yamp*cos(2*pi*f*(t - tau)) + y_dc {OUTPUT IN THE SAME ORDER}
            end
            r_out{i} = [mul, yamp, 1/T, tau, y_dc]';
        end
    else
        [pos_p, t_p] = findpeaks(r, Fs, 'MinPeakDistance', 0.96*T);
        [pos_m, t_m] = findpeaks(-r, Fs, 'MinPeakDistance', 0.96*T); pos_m = -pos_m;
        yamp = (mean(pos_p) - mean(pos_m))/2;
        y_dc = (mean(pos_p) + mean(pos_m))/2;
        if t_p(1) < t_m(1)
            tau = t_p(1); mul = 1;
        else
            tau = t_m(1); mul = -1;
        end
        r_out = [mul, yamp, 1/T, tau, y_dc]'; % sign, amplitude, frequency, phase offset, dc offset
    end

end

