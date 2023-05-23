function [out, ti] = shapeinterpft(in, f, t)
%SHAPEINTERPFT this function interpolates the shape-space trajectory as a periodic function
%   Using interpft function, we use fft to 1D interpolate each shape-element trajectory. This ofc comes with the periodicity assumption and if the trajectory is
%   ill-conditioned with noise, this interpolation might not be suited.

    switch iscell(in)

        case 0

            dt = mean(diff(t));
            N = (t(end) - t(1))*f*length(t); % scale the subsamlpling by numStrides*length_t

            out = interpft(in,N);

            dti = dt*length(t)/N;
            ti = t(1):dti:(t(end)+1e-3);
            out = out(1:length(ti));

        case 1

            dt = mean(diff(t));
            N = (t(end) - t(1))*f*length(t);
            dti = dt*length(t)/N;
            ti = t(1):dti:(t(end)+1e-3); % easier to perform this compute outside the loop

            out = cell(size(in));

            for i = 1:numel(in)

                temp = interpft(in{i},N);
                out{i} = temp(1:length(ti));

            end

    end



end