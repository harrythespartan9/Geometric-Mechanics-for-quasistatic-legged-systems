function Vq = interpColorAndCondition(X, V, Xq)
%INTERPCOLORANDCONDITION this function interpolates the given colormap to
%the specific requested value (has "interp1" args) and conditions the
%values between 0 and 1. We use "pchip" as the default interpolation method
%to preserved the structure of the provided colormap "V".

    % Interpolate
    Vq = interp1(X, V, Xq, "pchip");

    % Condition
    % ... RGB colors need all elements in [0, 1]
    Vq(Vq > 1) = 1; Vq(Vq < 0) = 0;

end

