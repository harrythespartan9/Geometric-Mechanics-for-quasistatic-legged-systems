% Function to generate the limb swing angle and angular rate trajectories 
% to simulate the system.
function [alpha_t, alphaDot_t] = computeSineShapeTrajectoriesV1(t, sHat, sP)
    alpha_t = sHat*[-cos(2*pi*t + sP(1));
                    +cos(2*pi*t + sP(2));
                    +cos(2*pi*t + sP(3));
                    -cos(2*pi*t + sP(4))];
    alphaDot_t = 2*pi*sHat*[+sin(2*pi*t + sP(1));
                            -sin(2*pi*t + sP(2));
                            -sin(2*pi*t + sP(3));
                            +sin(2*pi*t + sP(4))];
end