function out = pushforwardvel_rest2body(in)
%PUSHFORWARDVEL_REST2BODY pushforward the body velocity from the rest frame to body frame
%   Given a rest frame body velocity and trajectory, pushforward the body velocity to the body frame (g^-1 \dot{g})

    % unpack
    b = in{1};
    b_dot_rest = in{2};

    % get 'g' for the pushforward (taking transpose will do the left-pushforward on the body velocity in the rest frame)
    pullback = rot_SE2(b{3}); % using yaw

    % initialize, iterate, and compute
    out = cell(3, 1);
    for i = 1:numel(pullback)
        temp = transpose(pullback{i})*retrieve_ithTimeStepData(b_dot_rest, i); % pushforward = transpose(pullback)
        out = deposit_ithTimeStepData(temp, i, out);
    end

end

