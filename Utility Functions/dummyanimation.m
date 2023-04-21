% This function runs a dummy animation for HAMR 6 body xyz and legs xyz dataset.
function dummyanimation(exp_data, v, appx_i)

    % figure out the figure limits
    temp = [exp_data.XYZCOM, exp_data.FRfootXYZ, exp_data.FLfootXYZ, exp_data.RLfootXYZ, exp_data.RRfootXYZ];
    raw_lim_low = [min(temp(1,:)) min(temp(2,:)) min(temp(3,:))]; raw_lim_low = raw_lim_low - 0.07*abs(raw_lim_low);
    raw_lim_high = [max(temp(1,:)) max(temp(2,:)) max(temp(3,:))]; raw_lim_high = raw_lim_high + 0.07*abs(raw_lim_high);
    ax_lim = [raw_lim_low(1) raw_lim_high(1) raw_lim_low(2) raw_lim_high(2) raw_lim_low(3) raw_lim_high(3)];

    % animate
    for i = 1:numel(exp_data.Time)
        if i == 1
            figure('units','pixels','position',[0 0 1920 1080],'Color','w')
        end
        subplot(4, 4, [1, 15]); % controllable isometric view-- try changing v in the input for a different view
        scatter3(exp_data.XYZCOM(1,i), exp_data.XYZCOM(2,i), exp_data.XYZCOM(3,i), 50, 'k', 'filled', 'o');
        if i == 1
            hold on; 
        end
        axis equal; axis(ax_lim); view(v);
        scatter3(exp_data.RRfootXYZ(1,i), exp_data.RRfootXYZ(2,i), exp_data.RRfootXYZ(3,i), 50, 'r', 'filled', 'o');
        scatter3(exp_data.FRfootXYZ(1,i), exp_data.FRfootXYZ(2,i), exp_data.FRfootXYZ(3,i), 50, 'm', 'filled', 'o');
        scatter3(exp_data.FLfootXYZ(1,i), exp_data.FLfootXYZ(2,i), exp_data.FLfootXYZ(3,i), 50, 'c', 'filled', 'o');
        scatter3(exp_data.RLfootXYZ(1,i), exp_data.RLfootXYZ(2,i), exp_data.RLfootXYZ(3,i), 50, 'b', 'filled', 'o');
        if nargin == 3
            scatter3(appx_i{1,i}, appx_i{1,i}, appx_i{1,i}, 50, 'r', 'filled', '+');
            scatter3(appx_i{2,i}, appx_i{2,i}, appx_i{2,i}, 50, 'm', 'filled', '+');
            scatter3(appx_i{3,i}, appx_i{3,i}, appx_i{3,i}, 50, 'c', 'filled', '+');
            scatter3(appx_i{4,i}, appx_i{4,i}, appx_i{4,i}, 50, 'b', 'filled', '+');
        end
        xlabel('x'); ylabel('y'); zlabel('z');

        subplot(4, 4, [4, 8]); % fixed top view
        scatter3(exp_data.XYZCOM(1,i), exp_data.XYZCOM(2,i), exp_data.XYZCOM(3,i), 50, 'k', 'filled', 'o');
        if i == 1
            hold on;
        end
        axis equal; axis(ax_lim); view([-90, 90]);
        scatter3(exp_data.RRfootXYZ(1,i), exp_data.RRfootXYZ(2,i), exp_data.RRfootXYZ(3,i), 50, 'r', 'filled', 'o');
        scatter3(exp_data.FRfootXYZ(1,i), exp_data.FRfootXYZ(2,i), exp_data.FRfootXYZ(3,i), 50, 'm', 'filled', 'o');
        scatter3(exp_data.FLfootXYZ(1,i), exp_data.FLfootXYZ(2,i), exp_data.FLfootXYZ(3,i), 50, 'c', 'filled', 'o');
        scatter3(exp_data.RLfootXYZ(1,i), exp_data.RLfootXYZ(2,i), exp_data.RLfootXYZ(3,i), 50, 'b', 'filled', 'o');
        if nargin == 3
            scatter3(exp_data.RRfootXYZ(1,i), exp_data.RRfootXYZ(2,i), exp_data.RRfootXYZ(3,i), 50, 'r', 'filled', '+');
            scatter3(exp_data.FRfootXYZ(1,i), exp_data.FRfootXYZ(2,i), exp_data.FRfootXYZ(3,i), 50, 'm', 'filled', '+');
            scatter3(exp_data.FLfootXYZ(1,i), exp_data.FLfootXYZ(2,i), exp_data.FLfootXYZ(3,i), 50, 'c', 'filled', '+');
            scatter3(exp_data.RLfootXYZ(1,i), exp_data.RLfootXYZ(2,i), exp_data.RLfootXYZ(3,i), 50, 'b', 'filled', '+');
        end
        xlabel('x'); ylabel('y');

        drawnow;
    end

end