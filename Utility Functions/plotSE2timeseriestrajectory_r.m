function f = plotSE2timeseriestrajectory_r(traj, betaF, r_est)
%PLOTSE2TIMESERIESTRAJECTORY_R plot the shape trajectories of an SE(2), walking quadrupedal system
%   This function plots the shape timeseries/trajectories for a given run of a quadrupedal system-- this trajectory is obtained from experimentally measure
%   kinematic dataset. The estimate trajectory is obtained through denoising the original swing trajectory.
    
    r = traj.exp.r; t = traj.exp.t;
    for i = 1:4                                                        %%%%%%%%%%%% CONTACT TIME-SERIES
        if i == 1
            if betaF
                col = 2;
                font_size = 12;                                        %%%%%%%%%%%% PLOT PARAMETERS (tweak this to get the desired effect)
            else
                col = 1;
                font_size = 15;
            end
            f = figure('units','pixels','position',[0 0 1920 1080],'Color','w');
            % set(gcf,'Visible','on');                                             % pop-out the current figure
            tiledlayout(4, col, 'TileSpacing', 'tight', 'Padding', 'tight');
        end

        % swing trajectory
        if betaF
            swing_idx = 2*i -1;
        else
            swing_idx = i;
        end
        ax = nexttile(swing_idx);
        plot(t, rad2deg(r{2*i -1}), '-', 'LineWidth', 2.0, 'Color', 'k'); grid on;
        ax.FontSize = font_size;
        if nargin > 2                                                   % if the estimated shape trajectory is also input
            hold on;
            plot(t, rad2deg(genswing_t(t, r_est{2*i -1})), ':', 'LineWidth', 2.0, 'Color', [77, 77, 77]/255);
        end
        if i == 1
            xticklabels('');
        elseif i == 4
            xlabel('$$t$$', 'Interpreter', 'latex', 'FontSize', font_size);
        else
            xticklabels('');
        end
        ylabel(['$$\alpha_' num2str(i) '$$'], 'Interpreter', 'latex', 'FontSize', font_size); 
        % yticks([0 1]);

        % lift trajectory
        if betaF
            ax = nexttile(2*i);
            plot(t, rad2deg(r{2*i}), '-', 'LineWidth', 2.0, 'Color', 'k'); grid on; %  yticklabels('');
            ax.FontSize = font_size;
            if nargin > 2                                                   % if the estimated shape trajectory is also input
                hold on;
                plot(t, rad2deg(genswing_t(t, r_est{2*i})), ':', 'LineWidth', 2.0, 'Color', [77, 77, 77]/255);
            end
            if i == 1
                xticklabels('');
            elseif i == 4
                xlabel('$$t$$', 'Interpreter', 'latex', 'FontSize', font_size);
            else
                xticklabels('');
            end
            ylabel(['$$\beta_' num2str(i) '$$'], 'Interpreter', 'latex', 'FontSize', font_size); 
            % yticks([0 1]);
        end

    end
end

