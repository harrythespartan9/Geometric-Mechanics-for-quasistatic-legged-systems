% This function animates the given SE3(3) system by plotting snapshots at different points ion time.
function animateSE3sys(traj, v)

    % data
    Name = v.Name;
    fps = v.FrameRate;
    qual = v.Quality;
    camlos = v.View;
    vidF = v.VidF;
    t = traj.exp.tnum;
    
    % initialize video
    if vidF
        video = VideoWriter([Name,'.mp4'],'MPEG-4');
        video.FrameRate = fps;
        video.Quality = qual;
        open(video);
    end

    % plot
    f = figure('units','pixels','position',[0 0 1920 1080],'Color','w');
    set(f,'Visible','on');
    
    for i = 1:t

        if i ~= 1
            for j = 1:numel(h)
                delete(h{j}); % delete plots
            end
        end

        h = plotSE3snapshot(gca, traj, camlos, i); % SE(3) snapshot
        drawnow;
        if isfield(v, 'Speed')
            title(gca, ['Speed = '...
                num2str(v.Speed) 'x'],...
                'FontSize', 25);
        end

        if vidF
            writeVideo(video,getframe(f)); % get the frame
        end

    end

    if vidF
        close(video); % close the video
    end

end