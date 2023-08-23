function t_anim = extractTimeVectorForCompGaitAnimation(dnum, gcV, availF, cond)
%EXTRACTTIMEVECTORFORCOMPGAITANIMATION computes the plotting time vector for a composite gait animation (every middle gait repetition of a particular gait is 
% slowed down by 2x)

    % iterate and compute the animation time vector
    for i = 1:numel(gcV)
        
        % number of frames left (update if "i ~= 1")
        if i == 1
            idx_start = 1; % initialize the starting index if this is the first gait cycle
            dnumTaken = 0; 
            framesTaken = 0;
            dnumLeft = dnum; % initially no frames have been assigned yet
            framesLeft = availF;
            t_anim = []; % empty initialization
        end
        dnumLeft = dnumLeft - dnumTaken; % update to number of total frames
        framesLeft = framesLeft - framesTaken; % update to number of assignable frames

        % frame number info
        i_framesTotalNow = gcV(i)./sum(gcV(i:end))*dnumLeft; % total frames in the current gait cycle
        i_framesAllowedNow = round(gcV(i)./sum(gcV(i:end))*framesLeft); % total plottable frames in the current gait cycle
        
        % get the gait cycle of interest
        switch cond
            case "first"
                jStar = 1;
            case "mid"
                jStar = ceil(gcV(i)/2); % choose a middle gait cycle
            case "last"
                jStar = gcV(i);
        end
        
        % iterate and obtain the plot indices for current gait cycle
        j_fixed = floor(i_framesTotalNow/gcV(i));
        jStar_framesAllowedNow = floor(2*i_framesAllowedNow/gcV(i));
        j_framesAllowedNow = floor((i_framesAllowedNow - jStar_framesAllowedNow)/(gcV(i)-1));
        
        idx_begin = idx_start;
        for j = 1:gcV(i)
            switch j == jStar
                case 0
                    t_anim = [ t_anim,  round(linspace(idx_start, idx_start-1+j_fixed, j_framesAllowedNow)) ];
                case 1
                    t_anim = [ t_anim,  round(linspace(idx_start, idx_start-1+j_fixed, jStar_framesAllowedNow)) ];
            end
            if j ~= gcV(i)
                idx_start = t_anim(end) + 1;
            end
        end
        idx_end = idx_begin - 1 + i_framesTotalNow; % compute the final index
        idx_start = idx_end + 1; % reinitialize idx_start
        dnumTaken = i_framesTotalNow; % update the number of frames taken from the total number of frames
        framesTaken = i_framesAllowedNow; % update the same for plottable frames

    end
    

end

