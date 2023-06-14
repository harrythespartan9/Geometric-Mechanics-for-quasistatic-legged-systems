% This script extracts the SE(3) kinematic data from HAMR experiments and estimates the lift and swing values for each leg. If the output already exists, then
% use that to append more information.
% Inputs: 1) dataname char array to be called by the load function... 2) SE(3) kinematics of HAMR 6... 3) type of gait needed 'char' array... 4) experiment 
% number-- corresponds to the required frequency (array start: lowest and array end: highest)
function out = estHAMRtraj(char, kin, gait_char, out)

    % Define constants/transforms
    m2mm = 1e3; bl = kin.params.bl;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % obtain the experiment of interest %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nargin ~= 4
        out = [];
        load(char);  % load the dataset based on the input character
        exp_num = []; f = [];

        switch isfield(Metrics, 'TailConfig') % check if Tail is present and assign a flag -- defined before the loop, so efficient
            case 0
                flag = false;
            case 1
                flag = true;
        end

        for i = 1:numel(Metrics)
            switch flag
                case false
                    cond = string(Metrics(i).Gait) == gait_char;
                case true
                    cond = string(Metrics(i).Gait) == gait_char && ~Metrics(i).TailConfig;
            end
            if cond % supports 'Trot', 'Bound', 'Walk', and 'Jump' -- eliminates trials with tail in dataset_2
                exp_num = [exp_num, i];
                f = [f, Metrics(i).Frequency];
            end
        end

        [c, ~, ic] = unique(f);
        out.exp_num = {}; out.f = {};
        for i = 1:numel(c)
            out.f{i} = c(i);
            out.exp_num{i} = exp_num( ic == i );
        end
        out.traj = cell(1, numel(out.f));
    end

    if ~isfield(out, 'exp_num')
        error('ERROR! The "exp_num" field cant be empty.');
    end
%     if ~isfield(out, 'f') % not required right now-- not doing anything frequency dependent
%     end

    i_str = {'FR', 'FL', 'RL', 'RR'}; % correspondence between out leg frame naming and dataset

    for expn = 1:numel(out.exp_num) % for each frequency
        for idx = 1:numel(out.exp_num{expn}) % for each experiment
            if idx ~= 1
                clear traj_est; % clear the trajectory estimation from the previous run
            end
            traj = Metrics(out.exp_num{expn}(idx));
            b = {m2mm*traj.XYZCOM(1,:); m2mm*traj.XYZCOM(2,:); m2mm*traj.XYZCOM(3,:);...
                (traj.Roll(:))'; traj.Pitch(:)'; traj.Yaw(:)'};
            ht3_e__i_exp = cell(4, 1); C_i_exp = ht3_e__i_exp; temp = [];
            for j = 1:4
                ht3_e__i_exp{j} = m2mm*traj.([i_str{j}, 'footXYZ']);    % foot location in rest frame
                idx_c = ( ht3_e__i_exp{j}(3, :) < 0.01*bl );            % indices where the current foot is below the 1% of BL z-threshold        
                C_i_exp{j} = idx_c;                                     % the indicies calculated earlier will determine the contact
%                 ht3_e__i_exp{j}(3, idx_c) = 0;                          % zero-out the z-value at these indicies-- not the best idea; better to have the raw z-data
                temp = [temp; abs(diff(C_i_exp{j}))];
            end
            delC_i_exp = (sum(temp, 1) > 0);
            [S_exp, traj_col_exp] = compute_submanifold_color(C_i_exp, delC_i_exp, kin);
            r_trot = apprxSFBswingliftSE3(b, ht3_e__i_exp, kin);
            t = traj.Time;
            r = vecSE3traj(r_trot);
            traj_est = computeSE3trajectory(r, b, kin);
            traj_est.exp.t = t;
            traj_est.exp.r = r;
            traj_est.exp.b = b;
            traj_est.exp.tnum = numel(traj.Time);
            traj_est.exp.ht3_e__i = ht3_e__i_exp;
            traj_est.exp.C_i = C_i_exp;
            traj_est.exp.delC_i = delC_i_exp;
            traj_est.exp.S = S_exp;
            traj_est.exp.col = traj_col_exp;
            traj_est.bl = kin.params.bl;
            out.traj{expn}{idx} = traj_est;
        end
    end

end