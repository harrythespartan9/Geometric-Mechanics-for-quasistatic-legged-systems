function [shortT, bodyTrajsOut, stanceBodyTrajsOut] = ...
    computeBodyTrajSnapshots...
    (params, transforms, gaitConfigTrajsIn, prctPts)
%COMPUTEBODYTRAJSNAPSHOTS Compute the body trajectory snapshots with SE(2) 
%transforms
%   This function computes snapshots of the body trajectory using SE(2) 
%   transformations. It is used for visualizing the planar motion of a 
%   body, where SE(2) represents the special Euclidean group of rigid body 
%   transformations (translation and rotation).
%
%   Notes:
%       - For correct function calls and examples, check 
%         "se2_fixedCLARImobility_slipEstimates.mlx" in the projects root
%         folder for more details.
%       - Ensure that the input trajectory data is properly formatted.
%       - This function is part of the utility functions for plotting in 
%         the Geometric Mechanics for Quasistatic Legged Systems project.

    % if the fraction of points in the trajectory to plot is not provided,
    % assume it is 10 prct of all kinematic gait-time steps
    if nargin < 4
        prctPts = 0.1;
    end

    % unpack system design params
    na = params.na; nl = params.nl; nalpha_b = params.nalpha_b;

    % unpack the transforms
    fh_e__boxCorners = transforms.fh_e__boxCorners;
    fz_boxCorners = transforms.fz_boxCorners;
    fh_e__i = transforms.fh_e__i;

    % lets define a selection matrix to help choose just the translational
    % components (used inside the loop below)
    Cxy = [eye(2, 2), zeros(2, 1)];

    % number of stance phases in the gait cycle and interval duration for
    % each stance
    numStances = 2; stanceIntervalLength = 1/numStances;

    % init output containers and unpack the gait parameters for each gait 
    % cycle
    numTrajs = numel(gaitConfigTrajsIn); % number of trajectories provided
    bodyTrajsOut.body = cell(1, numTrajs); % init output
    bodyTrajsOut.foot = cell(1, numTrajs);
    stanceBodyTrajsOut = cell(1, numTrajs);
    shortT = cell(1, numTrajs);
    for i = 1:numTrajs % iterate
        % unpack the configuration trajectory
        gFullT = gaitConfigTrajsIn{i}(:, 1);
        gFullConfig = gaitConfigTrajsIn{i}(:, 2:end);
        numStepsFull = numel(gFullT);
        % interpolate the configuration trajectory to just the points
        % needed
        numStepsShort = floor(prctPts*numStepsFull);
        shortT{i} = linspace(0, 1, numStepsShort)';
        shortConfig = interp1( gFullT, gFullConfig, shortT{i}, "pchip" );
        % segement the interpolated configuration into components
        gX = shortConfig(:, 1);
        gY = shortConfig(:, 2);
        gTheta = shortConfig(:, 3);
        gAlpha = shortConfig(:, 4:end); numLimbs = size(gAlpha, 2);
        % compute the positions and displacements to quiver plot the body's
        % bounding box and scatter plot the foot locations at each 
        % timestep the snapshot is needed
        bodyPosNow = nan(2, 4, numStepsShort);
        bodyDispNow = nan(2, 4, numStepsShort);
        footLocNow = nan(2, numLimbs, numStepsShort);
        for t = 1:numStepsShort
            bodyPosNow(:, :, t) = ... % body position @ t
                Cxy*fh_e__boxCorners(na, nl, nalpha_b, ...
                                        gX(t), gY(t), gTheta(t));
            bodyDispNow(:, :, t) = ...% body displacement @ t
                Cxy*fz_boxCorners(na, nl, nalpha_b, ...
                                    gX(t), gY(t), gTheta(t));
            for j = 1:numLimbs
                footLocNow(:, j, t) = ... % current foot location @t
                    Cxy* ...
                        fh_e__i{j}(na, nl, nalpha_b, ...
                                        gX(t), gY(t), gTheta(t), ...
                                            gAlpha(t, j));
            end % END LIMB ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        end % END TIMESTEP ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % pack everything into the output containers in the appropriate
        % format
        bodyTrajsOut.body{i}.X = reshape( bodyPosNow(1, :, :), ...
                                   [size(bodyPosNow, 2), numStepsShort] )';
        bodyTrajsOut.body{i}.Y = reshape( bodyPosNow(2, :, :), ...
                                   [size(bodyPosNow, 2), numStepsShort] )';
        bodyTrajsOut.body{i}.U = reshape( bodyDispNow(1, :, :), ...
                                   [size(bodyPosNow, 2), numStepsShort] )';
        bodyTrajsOut.body{i}.V = reshape( bodyDispNow(2, :, :), ...
                                   [size(bodyPosNow, 2), numStepsShort] )';
        for j = 1:numLimbs
            bodyTrajsOut.foot{i}{j}.X = reshape(footLocNow(1, j, :), ...
                                                    [numStepsShort, 1]);
            bodyTrajsOut.foot{i}{j}.Y = reshape(footLocNow(2, j, :), ...
                                                    [numStepsShort, 1]);
        end % END LIMB ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % More specifically compute the same trajectories at the end of two
        % stance phases
        stanceBodyTrajsOut{i} = cell(1, numStances);
        for k = 1:numStances % just 2 stance phases for now
            % setup
            shortStanceConfig = interp1(gFullT, gFullConfig, ... % interp
                k*stanceIntervalLength, "pchip");
            gX = shortStanceConfig(:, 1); % extract
            gY = shortStanceConfig(:, 2);
            gTheta = shortStanceConfig(:, 3);
            gAlpha = shortStanceConfig(:, 4:end);
            % compute
            stanceBodyPosNow = ... % body position @ end of stance (EOS)
                Cxy*fh_e__boxCorners(na, nl, nalpha_b, gX, gY, gTheta);
            stanceBodyDispNow = ...% body displacement @ EOS
                Cxy*fz_boxCorners(na, nl, nalpha_b, gX, gY, gTheta);
            stanceFootLocNow = nan(2, numLimbs);
            for j = 1:numLimbs
                stanceFootLocNow(:, j) = ... % current foot's loc @ EOS
                    Cxy* ...
                        fh_e__i{j}(na, nl, nalpha_b, ...
                                        gX, gY, gTheta, gAlpha(j));
            end % END LIMB ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % assign
            stanceBodyTrajsOut{i}{k}.body.X = stanceBodyPosNow(1, :);
            stanceBodyTrajsOut{i}{k}.body.Y = stanceBodyPosNow(2, :);
            stanceBodyTrajsOut{i}{k}.body.U = stanceBodyDispNow(1, :);
            stanceBodyTrajsOut{i}{k}.body.V = stanceBodyDispNow(2, :);
            stanceBodyTrajsOut{i}{k}.foot = cell(1, numLimbs);
            for j = 1:numLimbs
                stanceBodyTrajsOut{i}{k}.foot{j}.X =stanceFootLocNow(1, j);
                stanceBodyTrajsOut{i}{k}.foot{j}.Y =stanceFootLocNow(2, j);
            end % END LIMB ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            stanceBodyTrajsOut{i}{k}.COM.X = gX;
            stanceBodyTrajsOut{i}{k}.COM.Y = gY;
            stanceBodyTrajsOut{i}{k}.COM.TH = gTheta;
        end % END STANCE ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    end % END CURRENT GAIT ITERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

end

