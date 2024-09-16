function g = exponentiateLieAlgebraElement( gCirc )
%EXPONENTIATELIEALGEBRAELEMENT exponentiate the lie-algebra element
%provided
%   Given a lie-algebra element, we compute the exponential map-- we
%   explicity compute this result based on the irrotational case and the
%   rotational case

    % input checks
    errMsg = ['ERROR! The input velocity or the lie algebra element ' ...
            'provided should contain 3 columns of x, y, and theta velocity ' ...
            'numeric time series data (atlleast one timestep). Using that, ' ...
            'we shall then compute the group element or body frame ' ...
            'location. Refer to "se2_toyproblems_case_1_mobility.mlx" for ' ...
            'more details.'];
    if ~ismatrix(gCirc)
        error(errMsg);
    end
    if size(gCirc, 2) ~= 3
        if numel(gCirc) ~= 3
            error(errMsg);
        else
            gCirc = gCirc'; % convert to correct form
        end
    end
    if isempty(gCirc)
        error(errMsg);
    end
    if isa(gCirc, "sym") % if a symbol, error out
        error(errMsg);
    end
    nPoints = size(gCirc, 1);
    % ... ideally you would need more information on what caused the error,
    % ... something to flesh out later

    % if you're here, no issues with the input, so initialize the group
    % position (SE(2) body location) and quickly return the theta position
    % as the theta-circ velocity
    g = nan(size(gCirc)); g(:, 3) = gCirc(:, 3);

    % compute the exponential map on the lie algebra elements
    % ... case 1: just one element
    % ... case 2: rows of elements
    % ... subcases within each case handle no rotation and rotation cases
    switch nPoints
        case 1
            switch gCirc(3) == 0
                case 1
                    g(1:2) = gCirc(1:2);
                otherwise
                    g(1:2) = (... % transpose the result into row
                        computeTransVelPerturbMatrix...
                                (gCirc(3))*... % perturb translations
                                        gCirc(1:2)'... % transform them
                             )';
            end
        otherwise
            if size(gCirc, 1) ~= 3
                transposeFlag = true;
                gCirc = gCirc';
            end
            idxZero = (gCirc(3, :) == 0); % indices without rotational vel
            gCircPages = permute(gCirc, [1, 3, 2]); % convert velocities from 2D matrix into 3D pages 1)
            Mpages = nan(2, 2, nPoints); % init transformation matrix in NaN-valued pages 2)
            Mpages(:, :, idxZero) = repmat(eye(2, 2), 1, 1, nnz(idxZero)); % unity transformation in the irrotational case 3)
            Mpages(:, :, ~idxZero) = ... % for the rotational velocity case 4)
                computeTransVelPerturbMatrix(... % compute the transformation pages
                reshape(gCircPages(3, :, ~idxZero), 1, 1, nnz(~idxZero))... % get it into [3 x 1 x "nPages"] format
                                            );
            gPagesXY = pagemtimes(Mpages, ...
                reshape(gCircPages(1:2, :, :), [2, 1, nPoints]) ... % extract the translational components
                                ); % get the Lie group element pages
            gPages = nan(3, 1, nPoints);
            gPages(3, :, :) = gCircPages(3, :, :); % rotations transfer as such
            gPages(1:2, :, :) = gPagesXY; % get the translational components
            g = permute(gPages, [1, 3, 2]); % permute dimensions into a matrix similar to gCirc (only 2D) 5)
            if transposeFlag
                g = g'; % if transposed bring convert to original coords and return 6)
            end
    end

end

% % % % %  OLD 'OTHERWISE' case logic-- preserve until verified
% idxZero = (gCirc(:, 3) == 0); % indices without rotational vel
% if any(idxZero) % if at least one case is present
%     g(idxZero, 1:2) = gCirc(idxZero, 1:2);
% end
% if any(~idxZero) % if at least one rotational case is present
%     indNonZero = find(~idxZero); % get the index locations
%     for i = indNonZero'
%         g(i, 1:2) = (...
%             computeTransVelPerturbMatrix...
%                     (gCirc(i, 3))*...
%                             gCirc(i, 1:2)'...
%                  )';
%     end
% end

%% AUXILIARY FUNCTIONS

% compute the arc-travel perturbation matrix to the translational
% velocities obtained using the rotation velocity
% ... the radius of this arc is the translational velocity divided by the
% ... rotational velocity
function M = computeTransVelPerturbMatrix(thDot)
    M = [sin(thDot),   cos(thDot)-1;
         1-cos(thDot), sin(thDot)  ]./thDot;
end
