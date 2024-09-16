function gCirc = logMapOfALieGroupElement( g )
%LOGMAPOFALIEGROUPELEMENT take the logarithmic map of a Lie group element
%to obtain a Lie algebra element (typically body velocity)
%   Given a Lie group element, we compute the logarithmic map-- we
%   explicity compute this result based on the irrotational case and the
%   rotational cases.

    % argument checks
    errMsg = ['ERROR! The input displacement or the Lie group element ' ...
            'should contain 3 columns of x, y, and theta displacement-' ...
            'numeric time series data (atlleast one timestep). Using that, ' ...
            'we shall then compute the Lie algebra element or body ' ...
            'velocity. Refer to "se2_toyproblems_case_1_mobility.mlx" for ' ...
            'more details.'];
    if ~ismatrix(g)
        error(errMsg);
    end
    if size(g, 2) ~= 3
        if numel(g) ~= 3
            error(errMsg);
        else
            g = g'; % convert to correct form
        end
    end
    if isempty(g)
        error(errMsg);
    end
    if isa(g, "sym") % if a symbol, error out
        error(errMsg);
    end
    nPoints = size(g, 1);
    % ... ideally you would need more information on what caused the error,
    % ... something to flesh out later

    % if you're here, no issues with the input argument, so initialize the 
    % lie group (SE(2)) element and quickly return the theta 
    % displacement as is as the rotational subspace of SE(2) is flat
    gCirc = nan(size(g)); gCirc(:, 3) = g(:, 3);

    % compute the exponential map on the lie algebra elements
    % ... case 1: just one element
    % ... case 2: rows of elements
    % ... subcases within each case handle no rotation and rotation cases
    switch nPoints
        case 1
            switch g(3) == 0
                case 1
                    gCirc(1:2) = g(1:2);
                otherwise
                    gCirc(1:2) = (... % transpose the result into row
                        computeInvOfTransVelPerturbMatrix...
                                (g(3))*... % perturb translations
                                    g(1:2)'... % transform them
                             )';
            end
        otherwise
            if size(g, 1) ~= 3
                transposeFlag = true;
                g = g';
            end
            idxZero = (g(3, :) == 0); % indices without rotational displacement
            gPages = permute(g, [1, 3, 2]); % convert displacement from 2D matrix into 3D pages 1)
            MinvPages = nan(2, 2, nPoints); % init transformation matrix in NaN-valued pages 2)
            MinvPages(:, :, idxZero) = repmat(eye(2, 2), 1, 1, nnz(idxZero)); % Id transformation for irrotational case 3)
            MinvPages(:, :, ~idxZero) = ... % for the rotational velocity case 4)
                computeInvOfTransVelPerturbMatrix(... % compute the transformation pages
                reshape(gPages(3, :, ~idxZero), 1, 1, nnz(~idxZero))... % get it into [3 x 1 x "nPages"] format
                                            );
            gCircPagesXY = pagemtimes(MinvPages, ... % get the translational components of the Lie algebra elements
                reshape(gPages(1:2, :, :), [2, 1, nPoints])); 
            gCircPages = nan(3, 1, nPoints);
            gCircPages(3, :, :) = gPages(3, :, :); % rotations transfer as such
            gCircPages(1:2, :, :) = gCircPagesXY; % get the translational components
            gCirc = permute(gCircPages, [1, 3, 2]); % permute dimensions into a matrix similar to 'g' (only 2D) 5)
            if transposeFlag
                gCirc = gCirc'; % if transposed bring convert to original coords and return 6)
            end
    end

end

%% AUXILIARY FUNCTIONS

% compute the inverse of the arc-travel perturbation matrix acting on the 
% translational displacement obtained using the rotation displacement
% ... for more details on the original transform, refer to
% ... "exponentiateLieAlgebraElement.m"'s auxiliary function which has the
% ... definition of the actual exponential map
function Minv = computeInvOfTransVelPerturbMatrix(th)
    Minv = th/2.*[  sin(th)./(1 - cos(th)), ones(size(th)); 
                  -1*ones(size(th)), sin(th)./(1 - cos(th))  ];
end