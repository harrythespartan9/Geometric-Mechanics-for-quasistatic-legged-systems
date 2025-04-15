function ad_v1__v2 = computeLieBracketOfse2VectorFields...
                                                    (v1, v2)
%COMPUTELIEBRACKETOFVECTORFIELDS computes the lie-bracket of two se(2) 
%vector fields
%   Given two se(2) vector fields (at a single point or multiple points), \
%   this function computes the lie-bracket between them.
    
    % input checks and some basic init
    if nargin ~= 2
        error(['ERROR! Incorrect number of arguments. Only two vector ' ...
            'fields with 3 components each is needed']);
    end
    if size(v1) ~= size(v2)
        error('ERROR! The size of the inputs should be the same.');
    end
    if size(v1, 2) ~= 3
        v1 = v1'; % transpose the inputs 
        v2 = v2';
        if size(v1, 2) ~= 3
            error(['ERROR! The second (last) dimension should have ' ...
                'size 3. So each column is a row vector.']);
        else
            trpFlag = true; % set the transpose flag
        end
    else
        trpFlag = false;
    end
    if isa(v1, "sym")
        error(['ERROR! Lie-bracket of symbolic variables not supported ' ...
            'at the moment.']);
    end
    if isa(v2, "sym")
        error(['ERROR! Lie-bracket of symbolic variables not supported ' ...
            'at the moment.']);
    end

    % initialize output
    ad_v1__v2 = nan(size(v1));

    % compute the bracket and return
    ad_v1__v2(:, 1) = v1(:, 2).*v2(:, 3) - v2(:, 2).*v1(:, 3);
    ad_v1__v2(:, 2) = v2(:, 1).*v1(:, 3) - v1(:, 1).*v2(:, 3);
    ad_v1__v2(:, 3) = zeros(size(ad_v1__v2, 1), 1);

    % return the output in the same format
    if trpFlag
        ad_v1__v2 = ad_v1__v2';
    end

end

