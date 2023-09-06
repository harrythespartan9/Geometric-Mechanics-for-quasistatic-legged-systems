function out = colwise_Dmat(M, v)
%COLWISE_DMAT takes the columnwise gradient of an input symbolic matrix given a set of symbolic variables as a vector
%   The columnwise jacobian of a sym input matrix 'M' is taken with respect to each sym variable in a sym vector 'v' and is then stored in cell arrays.

    % Check if both inputs are symbolic, else send an error
    if ~isa(M, 'sym') || ~isa(v, 'sym')
        error('ERROR! Both input matrix and variables need to have symbolic datatypes.');
    end

    % Check the number of columns in the matrix and iterate
    switch size(M, 2)
        case 0
            error('ERROR! The input matrix can not be empty.');
        case 1
            out = jacobian(M, v); % directly output the jacobian
        otherwise
            out = cell(1, size(M, 2));
            for i = 1:size(M, 2)
                out{i} = simplify(jacobian(M(:,i), v)); % compute the jacobian columnwise
            end
    end

end

