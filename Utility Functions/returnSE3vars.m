function [R, p] = returnSE3vars(in)

    if isnumeric(in) || isa(in, 'sym') % must be a numeric or a symbolic array ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        % Start the checks
        [R, p] = conditionR_SE3(in, isnumeric(in));
    
    else % if neither, throw an error
        
        error('ERROR! The input is neither a numeric SE(3) transformation matrix, nor a symbolic SE(3) transformation!');
    
    end

end