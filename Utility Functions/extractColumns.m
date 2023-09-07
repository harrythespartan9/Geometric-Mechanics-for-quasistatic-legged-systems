function Mprime = extractColumns(M, cols)
%EXTRACTCOLUMNS extract the corresponding columns to a relevant shape space slices for different contact states
%   Given an input matrix `M' (or cells of matrices) and a selection of columns `cols', the output of this function will be the truncated matrix Mprime with 
%   the sorted columns listed in `cols'.
    
    % Ensure that the number of arguments are two
    if nargin ~= 2
        error('ERROR! There must be two inputs: 1) matrix "M" and 2) column selection "cols".');
    end
    cols = int8(sort(cols)); % sort and integerize the cols input
    
    % Treat M casewise
    switch iscell(M)


        case 0 % single matrix
            
            % Ensure that the inputs are numeric
            if ~(isnumeric(M) || isa(M, "sym")) || ~isnumeric(cols)
                error(['ERROR! Both inputs need to have compatible datatypes-- ' ...
                    '1) M has to be numeric or symbolic and 2) cols has to be numeric.']);
            end
        
            % Ensure that the entries in cols are well conditioned
            if max(cols) > size(M, 2)
                error('ERROR! The max column to be appended can not be greater than the number of columns in M.');
            end
            if min(cols) < 1
                error('ERROR! The column entries need to be positive integers.');
            end
        
            % return
            Mprime = M(:, cols);


        case 1 % a cell collection of matrices
            
            % error checks
            if min(cols) < 1
                error('ERROR! The column entries need to be positive integers.');
            end
            for i = 1:numel(M)
                if ~(isnumeric(M{i}) || isa(M{i}, "sym")) || ~isnumeric(cols)
                    error(['ERROR! Both inputs need to have compatible datatypes-- ' ...
                        '1) ' num2str(i) 'th matrix cell has to be numeric or symbolic and 2) cols has to be numeric.']);
                end
                if max(cols) > size(M{i}, 2)
                    error('ERROR! The max column to be appended can not be greater than the number of columns in M.');
                end
            end
            
            % iterate again and get the output matrices
            Mprime = cell(size(M));
            for i = 1:numel(M)
                Mprime{i} = M{i}(:, cols);
            end

    end

end