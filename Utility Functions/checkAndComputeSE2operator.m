function [check, g_out] = checkAndComputeSE2operator(g_in, operator_flag)
%CHECKSE2COMPATIBILITY checks the compatibility of an input SE(2) group element or sequence of elements

    % compute validity and format
    switch iscell(g_in)
        case 0
            if numel(g_in) ~= 3 && sum(size(g_in) == [3, 3]) ~= 2
                check = 0; g_out = []; return; % invalid
            else
                check = 1;
                switch numel(g_in)
                    case 3
                        format = 1; % current format
                    otherwise
                        format = 2;
                end
            end
        case 1
            for i = 1:numel(g_in)
                if numel(g_in{i}) ~= 3 && sum(size(g_in{i}) == [3, 3]) ~= 2
                    check = 0; g_out = []; return;
                end
                if i == 1
                    switch numel(g_in)
                        case 3
                            format = 1; % current format (hopefull all elements are the same)
                        otherwise
                            format = 2;
                    end 
                end
            end
            check = 1;
    end
    
    % compute the operator form if needed
    if operator_flag
        switch format
            case 2
                g_out = SE2_vec2mat(g_in);
            otherwise
                g_out = in;
        end
    else
        g_out = [];
    end

end

