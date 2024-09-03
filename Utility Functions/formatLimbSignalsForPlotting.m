function limbAngleStruct = formatLimbSignalsForPlotting(tau, beta, subgait, ...
                                            stanceObj, scatterFlag, lineSty)
%FORMATLIMBSIGNALSFORPLOTTING format limb signals for plotting them 
%serially

    % if no scatterFlag is provided and line styles are provided, leave
    % those fields empty
    if nargin < 6
        lineSty = [];
    end
    if nargin < 5
        scatterFlag = [];
    end

    % Check for number of traces in subgait (coordinates) and plot them
    % separately
    switch size(subgait, 2)
        case 1 % just one trace
            limbAngleStruct.tau = tau;
            limbAngleStruct.beta = beta;
            limbAngleStruct.signal = subgait;
            limbAngleStruct.stanceColor = stanceObj.p_info.gc_col;
            limbAngleStruct.swingColor = stanceObj.p_info.col(end, :);
            limbAngleStruct.scatterFlag = scatterFlag;
            limbAngleStruct.lineSty = lineSty;
            limbAngleStruct.cs = stanceObj.cs(1); % if only one signal is 
                                                  % provided, we assume it 
                                                  % is the first one by 
                                                  % default
        otherwise % more traces (we shall be using this a lot more)
            limbAngleStruct = cell(size(subgait, 2), 1);
            for i = 1:size(subgait, 2)
                limbAngleStruct{i}.tau = tau;
                limbAngleStruct{i}.beta = beta;
                limbAngleStruct{i}.signal = subgait(:, i);
                limbAngleStruct{i}.stanceColor = stanceObj.p_info.gc_col;
                limbAngleStruct{i}.swingColor = stanceObj.p_info.col(end, :);
                limbAngleStruct{i}.scatterFlag = scatterFlag;
                limbAngleStruct{i}.lineSty = lineSty;
                limbAngleStruct{i}.cs = stanceObj.cs(i); % this will error 
                                                         % out if 'cs' 
                                                         % field doesn't 
                                                         % have enough 
                                                         % terms
            end
    end

end

