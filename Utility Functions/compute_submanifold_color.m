% This function computes the current submanifold and the corresponding color, when provided with a contact trajectory, contact change trajectory, and contact
% data for the kinematics of the system being analyzed. No input concistency checks are done in this function-- ensure you're
function [tS, tcol] = compute_submanifold_color(C, delC, kin) % , tcirc

    % Unpack

    
    % initialize
    t = numel(C{1});
    tS = cell(1, t);
    tcol = cell(1, t);
    % tcirc = cell(1, t);
    temp = cell(1, t);
    S = kin.contact.submanifolds;
    Slvl = kin.contact.submanifold_level;
    col = kin.contact.color;

    % iterate over each time-step
    for i = 1:t

        switch i



            case 1

                for j = 1:4
                    if C{j}(i) == 1
                        temp{i} = [temp{i}, j]; % active feet
                    end
                end
                if isempty(temp{i})
                    tcol{i} = [100, 100, 100]/255;
                    tS{i} = nan; 
                    % tcirc{i} = 25;
                else
                    c = zeros(size(S));
                    for k = 1:numel(temp{i})
                        c = c + (temp{i}(k) == S);
                    end
                    c = sum(c, 2); tS{i} = find(c == numel(temp{i}) & Slvl == numel(temp{i}));          % current submanifold
                    tcol{i} = col(tS{i}, :);                                                            % associated color
                    % tcirc{i} = 100;                                                                     % associated animation scatter size
                end

            otherwise
                
                % if contact change happens
                if delC(i-1) == 1
                    for j = 1:4
                        if C{j}(i) == 1
                            temp{i} = [temp{i}, j];
                        end
                    end
                    if isempty(temp{i})
                        tcol{i} = [100, 100, 100]/255;
                        tS{i} = nan; 
                        % tcirc{i} = 25;
                    else
                        c = zeros(size(S));
                        for k = 1:numel(temp{i})
                            c = c + (temp{i}(k) == S);
                        end
                        c = sum(c, 2); tS{i} = find(c == numel(temp{i}) & Slvl == numel(temp{i}));
                        tcol{i} = col(tS{i}, :);
                        % tcirc{i} = 100;
                    end
                else 
                    tS(i) = tS(i - 1);                                                          % same submanifold, color, and scat_size as the last time step
                    tcol{i} = tcol{i - 1};
                    % tcirc{i} = tcirc{i - 1};
                end



        end

    end
    

end