% This function plots the vector fields and heatmaps associated with the
% kinematics of the level-2 contact states of a quadrupedal robot.
function axC = plotchildlayout(C,pk,pi)
    
    axC = cell(1,numel(C)); % stores the axes cell array
    for i = 1:numel(C) % iterate over the provided layouts
        
        if C{i}.flag % if this is a valid layout
            switch C{i}.vecF
            
                case 0 % heat-map
                    if C{i}.flag
                        axC{i} = cell(1,prod(C{i}.grid)); % axes for each plot in a layout
                        for j = 1:prod(C{i}.grid)
                            axC{i}{j} = nexttile(C{i}.Layout_Obj,j);
                            plotcase1heatmap(axC{i}{j},pk,pi,C{i},j);
                            if C{i}.idx{2}(C{i}.idx{1} == j) == C{i}.start(1) - 1 + C{i}.grid(1) % plot colorbar
                                if C{i}.idx{3}(C{i}.idx{1} == j) == C{i}.start(2) - 1 + ceil(C{i}.grid(2)/2)
                                    cbi = colorbar(axC{i}{j},'TickLabelInterpreter','latex','FontSize',pi.cbarFS);
                                    cbi.Layout.Tile = 'South';
                                end
                            end
                        end
                    end
    
                case 1 % vector field
                    if C{i}.flag % check if this field is requested
                        axC{i} = cell(1,prod(C{i}.grid));
                        for j = 1:prod(C{i}.grid) % iterate
                            axC{i}{j} = nexttile(C{i}.Layout_Obj,j); % set the axis
                            plotcase1vectorfield(axC{i}{j},pk,pi,C{i},j); % plot the vector field
                        end
                    end
    
            end
            
        end

    end

end