% This function first computes the parent location of a child layout based 
% on it's size and starting location. Next, the indices of the tiles wrt to
% the parent are computed in 3 different ways, sequentially, just the row
% index and the column index are computed. Finally, we also compite the
% associated sweep data text to each tile to call the plot functions later.
% Inputs: parent grid size 'P.grid', starting row and col index of the child
% location, child grid size 'C.grid', is this a vector field flag 'vF', and the
% sweep text data structure which lists the heat map or vector field sweep
% data text for each child layout tile. This gets updated with each call of
% this function.
function [C, kin_text] = ...
    tileprops(p_i, P, C, vF, kin_text)

% Check if the sweep text structure is provided
if nargin < 5
    kintxtF = false;
else
    kintxtF = true;
end

% Starting tile number when converting the matrix of the tile layout into
% an array
C.tile_start = (C.start(1)-1)*P.grid(2) + C.start(2);

% Number of tiles in the layout
C.num = C.grid(1)*C.grid(2);

% Compute the next-tile linear indices and corresponding two element
% indices now--------------------------------------------------------------
temp_idx = nan(P.grid); % initialize the matrix
row_temp_idx = temp_idx; col_temp_idx = temp_idx; % initialize index values
temp_idx(C.start(1):C.start(1)+C.grid(1)-1,...
    C.start(2):C.start(2)+C.grid(2)-1) = reshape(find(ones(C.grid)),C.grid(2),C.grid(1))';
row_temp_idx(C.start(1):C.start(1)+C.grid(1)-1,...
    C.start(2):C.start(2)+C.grid(2)-1) = repmat((C.start(1):C.start(1)+C.grid(1)-1)',1,C.grid(2));
col_temp_idx(C.start(1):C.start(1)+C.grid(1)-1,... % col index values
    C.start(2):C.start(2)+C.grid(2)-1) = repmat((C.start(2):C.start(2)+C.grid(2)-1),C.grid(1),1);
C.idx{1} = temp_idx; % child tile values in the parent grid
C.idx{2} = row_temp_idx; % corresponding row index values
C.idx{3} = col_temp_idx; % corresponding col index values

% Compute the child layout text -------------------------------------------
% Initialize the heat map and vector field sweep text data if this is the
% first child layout properties call
dF = 1; % get the data from the text funtion.
switch vF

    case false % heat map text comp

        if kintxtF % initialize the cell array or obtain from inputs
            if isfield(kin_text,'hS_txt')
                hS_txt = kin_text.hS_txt;
            else
                hS_txt = cell(P.grid);
            end
        else
            hS_txt = cell(P.grid);
        end
            
        for i = 1:C.num % iterate for each tile
            hS_txt{C.idx{2}(C.idx{1} == i), C.idx{3}(C.idx{1} == i)}...
                = ['plot_kin.',...
                kintext(p_i.comps(p_i.R_comps == C.idx{2}(C.idx{1} == i)),...
                p_i.fields(p_i.C_fields == C.idx{3}(C.idx{1} == i)),dF),...
                '_sweep'];
        end

        kin_text.hS_txt = hS_txt; 
        % update the sweep text cell array

    case true % vector field text comp

        if kintxtF
            if isfield(kin_text,'vfS_txt')
                vfS_txt = kin_text.vfS_txt;
            else
                vfS_txt = cell([P.grid,2]);
            end
        else
            vfS_txt = cell([P.grid,2]);
        end

        for i = 1:C.num
            vfS_txt{C.idx{2}(C.idx{1} == i), C.idx{3}(C.idx{1} == i), 1}...
                = ['plot_kin.',...
                kintext(p_i.comps(p_i.R_comps == C.idx{2}(C.idx{1} == i)),...
                p_i.fields(p_i.C_fields == C.idx{3}(C.idx{1} == i)),dF),...
                '_1_sweep']; % component 1
            vfS_txt{C.idx{2}(C.idx{1} == i), C.idx{3}(C.idx{1} == i), 2}...
                = ['plot_kin.',...
                kintext(p_i.comps(p_i.R_comps == C.idx{2}(C.idx{1} == i)),...
                p_i.fields(p_i.C_fields == C.idx{3}(C.idx{1} == i)),dF),...
                '_2_sweep']; % component 2
        end

        kin_text.vfS_txt = vfS_txt;

end

dF = 0; % compute the title text
if kintxtF
    t_txt = kin_text.t_txt;
else
    t_txt = cell([P.grid,2]);
end
for i=1:C.num
    t_txt{C.idx{2}(C.idx{1} == i), C.idx{3}(C.idx{1} == i)}...
        = kintext(p_i.comps(p_i.R_comps == C.idx{2}(C.idx{1} == i)),...
                p_i.fields(p_i.C_fields == C.idx{3}(C.idx{1} == i)),dF);
end
kin_text.t_txt = t_txt;

end