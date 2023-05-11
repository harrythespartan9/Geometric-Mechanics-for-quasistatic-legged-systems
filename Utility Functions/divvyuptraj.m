function [u, out] = divvyuptraj(in)
%DIVVYUPTRAJ divide the given contact trajectory into chunks based on the unchanging contact.
%   Given and input discrete state trajectory, we identify unique, jointed periods of unchanging states. This is useful when plotting functions trajectories
%   where we want to highlight different contact states.
    out = cell.empty(0, 0);
    [u, ~, iU] = unique( double(cell2sym(in)) );

    for i = 1:numel(u)

        idx_state_i = find( iU == u(i) );

        diff_idx_i = diff(idx_state_i);
        idx_i_switch = find(diff_idx_i > 1);
        out{i} = cell( 1, numel(idx_i_switch) + 1 );

        idx_start = 1;

        for j = 1 : numel(idx_i_switch)

            idx_end = idx_i_switch(j);
            out{i}{j} = idx_state_i(idx_start:idx_end); out{i}{j}(end+1) = out{i}{j}(end) + 1;
            idx_start = idx_end + 1;

            if j == numel(idx_i_switch) % if it is the final switch
                idx_end = numel(idx_state_i);
                out{i}{j+1} = idx_state_i(idx_start:idx_end); out{i}{j+1}(end+1) = out{i}{j+1}(end) + 1;
            end

        end

    end


end

