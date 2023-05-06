function out = expkin_contact_thresholding(in ,t)
%EXPKIN_CONTACT_THRESHOLDING This sets a minimum threshold on the z-ht of legs to determine a contact trajectory.
%   Given the z-ht of each leg from the experimental kinematics data, the leg is thresh-held at 3% of the BL of HAMR. The value of 3% is determined from this
%   paper DOI: 10.1109/IROS.2017.8206249

    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            out{i} = (in{i}(3, :) < t);
        end
    else
        error('ERROR! The input needs to be a cell array.')
    end

end

