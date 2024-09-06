function compileWaveformsFromFSappx(stanceIstruct, stanceJstruct, tableName)
%COMPILEWAVEFORMSFROMFSAPPX compiles the gait phase representation of the
%waveforms and ouputs a csv file with its fourier series representation

    % unpack
    csI = stanceIstruct.cs;
    tauI = stanceIstruct.tau;
    subgaitI = stanceIstruct.subgait;
    nAppxOrderI = stanceIstruct.nAppxOrder;
    csJ = stanceJstruct.cs;
    tauJ = stanceJstruct.tau;
    subgaitJ = stanceJstruct.subgait;
    nAppxOrderJ = stanceJstruct.nAppxOrder;

    % obtain the sorting order for the concatenated stance indices
    [~, sortIdx] = sort([csI, csJ]); numLimbs = numel(sortIdx);

    % obtain the FS approximation
    % ... obtain the coefficients columnwise for different limbs
    % ... concatenate the results into 'coeffs' matrix
    % ... use the sorting order to arrange it into the each limb indexed in
    % ... ascending order from the left to right
    [a0I, akI, bkI, ~] = generateFourierSeriesAppx(tauI, ...
                                                subgaitI, nAppxOrderI);
    [a0J, akJ, bkJ, ~] = generateFourierSeriesAppx(tauJ, ...
                                                subgaitJ, nAppxOrderJ);
    switch nAppxOrderI ~= nAppxOrderJ
        case 1 % the orders are different
            if nAppxOrderI > nAppxOrderJ
                nAppxOrder = nAppxOrderI;
                coeffs = [a0I, a0J;
                          akI, [akJ; zeros(nAppxOrderI-nAppxOrderJ, size(akJ, 2))];
                          bkI, [bkJ; zeros(nAppxOrderI-nAppxOrderJ, size(bkJ, 2))]];
            elseif nAppxOrderJ > nAppxOrderI
                nAppxOrder = nAppxOrderJ;
                coeffs = [a0I, a0J;
                         [akI; zeros(nAppxOrderI-nAppxOrderJ, size(akI, 2))], akJ;
                         [bkI; zeros(nAppxOrderI-nAppxOrderJ, size(bkI, 2))], bkJ];
            end
        case 0 % the orders are the same
            nAppxOrder = nAppxOrderI;
            coeffs = [a0I, a0J;
                      akI, akJ;
                      bkI, bkJ];
    end
    coeffs = coeffs(:, sortIdx);

    % obtain the table and write to csv
    % ... make a table, the rows are names of the coefficient
    % ... the columns are limb index number (for e.g. 'limb 1')
    % ... we write to file using writetable command
    coefficientNames = ['a0', ...
        strcat(repmat(cellstr('a'), 1, nAppxOrder), ...
                                    strsplit(num2str(1:nAppxOrder))), ...
        strcat(repmat(cellstr('b'), 1, nAppxOrder), ...
                                    strsplit(num2str(1:nAppxOrder)))...
                        ];
    FStable = array2table(...
            coeffs,...
            'DimensionNames', {'coeffs', 'limbs'}, ...
            'RowNames', coefficientNames,...
            'VariableNames', strsplit(num2str(1:numLimbs))...
                    );
    writetable(FStable, tableName, 'WriteRowNames', true);


end

