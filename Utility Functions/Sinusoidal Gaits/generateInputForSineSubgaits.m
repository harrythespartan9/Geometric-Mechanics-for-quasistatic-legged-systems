function stanceInputSpace = ...
    generateInputForSineSubgaits(limbParams, maxAmplitude, disc)
%GENERATEINPUTFORSINESUBGAITS Based on the allowed limb angle range, 
%nominal values, and maximum allowable amplitude for sinusoidal shape 
%trajectories, this function computes the two-beat subgait parameter space; 
%the parameter space will be the same for other subgait.

    % ensure that the max amplitude provided is not greater than limb angle
    % limits
    if maxAmplitude > limbParams(2) || maxAmplitude > abs(limbParams(1))
        error(['ERROR! Maximum limb amplitude must not be bigger than the ' ...
            'allowable limb fore-aft angle range.']);
    end
    
    % there are 3-parameters per limb angle, and there are two limb angles 
    % in a quadrupedal subgait
    % ... init amplitude, phases, biases output struct
    nStance = 2;
    stanceInputSpace = [];
    stanceInputSpace.amplitude = cell(1, nStance);
    stanceInputSpace.phase = cell(1, nStance);
    stanceInputSpace.bias = cell(1, nStance);
    
    % setup linear arrays of parmeter variations
    amplitudeArr = linspace(0, maxAmplitude, disc);
    phaseArr = linspace(-pi, pi, disc);
    biasStandinArr = linspace(-1, 1, disc);
    % ... the 'biasStandinArr' will be replaced with actual bias values 
    % ... after generating the initial "ndgrid" because the value for a 
    % ... bias depends on the corresponding amplitude
    
    % use 'ndgrid' to setup the input space
    [a1, p1, b1, a2, p2, b2] = ndgrid( ...
                                amplitudeArr, phaseArr, biasStandinArr, ...
                                amplitudeArr, phaseArr, biasStandinArr ...
                                        );
    
    % scale the biases into angular coordinates using the auxiliary 
    % function defined below
    b1 = angularBiasFromRangeBias( a1, limbParams(1:2), b1 );
    b2 = angularBiasFromRangeBias( a2, limbParams(1:2), b2 );
    
    % pack everything into our return struct as columns of inputs, because
    % we do not need the exact 6 dimensional array
    stanceInputSpace.amplitude{1} = a1(:); 
        stanceInputSpace.amplitude{2} = a2(:);
    stanceInputSpace.phase{1} = p1(:);     
        stanceInputSpace.phase{2} = p2(:);
    stanceInputSpace.bias{1} = b1(:);      
        stanceInputSpace.bias{2} = b2(:);
    
end

%% AUXILIARY FUNCTION

% a function to convert bias into the correct angle format based on
% amplitude sweeps
function biasAngle = ...
        angularBiasFromRangeBias( amplitude, amplitudeLimits, biasRange )
    % compute arrays that hold maximum and minimum allowable bias
    maxAllowableBias = amplitudeLimits(2) - amplitude;
    minAllowableBias = -amplitudeLimits(1) - amplitude;
    % scale the bias range between -1 and 1 to the max and min allowable
    % bias to get it into angular coordinates
    idxPositiveBias = ( biasRange >= 0 );
    idxNegativeBias = ( biasRange <  0 );
    biasRange(idxPositiveBias) = ...
        biasRange(idxPositiveBias).*maxAllowableBias(idxPositiveBias);
    biasRange(idxNegativeBias) = ...
        biasRange(idxNegativeBias).*minAllowableBias(idxNegativeBias);
    % return the bias angle
    biasAngle = biasRange;
end