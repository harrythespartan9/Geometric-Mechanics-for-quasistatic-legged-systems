% compute the location at which the bounds of F are barely violated
function aViolation = ptViolatingFbounds(aa, ll, ...
    F_fxn, F_bounds, F_bounds_full, ...
    refPt, delX, delY, ...
    ~, optLims)

    aViolation = [];

    [aLowViolation, fValViolation] = ...
    fmincon( @(a) abs(F_bounds(1) - ...
        FalongLine(aa, ll, F_fxn, refPt, [delX, delY], a)), ...
        mean(optLims), ...
        [], [], [], [], ...
        optLims(1), optLims(2), ...
        [], ...
        optimoptions("fmincon", "Display", "off") ...
            );
    if fValViolation < 1e-5*diff(F_bounds_full)
        aViolation = aLowViolation;
    else
        [aHighViolation, fValViolation] = ...
        fmincon( @(a) abs(F_bounds(2) - ...
            FalongLine(aa, ll, F_fxn, refPt, [delX, delY], a)), ...
            mean(optLims), ...
            [], [], [], [], ...
            optLims(1), optLims(2), ...
            [], ...
            optimoptions("fmincon", "Display", "off") ...
                );
        if fValViolation < 1e-5*diff(F_bounds_full)
            aViolation = aHighViolation;
        end
    end
end

