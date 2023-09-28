function Aout = reduce_2beatgait_noslip_connection(Ain, idxi, idxj, Grad)
    % This function replaces the sprawl's group derivative with the rest of the accessible shape space.
    Ain(:, idxi+1) = Ain(:, idxi+1) - Ain(:, 1)*Grad(1)/Grad(3); 
    Ain(:, idxj+1) = Ain(:, idxj+1) - Ain(:, 1)*Grad(2)/Grad(3); 
    Aout = simplify(Ain(:, 2:end), "Steps", 10, "IgnoreAnalyticConstraints", true);
end