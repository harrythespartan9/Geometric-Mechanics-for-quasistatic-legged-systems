% This function computes the adjoint-inverse pullback matrix for moving velocities from body frame to global frame.
function out = TeLg(in)

    out = [cos(in), -sin(in), 0;
           sin(in),  cos(in), 0;
           0,              0, 1];

end