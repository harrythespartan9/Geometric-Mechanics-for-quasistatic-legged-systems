% This is a "Path2" class for defining paths on level-2 no-slip contact
% submanifolds for a quadrupedal robot. It takes the initial condition for
% the shape-space slice (2 dim), gait constraint vector field (2 dim), and
% integration time for the path to construct a Path2 object. A subclass of
% "Gait2" is passed as a property.

classdef Path2 < handle

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Access = private)

        path_constr

        init_cond

        int_time

        Gait2

    end

    properties (Access = private, Dependent)
        
        open_trajectory
        % the subclass gait will have a 'closed_trajectory' prop

    end

    methods
        
        % Constructor
        function [thisPath2] = Path2(dphi, ic, t)
            
            % assign the props
            thisPath2.path_constr = dphi;
            thisPath2.init_cond = ic;
            thisPath2.int_time = t;

            % increment the number of objects
            Path2.SetGet_static(1);

        end

        % Compute the trajectory-- dependent prop
        function open_trajectory = get.open_trajectory(thePath2)
            
            % Integrate the gait constraint ode to obtain the
            % open-trajectory for the system.

        end


    end

    methods (Static)
        
        % Static function to icnrement the number of objects
        function out = SetGet_static(~)
            
            persistent var

            if isempty(var)
                var = 0;
            end

            if nargin == 1

                var = var + 1;

            else

                out = var;

            end

        end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end