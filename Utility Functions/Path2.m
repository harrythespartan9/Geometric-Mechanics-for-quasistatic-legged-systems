% This is a "Path2" class for defining paths on level-2 no-slip contact
% submanifolds for a rigid quadrupedal robot. It takes the initial 
% condition for the shape-space slice (2 dim), gait constraint vector field
% (2 dim), and integration time for the path to construct a Path2 object. 
% A subclass of "Gait2" is passed as a property.

classdef Path2 < handle & RigidGeomQuad

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (SetAccess = private)

        path_constr (2,1) sym {mustBeA(path_constr, 'sym')}  % path constraint-- dphi
        strat_panel (2,1) sym {mustBeA(strat_panel, 'sym')}  % stratified panel-- A*dphi 
        init_cond (1,2) double {mustBeNumeric}               % path initial condition
        int_time (1,:) double {mustBeNumeric}                % time to integrate for
        Gait2                                                % closed loop gait prop (subclass)

    end

    properties (SetAccess = private, Dependent)
        
        open_trajectory
        % the subclass gait will have a 'closed_trajectory' prop

    end

    methods
        
        % Constructor
        function [thisPath2] = Path2(ank, a, l, dphi, dz, ic, t)

            % Setup the requirements for the arguments
            arguments
                
                ank
                a
                l
                dphi (2,1) sym {mustBeA(dphi, 'sym')}
                dz (2,1) sym {mustBeA(dz, 'sym')}
                ic (1,2) double
                t (1,:) double

            end

            % Get the arguments for a superclass constuctor
            if nargin == 4
                quadArgs = [];
            elseif nargin == 5
                quadArgs = ank;
            elseif nargin == 7
                quadArgs = [ank, a, l];
            else
                error('Error: Need 4 or 7 arguments to create an object.');
            end

            % call the RigidGeometricQuadruped class' constructor
            thisPath2 = thisPath2@RigidGeomQuad(quadArgs);

            % assign the props
            thisPath2.path_constr = dphi;
            thisPath2.strat_panel = dz;
            thisPath2.init_cond = ic;
            thisPath2.int_time = t;

            % increment the number of objects
            Path2.SetGet_static(1);

        end

        % Compute the trajectory-- dependent prop
        function open_trajectory = get.open_trajectory(thePath2)

            % Discretization
            dnum = 101;                   % path disc (preferably odd)

            % RigidGeometricQuadruped 's inherited props
            a = thePath2.leg_link_ratio; 
            l = thePath2.leg_length;
            
            % Integrate the gait constraint ode to obtain the
            % open-trajectory for the system.
            ai0 = thePath2.init_cond(1);  % initial conditions
            aj0 = thePath2.init_cond(2);

            % Vector fields to integrate
            dphi = thePath2.path_constr;
            dz = thePath2.strat_panel;

            % Initialize containers
            T = cell(1,5);                % time vector
            ai = T; aj = T;               % shape-space trajectory
            x = T; y = T; theta = T;      % position-space trajectory

            % Change the direction if the integration time is negative
            % using a flag
            dirnF = 1;

            % Integrate for each case
            for i = 1:numel(thePath2.int_time)

                T{i} = linspace(0, thePath2.int_time(i), dnum);

                [t_temp,a_temp,~,~,~] = ode45( @(t,y) dphi(t, a, l, y(1), y(2)),...
                    T{i}, [ai0; aj0] );
                T{i} = t_temp(:)';  %%%%%%%% REWRITE!!!!
                ai{i} = a_temp(:,1)'; aj{i} = a_temp(:,2)';

                [t_temp,p_temp,~,~,~] = ode45( @(t,y) dz(t, a, l, y(1), y(2)),...
                    T{i}, [ai0; aj0] );
                T{i} = t_temp(:)'; 
                x{i} = p_temp(:,1)'; y{i} = p_temp(:,2)'; theta{i} = p_temp(:,3)';

            end

            % Now, return the configuration trajectory slice q_ij
            open_trajectory = {x, y, z, ai, aj, T}';

        end

        % Condition the trajectory to remain within the shape-space bounds
        function [thePath2] = condition_trajectory(thePath2)

            % Get the ankle limit (RigidGeometricQuadruped 's prop)
            ank = thePath2.ankle;
            
            % condition the trajectories
            path_slice_1 = thePath2.open_trajectory{4};
            path_slice_2 = thePath2.open_trajectory{5};
            path_slice_1 = path_slice_1(path_slice_1 < ank(1) &...
                path_slice_1 > -ank(1));
            path_slice_2 = path_slice_2(path_slice_2 < ank(2) &...
                path_slice_2 > -ank(2));

            thePath2.open_trajectory(:,1) = path_slice_1;
            thePath2.open_trajectory(:,2) = path_slice_2;
            
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