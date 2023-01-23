% This is a "Path2" class for defining paths on level-2 no-slip contact
% submanifolds for a rigid quadrupedal robot. It takes the initial 
% condition for the shape-space slice (2 dim), gait constraint vector field
% (2 dim), and integration time for the path to construct a Path2 object. 
% A subclass of "Gait2" is passed as a property.

classdef Path2 < RigidGeomQuad

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (SetAccess = private)

        dq                   % Gait constraint-based configuration vector-field dqij-- [A; E]*\Vec{d\phi}_{ij}

        dphi                 % Gaitconstraint vector field dphi_ij-- \Vec{dphi}_{ij}

        path_start           % starting point to compute the path

        int_time             % Integration time in the backward and forward direction from the middle_path

        deadband_dutycycle   % period of time spent swining (0 <= val <= 1)

    end

    properties (SetAccess = private)
        
        open_trajectory                                            % configuration trajectory for the active path

        closed_trajectory                                          % configuration trajectory for the whole path

        path_length                                                % length of the gait

    end

    methods
        
        % Constructor
        function [thisPath2] = Path2(ank, a, l, dzdphi, dphi, strpt, t, dc)

            % Setup the requirements for the arguments
            arguments
                
                ank     (1, 1) double {mustBeGreaterThan(ank,0.1)}

                a       (1, 1) double {mustBeGreaterThan(a,0.1)}

                l       (1, 1) double {mustBeGreaterThan(l,0.1)}

                dzdphi  (5, 1) sym    {mustBeA(dzdphi, 'sym')}

                dphi    (2, 1) sym    {mustBeA(dphi, 'sym')}

                strpt   (1, 2) double {mustBeNumeric}

                t       (1, 2) double {mustBeNonnegative}

                dc      (1, 1) double {mustBeNonnegative, mustBeLessThanOrEqual(dc, 1)}

            end

            % Get the arguments for a superclass constuctor
            if nargin == 8
                quadArgs = [ank, a, l];
            elseif nargin == 6
                quadArgs = ank;
            elseif nargin == 5
                quadArgs = [];
            else
                error('Error: Need 8, 6, or 5 arguments to create an object.');
            end

            % call the RigidGeometricQuadruped class' constructor
            thisPath2 = thisPath2@RigidGeomQuad(quadArgs);

            % assign the props
            thisPath2.dq = dzdphi;
            thisPath2.dphi = dphi;
            thisPath2.path_start = strpt;
            thisPath2.int_time = t;
            thisPath2.deadband_dutycycle = dc;

            % increment the number of objects
            Path2.SetGet_static(1);

        end


    end

    methods (Static)
        
        % static function to icnrement the number of objects
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

        % Compute the open_trajectory
        function thePath2 = compute_open_trajectory(thePath2, funcstr)

            % discretization
            dnum = 101;

            % RigidGeometricQuadruped 's inherited props
            a = thePath2.get_a; 
            l = thePath2.get_l;
            
            % integrate the gait constraint ode to obtain the open-trajectory for the system.
            ai0 = thePath2.path_start(1);  % initial conditions
            aj0 = thePath2.path_start(2);

            % get the functions needed to integrate-- symbolic to functions
            eval(funcstr{1})
            dq = matlabFunction(thePath2.dq, 'Vars', eval(funcstr{2}));

            % integrate
            t = linspace(0, thePath2.int_time(1), dnum); % backward
            [tb,qb,~,~,~] = ode45( @(t,y) -dq(t, a, l, y(4), y(5)), t, [zeros(3,1); ai0; aj0] );
            t = linspace(0, thePath2.int_time(2), dnum); % forward
            [tf,qf,~,~,~] = ode45( @(t,y) dq(t, a, l, y(4), y(5)), t, [qb(end,1:3)'; ai0; aj0] );
            
            % solutions are concatenated such that [ai0; aj0] is the mid point of the shape-space path
            tf = tf(:); tb = tb(:); tf = tb(end) + tf;
            tb = flipud(tb(end) - tb(2:end));
            q = [qb; qf];

            % return the open configuration trajectory slice q(s)_ij
            thePath2.open_trajectory = {[tb; tf]', q(:,1)', q(:,2)', q(:,3)', q(:,4)', q(:,5)'}';
            % {x, y, \theta, \alpha_i, \alpha_j}

        end

        % compute the closed_trajectory
        function thePath2 = compute_closed_trajectory(thePath2)
            
            % unpack your open_trajectory
            t = thePath2.open_trajectory{1};
            x = thePath2.open_trajectory{2};
            y = thePath2.open_trajectory{3};
            theta = thePath2.open_trajectory{4};
            ai = thePath2.open_trajectory{5};
            aj = thePath2.open_trajectory{6};
            
            % get the length of the computed open trajectory
            dnum_active = numel(thePath2.open_trajectory{1});
            
            % get the number of points needed in the deadband
            dnum_dead = round(thePath2.deadband_dutycycle*dnum_active);

            % get the deadband configuration trajectories q_d
            t_d = [t, t(end) + t(end)/(dnum_active - 1)*(1:dnum_dead)];
            x_d = [x, x(end)*ones(1, dnum_dead)];
            y_d = [y, y(end)*ones(1, dnum_dead)];
            theta_d = [theta, theta(end)*ones(1, dnum_dead)];
            temp_i = linspace(ai(end), ai(1), dnum_dead + 2); temp_i = temp_i(2:end-1);
            ai_d = [ai, temp_i];
            temp_j = linspace(aj(end), aj(1), dnum_dead + 2); temp_j = temp_j(2:end-1);
            aj_d = [aj, temp_j];

            % create the closed configuration trajectory slice q(phi)_ij
            thePath2.closed_trajectory = {[t, t_d], [x, x_d], [y, y_d], [theta, theta_d], [ai, ai_d], [aj, aj_d]}';
            
        end

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
