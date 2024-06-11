% This is a "Path2_Mobility" class for defining paths on level-2 no-slip 
% contact submanifolds for a rigid quadrupedal robot. It takes the initial 
% condition for the shape-space slice (2 dim), gait constraint vector field
% (2 dim), and integration time for the path to construct a Path2 object. 
% It inherits properties from the abstract class "RigidGeomQuad".

classdef Path2_Mobility < RigidGeomQuad

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (SetAccess = private)

        active_state         % the contact state that this path belongs to, 
                             % i; an integer in the range (1, 6) the 
                             % contact state ordering is as follows: 
                             % {12, 23, 34, 41, 13, 24}

        F                    % squared interfoot distance between legs m 
                             % and n during stance i \in 
                             % {12, 23, 34, 41, 13, 24}

        deltaAlpha           % stance phase constraint vector field-- 
                             % \Delta\alpha_{i}

        A                    % Local connection vector field-- 
                             % \boldsymbol{A}_{i}

        dz                   % Gait constraint based stratified panel dzij
                             % -- [A]*\Delta\alpha_{i}

        reference_point      % point to analyze mobility from
        
        path_color           % color of the trajectory based on the gait 
                             % constraint colormap

    end

    methods
        
        % Constructor
        function [thisPath2] = Path2_Mobility( ank, a, l, ...
                F, delta_alpha, local_connection, stratified_panel, ...
                reference_point, color, lvl2SubmanifoldIdx)

            % Setup the requirements for the arguments
            arguments
                ank                (1, 1) double {mustBeGreaterThan(ank,0.1)}
                a                  (1, 1) double {mustBeGreaterThan(a,0.1)}
                l                  (1, 1) double {mustBeGreaterThan(l,0.1)}
                F                  (1, 1)
                delta_alpha        (2, 1) sym    {mustBeA(delta_alpha, 'sym')}
                local_connection   (3, 2) sym    {mustBeA(local_connection, 'sym')}
                stratified_panel   (3, 1) sym    {mustBeA(stratified_panel, 'sym')}
                reference_point    (1, 2) double {mustBeNumeric}
                color              (1, 3) double {mustBeLessThanOrEqual(color, 1)}
                lvl2SubmanifoldIdx (1, 1) double {mustBePositive, mustBeLessThanOrEqual(lvl2SubmanifoldIdx, 6)}
            end

            % Get the arguments for a superclass constuctor
            if nargin == 9
                quadArgs = [ank, a, l];
            elseif nargin == 7
                quadArgs = ank;
            elseif nargin == 6
                quadArgs = [];
            else
                error(['Error: Need 9, 7, or 6 arguments to create an ' ...
                    'object.']);
            end

            % call the RigidGeometricQuadruped class' constructor
            thisPath2 = thisPath2@RigidGeomQuad(quadArgs);

            % assign the props
            thisPath2.dz = stratified_panel;
            thisPath2.deltaAlpha = delta_alpha;
            thisPath2.A = local_connection;
            thisPath2.reference_point = reference_point;
            thisPath2.path_color = color;
            thisPath2.active_state = lvl2SubmanifoldIdx;

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
        
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end