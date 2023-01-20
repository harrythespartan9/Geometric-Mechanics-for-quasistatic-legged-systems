classdef (Abstract) RigidGeomQuad <  handle

    properties (Access = private)
        
        leg_link_ratio (1,1) double {mustBeGreaterThan(leg_link_ratio,0.1)}...
             = 1;

        link_length (1,1) double {mustBeGreaterThan(link_length,0.1)}...
            = 1;

        ankle_limit (1,1) double {mustBeGreaterThan(ankle_limit,0.1)}...
            = pi/2;

    end

    methods


        function [thisRigidGeomQuad] =...
                RigidGeomQuad(quadArgs)
            
            if ~isempty(quadArgs)

                if nargin == 1
                    thisRigidGeomQuad.ankle_limit = quadArgs;
                elseif nargin  == 3
                    thisRigidGeomQuad.leg_link_ratio = quadArgs(2);
                    thisRigidGeomQuad.link_length = quadArgs(3);
                    thisRigidGeomQuad.ankle_limit = quadArgs(1);
                else
                    error(['ERROR: the number of input arguments should be'...
                        ' 1 or 3.']);
                end

            end


        end


    end



end