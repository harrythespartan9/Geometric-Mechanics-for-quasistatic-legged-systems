classdef (Abstract) RigidGeomQuad <  handle

    properties (Access = private)
        
        leg_link_ratio (1,1) double {mustBeGreaterThan(leg_link_ratio,0.1)}...
             = 1;

        link_length (1,1) double {mustBeGreaterThan(link_length,0.1)}...
            = 1;

        ankle_limit (1,1) double {mustBeGreaterThan(ankle_limit,0.1)}...
            = pi; % to explore the full range of leg swing kinematics

    end

    methods


        function [thisRigidGeomQuad] =...
                RigidGeomQuad(quadArgs)
            
            if ~isempty(quadArgs)

                if numel(quadArgs) == 1
                    thisRigidGeomQuad.ankle_limit = quadArgs;
                elseif numel(quadArgs) == 3
                    thisRigidGeomQuad.leg_link_ratio = quadArgs(2);
                    thisRigidGeomQuad.link_length = quadArgs(3);
                    thisRigidGeomQuad.ankle_limit = quadArgs(1);
                else
                    error(['ERROR: the number of input arguments should be'...
                        ' 1 or 3.']);
                end

            end


        end

        function out = get_a(obj)
            
            out = obj.leg_link_ratio;

        end

        function out = get_l(obj)
            
            out = obj.link_length;

        end
        
        function out = get_ank(obj)
            
            out = obj.ankle_limit;

        end

    end


end