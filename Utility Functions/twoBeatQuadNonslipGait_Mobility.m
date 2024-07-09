classdef twoBeatQuadNonslipGait_Mobility
    %TWOBEATQUADNONSLIPGAIT using two "Path2_Mobility" instances, we
    %generate a quadrupedal nonslip two-beat gait.
    
    properties
        stanceI
        stanceJ
    end
    
    methods
        function obj = twoBeatQuadNonslipGait(inputArg1,inputArg2)
            %TWOBEATQUADNONSLIPGAIT Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end

    methods (Static)
    end

end

