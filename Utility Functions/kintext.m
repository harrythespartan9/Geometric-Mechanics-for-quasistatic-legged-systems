% This function computes the sweep data and lateX interepreted title text 
% based on the current field and component of the child layout under 
% consideration.
function txt = kintext(comp,field,flag)

switch flag %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case true % if data is requested

        % Check the field
        if strcmp('-A',field)
            txt = 'A';
        elseif strcmp('-dA',field)
            txt = 'mdA';
        elseif strcmp('[A_1,A_2]',field)
            txt = 'lb_A';
        elseif strcmp('D(-A)',field)
            txt = 'D_mA';
        elseif strcmp('dz',field)
            txt = 'dz';
        end
        
        % Check the component
        if strcmp('x',comp)
            txt = [txt, '__x'];
        elseif strcmp('y',comp)
            txt = [txt, '__y'];
        elseif strcmp('theta',comp)
            txt = [txt, '__theta'];
        end

    case false % if title text is requested
        
        F = false; DF = false;
        if strcmp('-A',field)
            txt = '$$-\mathbf{A}';
        elseif strcmp('-dA',field)
            txt = '$$-d\mathbf{A}';
        elseif strcmp('[A_1,A_2]',field)
            txt = '$$[\mathbf{A}_{1},\mathbf{A}_{2}]';
        elseif strcmp('D(-A)',field)
            DF = true;
            txt = '$$D(-\mathbf{A}';
        elseif strcmp('dz',field)
            F = true;
            txt = '$$dz_{\phi}';
        end
        
        if DF % if generalized curl is needed
            if strcmp('x',comp)
                txt = [txt, '^{x})$$'];
            elseif strcmp('y',comp)
                txt = [txt, '^{y})$$'];
            elseif strcmp('theta',comp)
                txt = [txt, '^{\theta})$$'];
            end
        else % if it is everything else
            if strcmp('x',comp)
                if F
                    txt = [txt, '^{x} = -\mathbf{A}^{x} \Delta\alpha_F$$'];
                else
                    txt = [txt, '^{x}$$'];
                end
            elseif strcmp('y',comp)
                txt = [txt, '^{y}$$'];
            elseif strcmp('theta',comp)
                txt = [txt, '^{\theta}$$'];
            end
        end

end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end