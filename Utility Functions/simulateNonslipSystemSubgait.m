% compute the nominal system subgait displacement when provided with
% reference point, integration times
function z = simulateNonslipSystemSubgait(refPt, startEndTimes, ... % shape refernce and integration times
                                            delAlpha, dQ, ... % shape velocity
                                                args, mode) % args for configuration velocity and system type
    % general unpacking and computing
    % ... input args for dQ to run the ode solver, and everything else next
    % ... computation is carried out based on the current mode
    tIC = startEndTimes(1); tFC = startEndTimes(2);
    switch mode
        case 'clari'
            na = args{1};
            nl = args{2};
            nalpha_b = args{3}; % our sprawl setup
            % ... compute the displacement
            % ... ... first find the initial condition
            % ... ... next simulate forward and return the displacement
            if tIC ~= tFC
                if tIC ~= 0
                    [~, a0] = ode89( @(t, y)...
                        delAlpha(na, nl, nalpha_b, y(1), y(2)), ...
                        [0, startEndTimes(1)], refPt ); % get shape ic
                else
                    a0 = refPt; % same ic for final path if tIC == 0
                end
                [~, solnY] = ode89( @(t, y) dQ(na, nl, nalpha_b,... % body fc
                                            y(1), y(2), y(3),...
                                            y(4), y(5)),...
                                            [0, tFC-tIC], ...
                                            [zeros(1, 3), a0(end, :)] );
                z = solnY(end, 1:3);
            else % no path, just point => no body trajectory/displacement
                z = zeros(1, 3);
            end
        case 'arbQuad' % very similar, can be fleshed out when needed
            z = zeros(1, 3); % CHANGE THIS
        otherwise
            error(['ERROR! Only modes "clari" and "arbQuad" ' ...
                '(for arbitrary quadruped) are supported at the moment.']);
    end
    
end