function q = inverse_kinematics_xyz(x, y, z)
% IK numerica basata sulla fk_robot_13

    p_des = [x; y; z];
    persistent q_prev

    if isempty(q_prev)
        q_prev = [0 0 0 0];   % guess iniziale
    end

    % funzione costo per fminsearch
    cost = @(q) norm( fk_robot_13(q) - p_des )^2;

    opt = optimset('Display','off', 'TolX', 1e-6, 'TolFun', 1e-8);
    q_sol = fminsearch(cost, q_prev, opt);

    q = q_sol;
    q_prev = q_sol;
end
